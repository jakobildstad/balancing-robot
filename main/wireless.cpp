#include "wireless.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "bootloader_random.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

namespace {
constexpr char TAG[] = "WiFi";
constexpr char SSID[] = "BalancingRobot";
constexpr char ORIGIN[] = "http://192.168.4.1";
portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
wireless::Session session;
wireless::Telemetry latest{};

extern const char page_start[] asm("_binary_index_html_start");
extern const char page_end[] asm("_binary_index_html_end");

esp_err_t load_password(char (&password)[33])
{
    nvs_handle_t storage;
    ESP_RETURN_ON_ERROR(nvs_open("robot_wifi", NVS_READWRITE, &storage), TAG, "Open credentials");
    std::size_t size = sizeof(password);
    esp_err_t err = nvs_get_str(storage, "password", password, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // Wi-Fi is still off: temporarily enable the RNG entropy source.
        std::uint8_t random[16];
        bootloader_random_enable();
        esp_fill_random(random, sizeof(random));
        bootloader_random_disable();
        for (std::size_t i = 0; i < sizeof(random); ++i) {
            std::snprintf(password + 2 * i, 3, "%02x", random[i]);
        }
        err = nvs_set_str(storage, "password", password);
        if (err == ESP_OK) err = nvs_commit(storage);
    }
    nvs_close(storage);
    if (err == ESP_OK && std::strlen(password) != 32) return ESP_ERR_INVALID_STATE;
    return err;
}

esp_err_t serve_page(httpd_req_t* request)
{
    httpd_resp_set_type(request, "text/html");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(request, page_start, page_end - page_start - 1);
}

esp_err_t open_websocket(httpd_req_t* request)
{
    char origin[64]{};
    if (httpd_req_get_hdr_value_str(request, "Origin", origin, sizeof(origin)) != ESP_OK ||
        std::strcmp(origin, ORIGIN) != 0) {
        httpd_resp_send_err(request, HTTPD_403_FORBIDDEN, "Open the robot's own page");
        return ESP_FAIL;
    }
    portENTER_CRITICAL(&state_lock);
    const bool accepted = session.connect(httpd_req_to_sockfd(request), esp_timer_get_time());
    portEXIT_CRITICAL(&state_lock);
    if (!accepted) {
        httpd_resp_set_status(request, "503 Service Unavailable");
        httpd_resp_sendstr(request, "One control page at a time");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void close_socket(httpd_handle_t, int socket)
{
    portENTER_CRITICAL(&state_lock);
    session.disconnect(socket);
    portEXIT_CRITICAL(&state_lock);
    close(socket);
}

esp_err_t send_text(httpd_req_t* request, const char* text)
{
    httpd_ws_frame_t frame{};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<std::uint8_t*>(const_cast<char*>(text));
    frame.len = std::strlen(text);
    return httpd_ws_send_frame(request, &frame);
}

esp_err_t send_telemetry(httpd_req_t* request)
{
    portENTER_CRITICAL(&state_lock);
    const wireless::Telemetry data = latest;
    portEXIT_CRITICAL(&state_lock);
    const bool valid = data.sample_valid && std::isfinite(data.tilt_rad) &&
                       std::isfinite(data.rate_rad_s);
    // Mode/status are fixed application strings, never browser input.
    char json[512];
    const int length = std::snprintf(json, sizeof(json),
        "{\"tilt_rad\":%.7g,\"rate_rad_s\":%.7g,\"command\":%.7g,"
        "\"sample_valid\":%s,\"motors_ready\":%s,\"mode\":\"%s\","
        "\"status\":\"%s\",\"kp\":%.9g,\"kd\":%.9g}",
        valid ? data.tilt_rad : 0.0f, valid ? data.rate_rad_s : 0.0f,
        std::isfinite(data.command) ? data.command : 0.0f,
        valid ? "true" : "false", data.motors_ready ? "true" : "false",
        data.mode, data.status, data.control.kp, data.control.kd);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(json)) return ESP_FAIL;
    return send_text(request, json);
}

esp_err_t receive_websocket(httpd_req_t* request)
{
    httpd_ws_frame_t frame{};
    ESP_RETURN_ON_ERROR(httpd_ws_recv_frame(request, &frame, 0), TAG, "Read frame size");
    if (frame.type != HTTPD_WS_TYPE_TEXT || !frame.final || frame.len == 0 || frame.len >= 64) {
        return ESP_FAIL;
    }
    std::uint8_t buffer[64]{};
    frame.payload = buffer;
    ESP_RETURN_ON_ERROR(httpd_ws_recv_frame(request, &frame, frame.len), TAG, "Read frame");
    wireless::Request message;
    if (!wireless::parse_request(
            {reinterpret_cast<const char*>(buffer), frame.len}, message)) {
        return send_text(request, "{\"error\":\"Invalid command or gains\"}");
    }
    portENTER_CRITICAL(&state_lock);
    const bool accepted = session.receive(
        httpd_req_to_sockfd(request), message, esp_timer_get_time());
    const bool alive = session.connected(esp_timer_get_time());
    portEXIT_CRITICAL(&state_lock);
    if (!alive) return ESP_FAIL;
    if (!accepted) return send_text(request, "{\"error\":\"Request busy; try again\"}");
    if (message.type == wireless::RequestType::heartbeat) return send_telemetry(request);
    return send_text(request, "{\"queued\":true}");
}
}  // namespace

esp_err_t wireless::init()
{
    // Preserve existing NVS contents on errors; do not erase the whole partition.
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "Initialize storage");
    char password[33]{};
    ESP_RETURN_ON_ERROR(load_password(password), TAG, "Load Wi-Fi password");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "Initialize network");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Create event loop");
    if (esp_netif_create_default_wifi_ap() == nullptr) return ESP_ERR_NO_MEM;

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_config), TAG, "Initialize Wi-Fi");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "Select RAM config");
    wifi_config_t config{};
    std::memcpy(config.ap.ssid, SSID, sizeof(SSID));
    std::memcpy(config.ap.password, password, sizeof(password));
    config.ap.ssid_len = sizeof(SSID) - 1;
    config.ap.channel = 1;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.max_connection = 1;
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "Select access point mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &config), TAG, "Configure access point");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Start access point");

    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.task_priority = 1;
    server_config.stack_size = 6144;
    server_config.max_open_sockets = 4;
    server_config.recv_wait_timeout = 1;
    server_config.send_wait_timeout = 1;
    // Release a dead client's socket so a new control page can connect.
    server_config.keep_alive_enable = true;
    server_config.keep_alive_idle = 2;
    server_config.keep_alive_interval = 1;
    server_config.keep_alive_count = 2;
    server_config.close_fn = close_socket;
    httpd_handle_t server = nullptr;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &server_config), TAG, "Start web server");
    httpd_uri_t page{};
    page.uri = "/";
    page.method = HTTP_GET;
    page.handler = serve_page;
    httpd_uri_t websocket{};
    websocket.uri = "/ws";
    websocket.method = HTTP_GET;
    websocket.handler = receive_websocket;
    websocket.is_websocket = true;
    websocket.ws_pre_handshake_cb = open_websocket;
    esp_err_t err = httpd_register_uri_handler(server, &page);
    if (err == ESP_OK) err = httpd_register_uri_handler(server, &websocket);
    if (err != ESP_OK) {
        httpd_stop(server);
        return err;
    }
    ESP_LOGI(TAG, "Network: %s | Password: %s | Open %s", SSID, password, ORIGIN);
    return ESP_OK;
}

bool wireless::read_request(Request& request)
{
    portENTER_CRITICAL(&state_lock);
    const bool available = session.take(request, esp_timer_get_time());
    portEXIT_CRITICAL(&state_lock);
    return available;
}

bool wireless::connected()
{
    portENTER_CRITICAL(&state_lock);
    const bool alive = session.connected(esp_timer_get_time());
    portEXIT_CRITICAL(&state_lock);
    return alive;
}

void wireless::publish(const Telemetry& telemetry)
{
    portENTER_CRITICAL(&state_lock);
    latest = telemetry;
    portEXIT_CRITICAL(&state_lock);
}
