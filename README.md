# Self balancing robot

ESP32-C6 firmware using an ICM-20948 accelerometer and gyroscope over SPI.

## ESP-IDF workflow in VS Code

Run the commands below from the Command Palette (`Cmd+Shift+P` on macOS).

### Initial setup

- Open this repository's root folder in VS Code.
- For a new project only: `ESP-IDF: New Project`.
- `ESP-IDF: Set Espressif Device Target` → `esp32c6`.
- `ESP-IDF: Select Flash Method` → `UART` for the current setup.
- `ESP-IDF: SDK Configuration Editor (Menuconfig)` → change project settings as needed.

### Each time you plug in the board

1. Connect the ESP32-C6 with a USB data cable.
2. Run `ESP-IDF: Select Port to Use (COM, tty, usbserial)` again and select the
   connected board. The `/dev/…usbmodem…` port name can change after reconnecting.
   See the [ESP-IDF connection guide](https://docs.espressif.com/projects/vscode-esp-idf-extension/en/latest/connectdevice.html).

### Build and upload changes

- **Build only:** `ESP-IDF: Build Your Project`.
- **Upload for Teleplot:** build, then run `ESP-IDF: Flash Your Project`.
- **Upload and view text output:** `ESP-IDF: Build, Flash and Start a Monitor on Your Device`.

Disconnect Teleplot or stop the serial monitor with `Ctrl+]` before flashing so
the serial port is available.

### Plot tilt with Teleplot

1. Stop the ESP-IDF serial monitor with `Ctrl+]` if it is running.
2. Run `teleplot: Start teleplot session`, select the board's current serial port,
   and connect at **115200 baud**. The plotted signal is `tilt_rad`.

The serial monitor and Teleplot cannot use the same port simultaneously. After
reconnecting the board, reselect its port in Teleplot as well.


## Tilt estimation

`main/tilt_estimator.cpp` implements `update_complementary_filter(...)`. The gyro
predicts angle changes, while the accelerometer slowly corrects drift. The filter
uses radians, radians/second, and the measured elapsed time between samples.
`main/main.cpp` samples at a nominal 100 Hz and prints `>tilt_rad:...` for Teleplot.

Assumed mounting: sensor X forward, Y along the wheel axle toward the left, and Z
up. Tilt is `atan2(-accel_x, accel_z)`, paired with `gyro_y`; positive tilt is nose
down. Hold the robot stationary at startup to initialize from the accelerometer.

Tune `TILT_FILTER_CONFIG` in `main/main.cpp`: the correction time constant starts
at **1.0 second**. Larger values resist brief acceleration disturbances but correct
drift more slowly; smaller values correct faster. This does not delay the gyro's
immediate response. Sustained linear acceleration can still bias the estimate.
Gyro bias defaults to zero; supply a bias calibrated while stationary if available.