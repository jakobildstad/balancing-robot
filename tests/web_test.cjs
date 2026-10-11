// Exercise the embedded page without a browser dependency or robot hardware.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, '../main/web/index.html'), 'utf8');
const elements = Object.fromEntries([...html.matchAll(/id="([^"]+)"/g)].map(([, id]) =>
  [id, { textContent: '', value: '0', disabled: true }]));
const buttons = [...html.matchAll(/data-command="([^"]+)"/g)].map(([, command]) =>
  ({ dataset: { command }, disabled: true }));
const listeners = {};
const sockets = [];
let now = 0;
let tick = null;
class WebSocket {
  static OPEN = 1;
  constructor() { this.readyState = 0; this.sent = []; sockets.push(this); }
  send(text) { this.sent.push(text); }
  close() { this.readyState = 3; this.onclose?.(); }
  open() { this.readyState = WebSocket.OPEN; this.onopen(); }
  receive(data) { this.onmessage({ data: JSON.stringify(data) }); }
}
const document = {
  hidden: false,
  getElementById: id => elements[id],
  querySelectorAll: () => buttons,
  addEventListener: (name, callback) => { listeners[name] = callback; },
};
vm.runInNewContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], {
  document,
  window: { addEventListener: (name, callback) => { listeners[name] = callback; } },
  location: { host: '192.168.4.1' }, WebSocket,
  Date: { now: () => now },
  setInterval: callback => { tick = callback; return 1; },
  clearInterval: () => { tick = null; },
});

const socket = sockets[0];
socket.open();
assert.deepEqual(socket.sent, ['heartbeat']);
assert.equal(elements.stop.disabled, false);
const state = { sample_valid: true, motors_ready: true, mode: 'idle',
  tilt_rad: 0.1, rate_rad_s: -0.2, command: 0, kp: 0, kd: 0, status: 'Ready' };
socket.receive(state);
assert.equal(elements.tilt.textContent, '0.100');
assert.equal(elements.connection.textContent, 'Connected');
assert(buttons.every(button => !button.disabled));

elements.kp.value = '2';
elements.kd.value = '0.1';
elements.kp.oninput();
socket.receive(state);
assert.equal(elements.kp.value, '2', 'Telemetry must not overwrite edits');
elements['gains-form'].onsubmit({ preventDefault() {} });
assert.equal(socket.sent.at(-1), 'gains 2 0.1');
for (const button of buttons) {
  button.onclick();
  assert.equal(socket.sent.at(-1), button.dataset.command);
}
socket.receive({ ...state, mode: 'balance' });
assert(buttons.every(button => button.disabled));
assert.equal(elements['gains-fields'].disabled, true);
elements.stop.onclick();
assert.equal(socket.sent.at(-1), 'stop');
socket.receive({ ...state, sample_valid: false });
assert.equal(elements.tilt.textContent, '—');
assert.equal(buttons.find(button => button.dataset.command === 'balance').disabled, true);
socket.receive({ error: 'Request busy; try again' });
assert.equal(elements.notice.textContent, 'Request busy; try again');

now = 1600;
tick();
assert.equal(socket.readyState, 3, 'Missing replies must close the connection');
assert.equal(elements.stop.disabled, true);
assert(buttons.every(button => button.disabled));
elements.connect.onclick();
const reconnected = sockets[1];
reconnected.open();
assert.deepEqual(reconnected.sent, ['heartbeat'], 'Reconnect must not replay commands');
reconnected.receive(state);
document.hidden = true;
listeners.visibilitychange();
assert.equal(reconnected.sent.at(-1), 'stop');
assert.equal(reconnected.readyState, 3);
assert.equal(tick, null, 'Hidden page must stop sending heartbeats');
console.log('Web controls, telemetry, timeout and reconnect checks passed.');
