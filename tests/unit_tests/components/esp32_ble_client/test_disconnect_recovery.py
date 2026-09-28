"""Execute production teardown control flow with a fake clock and ESP-IDF events.

ESP-IDF cannot run on the host. Extract the unchanged method bodies and the two
relevant event cases, compile them against SDK doubles, and inject event loss.
The firmware build separately checks the real headers and complete event handler.
"""

from pathlib import Path
import shutil
import subprocess

import pytest


def _block(source: str, marker: str) -> str:
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def test_disconnect_recovery(tmp_path: Path) -> None:
    """Lost events never falsely release a live peer or disturb another peer."""
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("Native teardown test requires g++")
    root = Path(__file__).resolve().parents[4]
    component = root / "esphome/components/esp32_ble_client"
    source = (component / "ble_client_base.cpp").read_text()
    header = (component / "ble_client_base.h").read_text()
    declarations = "\n".join(
        _block(header, marker)
        for marker in (
            "void set_idle_()",
            "void set_disconnecting_()",
            "bool connected()",
        )
    )
    methods = "\n".join(
        _block(source, marker)
        for marker in (
            "void BLEClientBase::loop()",
            "bool BLEClientBase::recover_disconnect_()",
            "void BLEClientBase::disconnect()",
            "void BLEClientBase::unconditional_disconnect()",
        )
    )
    events = "\n".join(
        _block(source, marker)
        for marker in (
            "case ESP_GATTC_DISCONNECT_EVT:",
            "case ESP_GATTC_CLOSE_EVT:",
        )
    )
    harness = r"""
#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_GATT_CONN_TIMEOUT = 8;
constexpr int ESP_GATT_CONN_TERMINATE_PEER_USER = 19;
constexpr int ESP_GATTC_DISCONNECT_EVT = 1, ESP_GATTC_CLOSE_EVT = 2;
constexpr uint16_t UNSET_CONN_ID = 65535;
constexpr uint32_t DISCONNECTING_TIMEOUT = 10000;
static uint32_t now = 0;
uint32_t millis() { return now; }
static int gap_result = 0, error_logs = 0;
static int close_calls = 0;
int esp_ble_gattc_close(int, uint16_t) { ++close_calls; return ESP_OK; }
static std::vector<uint8_t> disconnected_peers;
int esp_ble_gap_disconnect(uint8_t *peer) {
  disconnected_peers.push_back(peer[0]);
  return gap_result;
}
int esp_ble_gattc_app_register(int) { return 0; }
#define ESP_LOGE(...) (++error_logs)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGI(...) ((void)0)
namespace espbt { enum class ClientState { INIT, IDLE, DISCOVERED, CONNECTING, CONNECTED, ESTABLISHED, DISCONNECTING }; }
namespace esp32_ble {
struct BLE { bool is_active() { return true; } } instance;
static BLE *global_ble = &instance;
}
struct Event {
  struct { uint8_t remote_bda[6]{}; int reason = 0; } disconnect;
  struct { uint16_t conn_id = 7; int reason = 0; } close;
};
class BLEClientBase {
 public:
  espbt::ClientState st = espbt::ClientState::CONNECTED;
  uint32_t disconnecting_started_ = 0;
  uint16_t conn_id_ = 7;
  uint8_t remote_bda_[6]{};
  int app_id = 0, connection_index_ = 0, completions = 0, gattc_if_ = 1;
  bool want_disconnect_ = false;
  char address_str_[18]{};
  bool disconnect_seen_ = false, gap_disconnect_requested_ = false;
  bool disconnect_recovery_failed_ = false, loop_enabled = true;
  explicit BLEClientBase(uint8_t peer) { remote_bda_[0] = peer; }
  espbt::ClientState state() { return st; }
  void set_state(espbt::ClientState value) { st = value; }
  void set_address(uint64_t) {}
  void enable_loop() { loop_enabled = true; }
  void disable_loop() { loop_enabled = false; }
  void mark_failed() { assert(false); }
  void release_services() {}
  void on_disconnect_complete(int) { ++completions; }
  bool check_addr(uint8_t *peer) { return memcmp(peer, remote_bda_, 6) == 0; }
  void log_gattc_warning_(const char *, int) {}
  void log_gattc_lifecycle_event_(const char *) {}
  void log_warning_(const char *) {}
  void log_event_(const char *) {}
  void log_error_(const char *) {}
  void loop();
  bool recover_disconnect_();
  void disconnect();
  void unconditional_disconnect();
  DECLARATIONS
  bool event(int event, Event *param) {
    switch (event) { EVENTS }
    return true;
  }
};
METHODS
using State = espbt::ClientState;
void advance(BLEClientBase &c) { now += 10001; c.loop(); }
void physical_down(BLEClientBase &c) {
  Event e; memcpy(e.disconnect.remote_bda, c.remote_bda_, 6);
  c.event(ESP_GATTC_DISCONNECT_EVT, &e);
}
void close(BLEClientBase &c) { Event e; c.event(ESP_GATTC_CLOSE_EVT, &e); }
int main() {
  // Normal close: no physical disconnect request; late event cannot poison next session.
  BLEClientBase normal(1); normal.disconnect(); close(normal); physical_down(normal);
  assert(normal.st == State::IDLE && normal.completions == 1);
  assert(!normal.disconnect_seen_ && disconnected_peers.empty() && close_calls == 1);
  // Lost CLOSE, but physical disconnect already observed: existing timeout can release.
  BLEClientBase lost_close(2); physical_down(lost_close); advance(lost_close);
  assert(lost_close.st == State::IDLE && lost_close.completions == 1);
  assert(disconnected_peers.empty());
  // Hung close: force only this peer, hold slot across premature CLOSE.
  BLEClientBase hung(3), other(4); hung.set_disconnecting_(); advance(hung);
  assert(hung.st == State::DISCONNECTING && hung.conn_id_ == 7);
  assert(disconnected_peers == std::vector<uint8_t>{3});
  close(hung); assert(hung.completions == 0 && hung.conn_id_ == 7);
  assert(other.st == State::CONNECTED && other.completions == 0);
  physical_down(hung); close(hung);
  assert(hung.st == State::IDLE && hung.completions == 1);
  assert(!hung.gap_disconnect_requested_ && !hung.disconnect_seen_);
  // Lost CLOSE after forced physical disconnect still releases on the next timeout.
  BLEClientBase forced(5); forced.set_disconnecting_(); advance(forced);
  physical_down(forced); advance(forced);
  assert(forced.st == State::IDLE && forced.completions == 1);
  // Rejected SDK call / missing events: never lie about capacity or spam retries.
  gap_result = -1;
  BLEClientBase failed(6); failed.set_disconnecting_(); advance(failed);
  auto count = disconnected_peers.size(); int logs = error_logs;
  advance(failed); advance(failed);
  assert(failed.st == State::CONNECTED && failed.completions == 0);
  assert(disconnected_peers.size() == count && error_logs == logs + 1);
  assert(!failed.connected());  // No GATT operations on the quarantined link.
  int before = close_calls;
  failed.disconnect(); failed.disconnect();
  assert(close_calls == before);  // Lost API owner must not trigger retry storms.
  Event wrong; wrong.disconnect.remote_bda[0] = 99; wrong.close.conn_id = 99;
  failed.event(ESP_GATTC_DISCONNECT_EVT, &wrong); failed.event(ESP_GATTC_CLOSE_EVT, &wrong);
  assert(failed.completions == 0);
  // Delayed genuine events can release the quarantined slot.
  physical_down(failed); close(failed);
  assert(failed.st == State::IDLE && failed.completions == 1);
  // uint32 millis rollover does not defeat the timeout.
  gap_result = 0; now = UINT32_MAX - 5000;
  BLEClientBase rollover(7); rollover.set_disconnecting_(); advance(rollover);
  assert(disconnected_peers.back() == 7 && rollover.gap_disconnect_requested_);
  std::cout << "teardown fault-injection scenarios passed\n";
}
"""
    harness = harness.replace("DECLARATIONS", declarations)
    harness = harness.replace("EVENTS", events).replace("METHODS", methods)
    cpp = tmp_path / "disconnect_recovery.cpp"
    binary = tmp_path / "disconnect_recovery"
    cpp.write_text(harness)
    subprocess.run(
        [
            compiler,
            "-std=c++20",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(cpp),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(binary)], check=True, capture_output=True, text=True)
