/*
  nRF52840 mocopi BLE Central -> SlimeVR USB HID receiver

  Build:
  - Serial disabled
  - USB HID enabled
  - 6 tracker probe-based auto MAC discovery version
  - Tuned for 6 concurrent central links
*/

#include <Arduino.h>
#include <bluefruit.h>
#include "arduino/hid/Adafruit_USBD_HID.h"

// ============================================================
// User configuration
// ============================================================

static constexpr uint8_t TRACKER_COUNT = 6;

// MAC addresses are learned automatically from BLE advertisements.
// Tracker IDs are fixed by sorting discovered mocopi BLE MAC addresses, not by connection order.
// This keeps the same mocopi unit on the same SlimeVR tracker ID across boot cycles
// as long as the same set of mocopi devices is discovered.
static constexpr bool USE_AUTO_MAC_DISCOVERY = true;
static constexpr bool AUTO_FIXED_ID_BY_MAC_SORT = true;
static constexpr uint32_t AUTO_DISCOVERY_COLLECT_MS = 3500;

// Keep tracker IDs stable: do not finalize the auto-discovery ID map from a partial set.
// If fewer than TRACKER_COUNT mocopi units are discovered during the first window,
// keep scanning and only start connecting after all expected MAC addresses are known.
static constexpr bool AUTO_DISCOVERY_REQUIRE_FULL_SET = true;

// During initial bring-up, do not advertise trackers as offline to the host until all
// expected trackers have reached live streaming at least once. This prevents the host
// from seeing repeated connect/disconnect churn while the central links are being built.
static constexpr bool HOLD_HOST_ONLINE_UNTIL_ALL_INITIAL_STREAMING = true;

// After a tracker has streamed once, tolerate brief BLE reconnect churn by keeping
// host status online for this period. Power-off still becomes offline after this grace.
static constexpr uint32_t HOST_RECONNECT_GRACE_MS = 8000;

// Reconnection guard. Do not hammer a tracker immediately after disconnect or
// after a failed GATT/start sequence. Rapid retry loops are a common cause of
// one mocopi staying half-connected on nRF52840 central builds.
static constexpr uint32_t RECONNECT_FIRST_BACKOFF_MS = 1200;
static constexpr uint32_t RECONNECT_MAX_BACKOFF_MS = 12000;
static constexpr uint32_t RECONNECT_AFTER_DISCONNECT_BACKOFF_MS = 900;
static constexpr uint8_t RECONNECT_FAIL_COUNT_MAX = 6;

// Battery GATT is optional and relatively disruptive with 6 concurrent central
// links. Only run it after all trackers are streaming and the BLE topology has
// been stable for a while.
static constexpr uint32_t BATTERY_SETUP_AFTER_TOPOLOGY_STABLE_MS = 12000;
static constexpr uint32_t BATTERY_READ_AFTER_TOPOLOGY_STABLE_MS = 8000;

static const char* MOCOPI_NAME_PREFIX = "QM-SS1";

static constexpr uint16_t SLIMEVR_HID_RECEIVER_VID = 0x1209;
static constexpr uint16_t SLIMEVR_HID_RECEIVER_PID = 0x7690;

static constexpr uint32_t HID_GLOBAL_REPORT_HZ = 360;
static constexpr uint32_t HID_SEND_INTERVAL_US = 1000000UL / HID_GLOBAL_REPORT_HZ;

static constexpr uint32_t SENSOR_STALE_TIMEOUT_MS = 1000;
static constexpr uint32_t BATTERY_READ_INTERVAL_MS = 60000;
static constexpr uint32_t BATTERY_RETRY_INTERVAL_MS = 5000;
static constexpr uint32_t BATTERY_SETUP_STEP_INTERVAL_MS = 800;
static constexpr uint32_t USB_MOUNT_WAIT_MS = 300;
static constexpr uint16_t SCAN_INTERVAL_UNITS = 160;  // 100ms
static constexpr uint16_t SCAN_WINDOW_UNITS = 160;    // 100ms
static constexpr uint8_t BATTERY_SETUP_MAX_ATTEMPTS = 30;
static constexpr uint32_t CONNECTING_STALE_TIMEOUT_MS = 15000;

// When a tracker is powered off, keep sending explicit HID offline status packets
// for this period. Do not send quaternion packets while offline/stale; otherwise
// SlimeVR may treat the tracker as still receiving live sensor data.
static constexpr uint32_t DISCONNECT_STATUS_NOTIFY_MS = 5000;

// If mocopi advertisements do not expose the name or service UUID, connect first and
// validate by GATT discovery. Non-mocopi devices are rejected after service discovery fails.
static constexpr bool PROBE_UNKNOWN_CONNECTABLE_ADVERTISEMENTS = true;
static constexpr uint8_t REJECTED_ADDR_COUNT = 16;

// Battery Service discovery can destabilize 6 concurrent BLE central links on nRF52840.
// Keep it disabled unless you specifically need battery reporting.
static constexpr bool ENABLE_BATTERY_SERVICE = true;

// Serial is fully disabled in this build.
static constexpr bool ENABLE_SERIAL_DEBUG = false;

// XIAO nRF52840 status LED behavior.
// - USB mounted/powered: LED solid ON
// - One or more trackers connected: slow blink
// - USB not mounted/powered: LED OFF
static constexpr uint32_t STATUS_LED_SLOW_BLINK_MS = 700;
static constexpr bool STATUS_LED_ACTIVE_LOW = true;

// Acceleration reporting. SlimeVR HID acceleration fields are sent as signed Q7
// values representing m/s^2. mocopi notification acceleration is treated as G.
// If your captured mocopi data already reads about 9.8 while stationary, set
// MOCOPI_ACCEL_RAW_UNIT_IS_G to false.
static constexpr bool MOCOPI_ACCEL_RAW_UNIT_IS_G = false;
static constexpr float STANDARD_GRAVITY_MS2 = 9.80665f;
static constexpr float ACCEL_REPORT_MAX_ABS_MS2 = 78.4532f;  // +/-8G
static constexpr bool ENABLE_ACCEL_LOW_PASS_FOR_HID = false;
static constexpr float ACCEL_LPF_ALPHA = 0.35f;

// GATT discovery stabilization.
static constexpr uint16_t GATT_CONNECT_SETTLE_DELAY_MS = 800;
static constexpr uint8_t GATT_DISCOVER_RETRY_COUNT = 5;
static constexpr uint16_t GATT_DISCOVER_RETRY_DELAY_MS = 400;

// ============================================================
// mocopi BLE UUIDs
// ============================================================

BLEUuid mocopiServiceUuid("91a7608d-4456-479d-b9b1-4706e8711cf8");
BLEUuid mocopiDataUuid("25047e64-657c-4856-afcf-e315048a965b");

// 128-bit UUID in BLE advertising byte order, i.e. little-endian.
static const uint8_t MOCOPI_SERVICE_UUID128_ADV_LE[16] = {
  0xf8, 0x1c, 0x71, 0xe8,
  0x06, 0x47,
  0xb1, 0xb9,
  0x9d, 0x47,
  0x56, 0x44, 0x8d, 0x60, 0xa7, 0x91
};

BLEUuid mocopiCmdUuid("0000ff01-0000-1000-8000-00805f9b34fb");
BLEUuid mocopiCommandServiceUuid("0000ff00-0000-1000-8000-00805f9b34fb");
BLEUuid mocopiFf03Uuid("0000ff03-0000-1000-8000-00805f9b34fb");

// Standard BLE Battery Service.
// This is optional and must never block mocopi connection/stream startup.
BLEUuid batteryServiceUuid(0x180F);
BLEUuid batteryLevelUuid(0x2A19);

static const uint8_t MOCOPI_STREAM_START_CMD[] = {
  0x7e, 0x03, 0x18, 0xd6, 0x01, 0x00, 0x00
};

static const uint8_t MOCOPI_CMD_SET_RTC_1[] = {
  0x7e, 0x0a, 0x18, 0x1e, 0x04, 0xd9, 0x9b, 0xc6,
  0x87, 0x01, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t MOCOPI_CMD_GET_RTC[] = {
  0x7e, 0x02, 0x18, 0x1f, 0x02, 0x18, 0x00, 0x00
};

static const uint8_t MOCOPI_CMD_SET_RTC_2[] = {
  0x7e, 0x0a, 0x18, 0x1e, 0x19, 0xdb, 0x9b, 0xc6,
  0x87, 0x01, 0x00, 0x00, 0x00, 0x00
};

// ============================================================
// SlimeVR HID protocol constants
// ============================================================

static constexpr uint8_t HID_PACKET_SIZE = 16;
static constexpr uint8_t HID_REPORT_SIZE = 64;
static constexpr uint8_t HID_PACKETS_PER_REPORT = HID_REPORT_SIZE / HID_PACKET_SIZE;

static constexpr uint8_t SLIMEVR_IMU_TYPE_MOCOPI_COMPAT = 8;
static constexpr uint8_t SLIMEVR_MAG_NOT_SUPPORTED = 0;
static constexpr uint8_t SLIMEVR_BOARD_CUSTOM = 4;
static constexpr uint8_t SLIMEVR_MCU_NRF52 = 7;

static constexpr uint8_t BATTERY_PERCENT_UNKNOWN = 128;
static constexpr uint8_t BATTERY_VOLTAGE_3V70_ENCODED = 125;
static constexpr uint8_t TEMPERATURE_UNKNOWN = 0;
static constexpr uint8_t RSSI_FAKE = 1;

static constexpr uint16_t FW_DATE_ENCODED =
  (uint16_t)(((2026 - 2020) & 0x7F) << 9) |
  (uint16_t)((5 & 0x0F) << 5) |
  (uint16_t)(9 & 0x1F);

static constexpr uint8_t FW_MAJOR = 1;
static constexpr uint8_t FW_MINOR = 0;
static constexpr uint8_t FW_PATCH = 0;

// ============================================================
// USB HID descriptor
// ============================================================

uint8_t const report_descriptor[] = {
  0x05, 0x01,
  0x09, 0x00,
  0xA1, 0x01,
  0x09, 0x00,
  0x15, 0x00,
  0x26, 0xFF, 0x00,
  0x75, 0x08,
  0x95, 0x40,
  0x81, 0x02,
  0xC0
};

Adafruit_USBD_HID usb_hid;

// ============================================================
// Data model
// ============================================================

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct Accel {
  float x;
  float y;
  float z;
};

struct TrackerState {
  uint8_t id;
  uint8_t targetAddr[6];
  bool targetAddrValid;

  uint16_t connHandle;
  bool connected;
  bool connecting;
  bool streamStarted;
  bool notifyEnabled;
  bool hasFreshSample;
  bool hostRegistered;
  uint32_t offlineStatusUntilMs;
  bool everStreamed;
  uint32_t hostOnlineUntilMs;
  uint32_t reconnectNotBeforeMs;
  uint32_t lastDisconnectMs;
  uint8_t reconnectFailCount;
  uint8_t batterySuspiciousReadCount;

  BLEClientService service;
  BLEClientService commandService;
  BLEClientService batteryService;
  BLEClientCharacteristic dataChar;
  BLEClientCharacteristic cmdChar;
  BLEClientCharacteristic ff03Char;
  BLEClientCharacteristic batteryLevelChar;

  Quaternion quat;
  Accel accel;
  uint32_t lastSampleMs;

  uint8_t batteryPercent;
  bool batteryValid;
  bool batteryDiscovered;
  bool batterySetupDone;
  uint8_t batterySetupAttempts;
  uint32_t lastBatteryReadMs;
  uint32_t connectingStartedMs;

  uint32_t packetsReceivedCounter;
  uint32_t packetsLostCounter;

  uint64_t slimeAddress48;

  uint32_t notifyCount;
  uint32_t hidSentCount;
  uint32_t hidFailCount;

  TrackerState()
    : id(0),
      targetAddr{0},
      targetAddrValid(false),
      connHandle(BLE_CONN_HANDLE_INVALID),
      connected(false),
      connecting(false),
      streamStarted(false),
      notifyEnabled(false),
      hasFreshSample(false),
      hostRegistered(false),
      offlineStatusUntilMs(0),
      everStreamed(false),
      hostOnlineUntilMs(0),
      reconnectNotBeforeMs(0),
      lastDisconnectMs(0),
      reconnectFailCount(0),
      batterySuspiciousReadCount(0),
      service(mocopiServiceUuid),
      commandService(mocopiCommandServiceUuid),
      batteryService(batteryServiceUuid),
      dataChar(mocopiDataUuid),
      cmdChar(mocopiCmdUuid),
      ff03Char(mocopiFf03Uuid),
      batteryLevelChar(batteryLevelUuid),
      quat{1.0f, 0.0f, 0.0f, 0.0f},
      accel{0.0f, 0.0f, 0.0f},
      lastSampleMs(0),
      batteryPercent(BATTERY_PERCENT_UNKNOWN),
      batteryValid(false),
      batteryDiscovered(false),
      batterySetupDone(false),
      batterySetupAttempts(0),
      lastBatteryReadMs(0),
      connectingStartedMs(0),
      packetsReceivedCounter(0),
      packetsLostCounter(0),
      slimeAddress48(0),
      notifyCount(0),
      hidSentCount(0),
      hidFailCount(0) {}
};

TrackerState trackers[TRACKER_COUNT];

static uint8_t nextHidTrackerIndex = 0;
static uint32_t prevHidSendUs = 0;
static uint32_t lastScanRestartMs = 0;
static uint8_t nextBatterySetupIndex = 0;
static uint32_t lastBatterySetupAttemptMs = 0;
static uint8_t rejectedAddrs[REJECTED_ADDR_COUNT][6];
static uint8_t rejectedAddrCount = 0;

// Auto fixed-ID discovery state. Addresses are stored in conventional
// big-endian MAC order, sorted, then copied into trackers[0..N-1].
static uint8_t autoDiscoveredAddrs[TRACKER_COUNT][6];
static uint8_t autoDiscoveredAddrCount = 0;
static bool autoDiscoveryMapFinalized = false;
static uint32_t autoDiscoveryStartedMs = 0;
static bool initialAllTrackersStreamed = false;
static uint32_t lastBleTopologyChangeMs = 0;

// ============================================================
// Debug no-op functions
// ============================================================

void debug_print(const char* s) {
  (void)s;
}

void debug_println(const char* s) {
  (void)s;
}

void print_hex_buffer(const char* label, const uint8_t* data, uint16_t len) {
  (void)label;
  (void)data;
  (void)len;
}

void debug_read_characteristic(const char* label, BLEClientCharacteristic& chr) {
  (void)label;

  uint8_t buffer[64];
  memset(buffer, 0, sizeof(buffer));
  chr.read(buffer, sizeof(buffer));
}

// ============================================================
// GATT helpers
// ============================================================

bool discover_service_with_retry(
  const char* label,
  BLEClientService& service,
  uint16_t connHandle,
  uint8_t attempts = GATT_DISCOVER_RETRY_COUNT,
  uint16_t delayMs = GATT_DISCOVER_RETRY_DELAY_MS
) {
  (void)label;

  for (uint8_t attempt = 1; attempt <= attempts; attempt++) {
    if (service.discover(connHandle)) {
      return true;
    }

    delay(delayMs);
  }

  return false;
}

bool discover_characteristic_with_retry(
  const char* label,
  BLEClientCharacteristic& characteristic,
  uint8_t attempts = GATT_DISCOVER_RETRY_COUNT,
  uint16_t delayMs = GATT_DISCOVER_RETRY_DELAY_MS
) {
  (void)label;

  for (uint8_t attempt = 1; attempt <= attempts; attempt++) {
    if (characteristic.discover()) {
      return true;
    }

    delay(delayMs);
  }

  return false;
}

bool mocopi_write_handle(uint16_t connHandle, uint16_t handle, const uint8_t* data, uint16_t len) {
  ble_gattc_write_params_t writeParams;
  memset(&writeParams, 0, sizeof(writeParams));

  writeParams.write_op = BLE_GATT_OP_WRITE_REQ;
  writeParams.flags = 0;
  writeParams.handle = handle;
  writeParams.offset = 0;
  writeParams.len = len;
  writeParams.p_value = (uint8_t*)data;

  uint32_t err = sd_ble_gattc_write(connHandle, &writeParams);

  return err == NRF_SUCCESS;
}

bool mocopi_send_pre_start_sequence_by_handle(uint16_t connHandle) {
  static const uint16_t MOCOPI_COMMAND_VALUE_HANDLE = 0x0020;

  if (!mocopi_write_handle(
        connHandle,
        MOCOPI_COMMAND_VALUE_HANDLE,
        MOCOPI_CMD_SET_RTC_1,
        sizeof(MOCOPI_CMD_SET_RTC_1)
      )) {
    return false;
  }
  delay(200);

  if (!mocopi_write_handle(
        connHandle,
        MOCOPI_COMMAND_VALUE_HANDLE,
        MOCOPI_CMD_GET_RTC,
        sizeof(MOCOPI_CMD_GET_RTC)
      )) {
    return false;
  }
  delay(200);

  if (!mocopi_write_handle(
        connHandle,
        MOCOPI_COMMAND_VALUE_HANDLE,
        MOCOPI_CMD_SET_RTC_2,
        sizeof(MOCOPI_CMD_SET_RTC_2)
      )) {
    return false;
  }
  delay(200);

  if (!mocopi_write_handle(
        connHandle,
        MOCOPI_COMMAND_VALUE_HANDLE,
        MOCOPI_CMD_GET_RTC,
        sizeof(MOCOPI_CMD_GET_RTC)
      )) {
    return false;
  }
  delay(200);

  return true;
}

bool mocopi_write_start_command_by_handle(uint16_t connHandle) {
  static const uint16_t MOCOPI_COMMAND_VALUE_HANDLE = 0x0020;

  return mocopi_write_handle(
    connHandle,
    MOCOPI_COMMAND_VALUE_HANDLE,
    MOCOPI_STREAM_START_CMD,
    sizeof(MOCOPI_STREAM_START_CMD)
  );
}

// ============================================================
// Utility
// ============================================================

void write_u16_le(uint16_t v, uint8_t* buf) {
  buf[0] = (uint8_t)(v & 0xFF);
  buf[1] = (uint8_t)((v >> 8) & 0xFF);
}

void write_i16_le(int16_t v, uint8_t* buf) {
  uint16_t u = (uint16_t)v;
  buf[0] = (uint8_t)(u & 0xFF);
  buf[1] = (uint8_t)((u >> 8) & 0xFF);
}

void write_u64_le(uint64_t v, uint8_t* buf) {
  for (uint8_t i = 0; i < 8; i++) {
    buf[i] = (uint8_t)((v >> (8 * i)) & 0xFF);
  }
}

float clamp_float(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int16_t float_to_q15(float v) {
  if (isnan(v) || isinf(v)) return 0;
  v = clamp_float(v, -1.0f, 0.999969f);
  if (v >= 0.0f) return (int16_t)lroundf(v * 32767.0f);
  return (int16_t)lroundf(v * 32768.0f);
}

int16_t accel_ms2_to_q7(float v) {
  if (isnan(v) || isinf(v)) return 0;
  float scaled = v * 128.0f;
  if (scaled > 32767.0f) scaled = 32767.0f;
  if (scaled < -32768.0f) scaled = -32768.0f;
  return (int16_t)lroundf(scaled);
}

uint8_t encode_status_counter_nonzero(uint32_t count) {
  if (count == 0) return 0;

  uint8_t encoded = (uint8_t)(count % 255UL);
  if (encoded == 0) return 255;
  return encoded;
}

uint8_t hex_value(char c) {
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'a' && c <= 'f') return (uint8_t)(10 + c - 'a');
  if (c >= 'A' && c <= 'F') return (uint8_t)(10 + c - 'A');
  return 0xFF;
}

bool parse_mac(const char* text, uint8_t out[6]) {
  if (!text) return false;

  uint8_t index = 0;
  uint8_t high = 0;
  bool waitingLow = false;

  for (const char* p = text; *p != '\0'; ++p) {
    if (*p == ':' || *p == '-' || *p == ' ') continue;

    uint8_t v = hex_value(*p);
    if (v == 0xFF) return false;

    if (!waitingLow) {
      high = v;
      waitingLow = true;
    } else {
      if (index >= 6) return false;
      out[index++] = (uint8_t)((high << 4) | v);
      waitingLow = false;
    }
  }

  return index == 6 && !waitingLow;
}

bool addr_matches_big_or_little(const ble_gap_addr_t& advAddr, const uint8_t targetBig[6]) {
  bool direct = true;
  bool reversed = true;

  for (uint8_t i = 0; i < 6; i++) {
    if (advAddr.addr[i] != targetBig[i]) direct = false;
    if (advAddr.addr[i] != targetBig[5 - i]) reversed = false;
  }

  return direct || reversed;
}

void peer_addr_to_big_endian(const ble_gap_addr_t& peerAddr, uint8_t outBig[6]) {
  // SoftDevice stores BLE addresses little-endian in ble_gap_addr_t::addr.
  // Keep targetAddr/slimeAddress in conventional big-endian MAC order.
  for (uint8_t i = 0; i < 6; i++) {
    outBig[i] = peerAddr.addr[5 - i];
  }
}

uint64_t addr_to_u48(const uint8_t addrBig[6]) {
  uint64_t v = 0;
  for (uint8_t i = 0; i < 6; i++) {
    v = (v << 8) | addrBig[i];
  }
  return v & 0x0000FFFFFFFFFFFFULL;
}

bool get_adv_name(const uint8_t* advData, uint16_t advLen, char* out, size_t outSize) {
  if (!out || outSize == 0) return false;
  memset(out, 0, outSize);

  uint8_t found = Bluefruit.Scanner.parseReportByType(
    advData,
    advLen,
    BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
    (uint8_t*)out,
    outSize - 1
  );

  if (found == 0) {
    found = Bluefruit.Scanner.parseReportByType(
      advData,
      advLen,
      BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
      (uint8_t*)out,
      outSize - 1
    );
  }

  return found > 0;
}

bool name_starts_with(const uint8_t* advData, uint16_t advLen, const char* prefix) {
  if (!prefix || prefix[0] == '\0') return true;

  char nameBuffer[32];
  if (!get_adv_name(advData, advLen, nameBuffer, sizeof(nameBuffer))) {
    return false;
  }

  return strncmp(nameBuffer, prefix, strlen(prefix)) == 0;
}

bool adv_contains_uuid128(const uint8_t* advData, uint16_t advLen, const uint8_t uuidLe[16]) {
  if (!advData || !uuidLe) return false;

  uint16_t offset = 0;
  while (offset < advLen) {
    uint8_t fieldLen = advData[offset];
    if (fieldLen == 0) break;

    uint16_t fieldEnd = (uint16_t)(offset + 1 + fieldLen);
    if (fieldEnd > advLen) break;

    uint8_t type = advData[offset + 1];
    if (type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE ||
        type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE) {
      uint16_t valueOffset = (uint16_t)(offset + 2);
      uint16_t valueLen = (uint16_t)(fieldLen - 1);

      for (uint16_t p = valueOffset; (uint16_t)(p + 16) <= (uint16_t)(valueOffset + valueLen); p += 16) {
        if (memcmp(&advData[p], uuidLe, 16) == 0) {
          return true;
        }
      }
    }

    offset = fieldEnd;
  }

  return false;
}

bool adv_looks_like_mocopi(const ble_gap_evt_adv_report_t* report) {
  if (!report || !report->data.p_data || report->data.len == 0) return false;

  if (name_starts_with(report->data.p_data, report->data.len, MOCOPI_NAME_PREFIX)) {
    return true;
  }

  if (adv_contains_uuid128(report->data.p_data, report->data.len, MOCOPI_SERVICE_UUID128_ADV_LE)) {
    return true;
  }

  return false;
}

int find_tracker_by_conn(uint16_t connHandle) {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (trackers[i].connected && trackers[i].connHandle == connHandle) return i;
  }
  return -1;
}

int find_tracker_by_data_char(BLEClientCharacteristic* chr) {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (&trackers[i].dataChar == chr) return i;
  }
  return -1;
}

int find_tracker_by_battery_char(BLEClientCharacteristic* chr) {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (&trackers[i].batteryLevelChar == chr) return i;
  }
  return -1;
}

bool peer_addr_equal(const ble_gap_addr_t& a, const ble_gap_addr_t& b) {
  for (uint8_t i = 0; i < 6; i++) {
    if (a.addr[i] != b.addr[i]) return false;
  }
  return true;
}

bool peer_addr_matches_big(const ble_gap_addr_t& peerAddr, const uint8_t addrBig[6]) {
  for (uint8_t i = 0; i < 6; i++) {
    if (peerAddr.addr[i] != addrBig[5 - i]) return false;
  }
  return true;
}

bool is_rejected_peer_addr(const ble_gap_addr_t& peerAddr) {
  for (uint8_t i = 0; i < rejectedAddrCount; i++) {
    if (peer_addr_matches_big(peerAddr, rejectedAddrs[i])) {
      return true;
    }
  }
  return false;
}

void remember_rejected_peer_addr(const ble_gap_addr_t& peerAddr) {
  if (is_rejected_peer_addr(peerAddr)) return;

  uint8_t slot = rejectedAddrCount;
  if (slot >= REJECTED_ADDR_COUNT) {
    slot = 0;
  } else {
    rejectedAddrCount++;
  }

  peer_addr_to_big_endian(peerAddr, rejectedAddrs[slot]);
}

bool mac_big_equal(const uint8_t a[6], const uint8_t b[6]) {
  for (uint8_t i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

int compare_mac_big(const uint8_t a[6], const uint8_t b[6]) {
  for (uint8_t i = 0; i < 6; i++) {
    if (a[i] < b[i]) return -1;
    if (a[i] > b[i]) return 1;
  }
  return 0;
}

bool auto_discovery_addr_already_seen(const uint8_t addrBig[6]) {
  for (uint8_t i = 0; i < autoDiscoveredAddrCount; i++) {
    if (mac_big_equal(autoDiscoveredAddrs[i], addrBig)) {
      return true;
    }
  }
  return false;
}

bool remember_auto_discovered_peer_addr(const ble_gap_addr_t& peerAddr) {
  if (autoDiscoveryMapFinalized) return false;
  if (autoDiscoveredAddrCount >= TRACKER_COUNT) return false;

  uint8_t addrBig[6];
  peer_addr_to_big_endian(peerAddr, addrBig);

  if (auto_discovery_addr_already_seen(addrBig)) {
    return false;
  }

  memcpy(autoDiscoveredAddrs[autoDiscoveredAddrCount], addrBig, sizeof(autoDiscoveredAddrs[autoDiscoveredAddrCount]));
  autoDiscoveredAddrCount++;
  return true;
}

void sort_auto_discovered_addrs() {
  for (uint8_t i = 0; i < autoDiscoveredAddrCount; i++) {
    for (uint8_t j = (uint8_t)(i + 1); j < autoDiscoveredAddrCount; j++) {
      if (compare_mac_big(autoDiscoveredAddrs[j], autoDiscoveredAddrs[i]) < 0) {
        uint8_t tmp[6];
        memcpy(tmp, autoDiscoveredAddrs[i], sizeof(tmp));
        memcpy(autoDiscoveredAddrs[i], autoDiscoveredAddrs[j], sizeof(tmp));
        memcpy(autoDiscoveredAddrs[j], tmp, sizeof(tmp));
      }
    }
  }
}

void finalize_auto_discovery_map() {
  if (autoDiscoveryMapFinalized) return;

  if (AUTO_FIXED_ID_BY_MAC_SORT && AUTO_DISCOVERY_REQUIRE_FULL_SET && autoDiscoveredAddrCount < TRACKER_COUNT) {
    return;
  }

  sort_auto_discovered_addrs();

  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    TrackerState& t = trackers[i];

    if (i < autoDiscoveredAddrCount) {
      memcpy(t.targetAddr, autoDiscoveredAddrs[i], sizeof(t.targetAddr));
      t.targetAddrValid = true;
      t.slimeAddress48 = addr_to_u48(t.targetAddr);
    }
  }

  autoDiscoveryMapFinalized = true;
}

bool auto_discovery_collect_window_elapsed() {
  if (autoDiscoveryStartedMs == 0) return false;
  return (uint32_t)(millis() - autoDiscoveryStartedMs) >= AUTO_DISCOVERY_COLLECT_MS;
}

int find_tracker_by_known_addr(const ble_gap_addr_t& peerAddr) {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    TrackerState& t = trackers[i];
    if (!t.targetAddrValid) continue;
    if (addr_matches_big_or_little(peerAddr, t.targetAddr)) return i;
  }

  return -1;
}

uint32_t reconnect_backoff_for_fail_count(uint8_t failCount) {
  uint32_t backoff = RECONNECT_FIRST_BACKOFF_MS;

  for (uint8_t i = 1; i < failCount && backoff < RECONNECT_MAX_BACKOFF_MS; i++) {
    backoff <<= 1;
    if (backoff > RECONNECT_MAX_BACKOFF_MS) {
      backoff = RECONNECT_MAX_BACKOFF_MS;
      break;
    }
  }

  return backoff;
}

bool tracker_reconnect_allowed(const TrackerState& t, uint32_t nowMs) {
  if (t.reconnectNotBeforeMs == 0) return true;
  return (int32_t)(nowMs - t.reconnectNotBeforeMs) >= 0;
}

void mark_ble_topology_changed() {
  lastBleTopologyChangeMs = millis();
}

void schedule_reconnect_backoff(TrackerState& t, uint32_t nowMs, bool failureBackoff) {
  if (failureBackoff) {
    if (t.reconnectFailCount < RECONNECT_FAIL_COUNT_MAX) {
      t.reconnectFailCount++;
    }
    t.reconnectNotBeforeMs = nowMs + reconnect_backoff_for_fail_count(t.reconnectFailCount);
  } else {
    t.reconnectNotBeforeMs = nowMs + RECONNECT_AFTER_DISCONNECT_BACKOFF_MS;
  }
}

void clear_reconnect_backoff(TrackerState& t) {
  t.reconnectNotBeforeMs = 0;
  t.reconnectFailCount = 0;
}

bool any_tracker_connecting() {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (trackers[i].connecting) return true;
  }
  return false;
}

bool ble_topology_stable_for(uint32_t stableMs) {
  if (lastBleTopologyChangeMs == 0) return false;
  return (uint32_t)(millis() - lastBleTopologyChangeMs) >= stableMs;
}

void clear_tracker_addr(TrackerState& t) {
  memset(t.targetAddr, 0, sizeof(t.targetAddr));
  t.targetAddrValid = false;
  t.slimeAddress48 = 0x4D4F434F5000ULL | t.id;
}

int find_free_tracker_slot() {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    TrackerState& t = trackers[i];
    if (t.connected || t.connecting) continue;
    if (t.hostRegistered || t.everStreamed) continue;
    if (!t.targetAddrValid) return i;
  }

  return -1;
}

void register_tracker_peer_addr(TrackerState& t, const ble_gap_addr_t& peerAddr) {
  peer_addr_to_big_endian(peerAddr, t.targetAddr);
  t.targetAddrValid = true;
  t.slimeAddress48 = addr_to_u48(t.targetAddr);
}

int find_tracker_for_adv(const ble_gap_evt_adv_report_t* report) {
  if (!USE_AUTO_MAC_DISCOVERY || !report) {
    return -1;
  }

  if (is_rejected_peer_addr(report->peer_addr)) {
    return -1;
  }

  uint32_t nowMs = millis();

  int knownIndex = find_tracker_by_known_addr(report->peer_addr);
  if (knownIndex >= 0) {
    if (trackers[knownIndex].connected || trackers[knownIndex].connecting) {
      return -1;
    }
    if (!tracker_reconnect_allowed(trackers[knownIndex], nowMs)) {
      return -1;
    }
    return knownIndex;
  }

  bool likelyMocopi = adv_looks_like_mocopi(report);

  // Fixed-ID auto discovery mode:
  // 1. Collect mocopi-looking MAC addresses for a short window.
  // 2. Sort those MAC addresses.
  // 3. Assign sorted[0] -> tracker ID 0, sorted[1] -> tracker ID 1, etc.
  // 4. Only then start connecting, so IDs do not depend on advertisement/connection order.
  if (AUTO_FIXED_ID_BY_MAC_SORT && !autoDiscoveryMapFinalized) {
    if (likelyMocopi) {
      remember_auto_discovered_peer_addr(report->peer_addr);
    }

    bool shouldFinalize = autoDiscoveredAddrCount >= TRACKER_COUNT;
    if (!AUTO_DISCOVERY_REQUIRE_FULL_SET && auto_discovery_collect_window_elapsed()) {
      shouldFinalize = true;
    }

    if (shouldFinalize) {
      finalize_auto_discovery_map();

      knownIndex = find_tracker_by_known_addr(report->peer_addr);
      if (knownIndex >= 0 && !trackers[knownIndex].connected && !trackers[knownIndex].connecting &&
          tracker_reconnect_allowed(trackers[knownIndex], nowMs)) {
        return knownIndex;
      }
    }

    return -1;
  }

  // Some mocopi firmware/advertising states do not expose the local name or the
  // 128-bit service UUID in the primary advertisement. In that case, probe
  // connectable advertisements and validate after connection by discovering the
  // mocopi GATT service. This keeps the original fallback behavior, but IDs for
  // these fallback-only devices can only be assigned after the GATT probe.
  bool probeCandidate = false;
#if defined(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED) || defined(ARDUINO_ARCH_NRF52)
  probeCandidate = PROBE_UNKNOWN_CONNECTABLE_ADVERTISEMENTS && report->type.connectable && !report->type.scan_response;
#else
  probeCandidate = PROBE_UNKNOWN_CONNECTABLE_ADVERTISEMENTS;
#endif

  if (!likelyMocopi && !probeCandidate) {
    return -1;
  }

  int freeIndex = find_free_tracker_slot();
  if (freeIndex < 0) {
    return -1;
  }

  if (!tracker_reconnect_allowed(trackers[freeIndex], nowMs)) {
    return -1;
  }

  register_tracker_peer_addr(trackers[freeIndex], report->peer_addr);
  return freeIndex;
}

float half_to_float(uint16_t h) {
  uint16_t sign = (h >> 15) & 0x0001;
  uint16_t exp = (h >> 10) & 0x001F;
  uint16_t frac = h & 0x03FF;

  float value;

  if (exp == 0) {
    if (frac == 0) {
      value = 0.0f;
    } else {
      value = ldexpf((float)frac, -24);
    }
  } else if (exp == 31) {
    value = (frac == 0) ? INFINITY : NAN;
  } else {
    value = ldexpf(1.0f + ((float)frac / 1024.0f), (int)exp - 15);
  }

  return sign ? -value : value;
}

int16_t read_i16_le(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t read_u16_le(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

Quaternion multiply_quaternion(const Quaternion& a, const Quaternion& b) {
  Quaternion out;
  out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return out;
}

Quaternion normalize_quaternion(const Quaternion& q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n <= 0.0f || isnan(n) || isinf(n)) return Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  return Quaternion{q.w / n, q.x / n, q.y / n, q.z / n};
}

Quaternion convert_quaternion_axes(const Quaternion& q) {
  Quaternion out;
  out.w = q.w;
  out.x = -q.x;
  out.y = q.z;
  out.z = q.y;
  return normalize_quaternion(out);
}

Accel convert_accel_axes(float axis1, float axis2, float axis3) {
  // Axis alignment for SlimeVR send path:
  // axis1=x, axis2=z?, axis3=y? -> (-x, z, y) * 0.12
  float ax = axis1;
  float ay = axis3;
  float az = axis2;

  Accel out;
  out.x = -ax * 0.12f;
  out.y = az * 0.12f;
  out.z = ay * 0.12f;
  return out;
}

bool parse_mocopi_packet(const uint8_t* data, uint16_t len, Quaternion& outQuat, Accel& outAccel) {
  if (!data || len < 30) return false;

  Quaternion rawQuat;
  rawQuat.w = (float)read_i16_le(&data[8]) / 8192.0f;
  rawQuat.x = (float)read_i16_le(&data[10]) / 8192.0f;
  rawQuat.y = (float)read_i16_le(&data[12]) / 8192.0f;
  rawQuat.z = (float)read_i16_le(&data[14]) / 8192.0f;
  outQuat = convert_quaternion_axes(rawQuat);

  float ax = half_to_float(read_u16_le(&data[24]));
  float ay = half_to_float(read_u16_le(&data[26]));
  float az = half_to_float(read_u16_le(&data[28]));

  if (MOCOPI_ACCEL_RAW_UNIT_IS_G) {
    ax *= STANDARD_GRAVITY_MS2;
    ay *= STANDARD_GRAVITY_MS2;
    az *= STANDARD_GRAVITY_MS2;
  }

  Accel converted = convert_accel_axes(ax, ay, az);
  outAccel.x = clamp_float(converted.x, -ACCEL_REPORT_MAX_ABS_MS2, ACCEL_REPORT_MAX_ABS_MS2);
  outAccel.y = clamp_float(converted.y, -ACCEL_REPORT_MAX_ABS_MS2, ACCEL_REPORT_MAX_ABS_MS2);
  outAccel.z = clamp_float(converted.z, -ACCEL_REPORT_MAX_ABS_MS2, ACCEL_REPORT_MAX_ABS_MS2);

  return true;
}

// ============================================================
// SlimeVR HID packet builders
// ============================================================

uint8_t* packet_ptr(uint8_t* report, uint8_t packetIndex) {
  return &report[packetIndex * HID_PACKET_SIZE];
}

void make_packet_receiver_register(uint8_t* p, const TrackerState& t) {
  memset(p, 0, HID_PACKET_SIZE);
  p[0] = 255;
  p[1] = t.id;
  write_u64_le(t.slimeAddress48, &p[2]);
}

void make_packet_device_info(uint8_t* p, const TrackerState& t) {
  memset(p, 0, HID_PACKET_SIZE);

  p[0] = 0;
  p[1] = t.id;
  p[2] = t.batteryValid ? t.batteryPercent : BATTERY_PERCENT_UNKNOWN;
  p[3] = BATTERY_VOLTAGE_3V70_ENCODED;
  p[4] = TEMPERATURE_UNKNOWN;
  p[5] = SLIMEVR_BOARD_CUSTOM;
  p[6] = SLIMEVR_MCU_NRF52;
  p[7] = 0;
  p[8] = SLIMEVR_IMU_TYPE_MOCOPI_COMPAT;
  p[9] = SLIMEVR_MAG_NOT_SUPPORTED;

  write_u16_le(FW_DATE_ENCODED, &p[10]);

  p[12] = FW_MAJOR;
  p[13] = FW_MINOR;
  p[14] = FW_PATCH;
  p[15] = RSSI_FAKE;
}

void make_packet_quat_accel(uint8_t* p, const TrackerState& t) {
  memset(p, 0, HID_PACKET_SIZE);

  p[0] = 1;
  p[1] = t.id;

  write_i16_le(float_to_q15(t.quat.x), &p[2]);
  write_i16_le(float_to_q15(t.quat.y), &p[4]);
  write_i16_le(float_to_q15(t.quat.z), &p[6]);
  write_i16_le(float_to_q15(t.quat.w), &p[8]);

  write_i16_le(accel_ms2_to_q7(t.accel.x), &p[10]);
  write_i16_le(accel_ms2_to_q7(t.accel.y), &p[12]);
  write_i16_le(accel_ms2_to_q7(t.accel.z), &p[14]);
}

bool all_trackers_ever_streamed() {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (!trackers[i].everStreamed) {
      return false;
    }
  }
  return true;
}

void update_initial_streaming_latch() {
  if (initialAllTrackersStreamed) return;
  if (all_trackers_ever_streamed()) {
    initialAllTrackersStreamed = true;
  }
}

bool tracker_grace_online_for_host(const TrackerState& t) {
  if (!t.hostRegistered) return false;
  if (t.hostOnlineUntilMs == 0) return false;
  return (int32_t)(t.hostOnlineUntilMs - millis()) > 0;
}

bool tracker_host_should_be_online(const TrackerState& t) {
  if (t.connected && t.streamStarted) {
    if (HOLD_HOST_ONLINE_UNTIL_ALL_INITIAL_STREAMING && !initialAllTrackersStreamed) {
      return true;
    }

    if (t.hasFreshSample && (uint32_t)(millis() - t.lastSampleMs) <= SENSOR_STALE_TIMEOUT_MS) {
      return true;
    }
  }

  if (tracker_grace_online_for_host(t)) {
    return true;
  }

  return false;
}

bool tracker_active_for_host(const TrackerState& t) {
  return tracker_host_should_be_online(t);
}

void make_packet_status(uint8_t* p, const TrackerState& t) {
  memset(p, 0, HID_PACKET_SIZE);

  p[0] = 3;
  p[1] = t.id;
  p[2] = tracker_active_for_host(t) ? 1 : 0;
  p[3] = 0;
  // SlimeVR UI may treat received=0 and lost=0 as an invalid/no-data status.
  // The HID payload is only 8-bit, so avoid the 255 -> 0 wrap becoming visible.
  p[4] = encode_status_counter_nonzero(t.packetsReceivedCounter + 1UL);
  p[5] = encode_status_counter_nonzero(t.packetsLostCounter);
  p[6] = 0;
  p[7] = 0;
  p[15] = RSSI_FAKE;
}

// ============================================================
// USB HID
// ============================================================

void setup_usb_hid() {
  TinyUSBDevice.setID(SLIMEVR_HID_RECEIVER_VID, SLIMEVR_HID_RECEIVER_PID);
  TinyUSBDevice.setManufacturerDescriptor("SlimeVR");
  TinyUSBDevice.setProductDescriptor("SlimeVR HID Receiver");

  usb_hid.setPollInterval(1);
  usb_hid.setReportDescriptor(report_descriptor, sizeof(report_descriptor));
  usb_hid.setStringDescriptor("SlimeVR HID Receiver");
  usb_hid.begin();

  uint32_t startMs = millis();
  while (!TinyUSBDevice.mounted() && (millis() - startMs < USB_MOUNT_WAIT_MS)) {
    delay(1);
  }
}

bool tracker_sample_valid_for_hid(const TrackerState& t) {
  return tracker_active_for_host(t);
}

bool tracker_needs_offline_status(const TrackerState& t, uint32_t nowMs) {
  if (t.connected) return false;
  if (!t.hostRegistered) return false;
  if ((int32_t)(t.hostOnlineUntilMs - nowMs) > 0) return false;
  return (int32_t)(t.offlineStatusUntilMs - nowMs) > 0;
}

void make_offline_status_only_report(uint8_t* report, const TrackerState& t) {
  memset(report, 0, HID_REPORT_SIZE);

  // Do not include receiver-register, device-info, or quaternion packets here.
  // The important part is to stop emitting p[0] == 1 sensor packets after power-off.
  // Repeating the status packet fills all four HID slots without creating fake data.
  make_packet_status(packet_ptr(report, 0), t);
  make_packet_status(packet_ptr(report, 1), t);
  make_packet_status(packet_ptr(report, 2), t);
  make_packet_status(packet_ptr(report, 3), t);
}

void make_tracker_hid_report(uint8_t* report, const TrackerState& t) {
  memset(report, 0, HID_REPORT_SIZE);

  make_packet_receiver_register(packet_ptr(report, 0), t);
  make_packet_device_info(packet_ptr(report, 1), t);

  if (tracker_active_for_host(t)) {
    make_packet_quat_accel(packet_ptr(report, 2), t);
  } else {
    // Connected but not streaming yet, or sample went stale.
    // Send status instead of quaternion so the server does not keep the tracker alive
    // from repeated stale sensor samples.
    make_packet_status(packet_ptr(report, 2), t);
  }

  make_packet_status(packet_ptr(report, 3), t);
}

void send_one_tracker_hid_report(uint8_t trackerIndex) {
  if (trackerIndex >= TRACKER_COUNT) return;

  TrackerState& t = trackers[trackerIndex];
  uint32_t nowMs = millis();

  bool sendConnectedOrStale = t.connected || tracker_grace_online_for_host(t);
  bool sendOfflineNotice = tracker_needs_offline_status(t, nowMs);

  if (!sendConnectedOrStale && !sendOfflineNotice) return;

  if (!usb_hid.ready()) {
    t.hidFailCount++;
    return;
  }

  uint8_t report[HID_REPORT_SIZE];

  if (sendOfflineNotice) {
    make_offline_status_only_report(report, t);
  } else {
    make_tracker_hid_report(report, t);
  }

  bool sent = usb_hid.sendReport(0, report, HID_REPORT_SIZE);

  if (sent) {
    if (sendConnectedOrStale) {
      t.hostRegistered = true;
      if (tracker_active_for_host(t)) {
        t.packetsReceivedCounter++;
      }
    }
    t.hidSentCount++;
  } else {
    t.packetsLostCounter++;
    t.hidFailCount++;
  }
}

void service_hid_reports() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - prevHidSendUs) < HID_SEND_INTERVAL_US) return;
  prevHidSendUs = nowUs;

  uint32_t nowMs = millis();

  for (uint8_t attempt = 0; attempt < TRACKER_COUNT; attempt++) {
    uint8_t index = nextHidTrackerIndex;
    nextHidTrackerIndex = (uint8_t)((nextHidTrackerIndex + 1) % TRACKER_COUNT);

    if (trackers[index].connected || tracker_grace_online_for_host(trackers[index]) || tracker_needs_offline_status(trackers[index], nowMs)) {
      send_one_tracker_hid_report(index);
      break;
    }
  }
}

// ============================================================
// Battery handling
// ============================================================

void update_tracker_battery_from_value(TrackerState& t, uint8_t value) {
  // mocopi/nRF reconnect races can occasionally return 0 or 1 while the Battery
  // Service has not stabilized. Treat that as unknown instead of advertising a
  // bogus 1% battery to SlimeVR.
  if (value <= 1) {
    if (t.batterySuspiciousReadCount < 255) {
      t.batterySuspiciousReadCount++;
    }
    if (!t.batteryValid || t.batterySuspiciousReadCount < 3) {
      t.batteryPercent = BATTERY_PERCENT_UNKNOWN;
      t.batteryValid = false;
      t.lastBatteryReadMs = millis();
      return;
    }
  }

  t.batterySuspiciousReadCount = 0;

  if (value <= 100) {
    t.batteryPercent = value;
    t.batteryValid = true;
  } else {
    t.batteryPercent = BATTERY_PERCENT_UNKNOWN;
    t.batteryValid = false;
  }

  t.lastBatteryReadMs = millis();
}

void mocopi_battery_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  int index = find_tracker_by_battery_char(chr);
  if (index < 0 || !data || len < 1) {
    return;
  }

  update_tracker_battery_from_value(trackers[index], data[0]);
}

bool read_tracker_battery(TrackerState& t) {
  if (!t.connected) return false;
  if (!t.batteryDiscovered) return false;

  uint8_t value = BATTERY_PERCENT_UNKNOWN;
  uint16_t readLen = t.batteryLevelChar.read(&value, 1);
  if (readLen != 1) {
    return false;
  }

  update_tracker_battery_from_value(t, value);
  return true;
}

bool all_trackers_connected() {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (!trackers[i].connected) {
      return false;
    }
  }
  return true;
}

bool setup_battery_for_tracker(TrackerState& t, uint16_t connHandle) {
  // Optional path. Never disconnect or fail mocopi streaming because of battery discovery.
  if (!t.connected) return false;
  if (t.batterySetupAttempts < 255) {
    t.batterySetupAttempts++;
  }

  if (!t.batteryDiscovered) {
    if (!discover_service_with_retry("battery service", t.batteryService, connHandle, 1, 80)) {
      return false;
    }

    if (!discover_characteristic_with_retry("battery level characteristic", t.batteryLevelChar, 1, 80)) {
      return false;
    }

    t.batteryDiscovered = true;
    t.batteryLevelChar.setNotifyCallback(mocopi_battery_notify_callback);

    // Battery Level notify is optional. Ignore failure and use periodic read.
    t.batteryLevelChar.enableNotify();
  }

  read_tracker_battery(t);
  return t.batteryValid || t.batteryDiscovered;
}

void service_battery_setup() {
  if (!ENABLE_BATTERY_SERVICE) {
    return;
  }

  // Keep reconnection stable: do not perform optional Battery Service discovery
  // until every tracker is connected, every tracker has streamed at least once,
  // no tracker is currently connecting, and the BLE topology has been quiet.
  if (!all_trackers_connected()) {
    return;
  }

  if (!initialAllTrackersStreamed) {
    return;
  }

  if (any_tracker_connecting()) {
    return;
  }

  if (!ble_topology_stable_for(BATTERY_SETUP_AFTER_TOPOLOGY_STABLE_MS)) {
    return;
  }

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastBatterySetupAttemptMs) < BATTERY_SETUP_STEP_INTERVAL_MS) {
    return;
  }
  lastBatterySetupAttemptMs = nowMs;

  for (uint8_t attempt = 0; attempt < TRACKER_COUNT; attempt++) {
    uint8_t index = nextBatterySetupIndex;
    nextBatterySetupIndex = (uint8_t)((nextBatterySetupIndex + 1) % TRACKER_COUNT);

    TrackerState& t = trackers[index];
    if (!t.connected) continue;
    if (t.batterySetupDone) continue;

    bool ok = setup_battery_for_tracker(t, t.connHandle);
    if (ok || t.batterySetupAttempts >= BATTERY_SETUP_MAX_ATTEMPTS) {
      t.batterySetupDone = true;
    }
    break;
  }
}

void service_battery_reads() {
  if (!ENABLE_BATTERY_SERVICE) {
    return;
  }

  uint32_t nowMs = millis();

  if (!initialAllTrackersStreamed || any_tracker_connecting() ||
      !ble_topology_stable_for(BATTERY_READ_AFTER_TOPOLOGY_STABLE_MS)) {
    return;
  }

  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    TrackerState& t = trackers[i];

    if (!t.connected) continue;
    if (!t.batteryDiscovered) continue;

    uint32_t intervalMs = t.batteryValid ? BATTERY_READ_INTERVAL_MS : BATTERY_RETRY_INTERVAL_MS;
    if ((uint32_t)(nowMs - t.lastBatteryReadMs) >= intervalMs) {
      read_tracker_battery(t);
    }
  }
}

// ============================================================
// BLE callbacks
// ============================================================

void mocopi_ff03_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)chr;
  (void)data;
  (void)len;
}

Accel low_pass_accel(const Accel& previous, const Accel& current, float alpha) {
  float a = clamp_float(alpha, 0.0f, 1.0f);

  Accel out;
  out.x = previous.x + (current.x - previous.x) * a;
  out.y = previous.y + (current.y - previous.y) * a;
  out.z = previous.z + (current.z - previous.z) * a;
  return out;
}

void mocopi_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  int index = find_tracker_by_data_char(chr);
  if (index < 0) {
    return;
  }

  TrackerState& t = trackers[index];

  t.notifyCount++;

  Quaternion q;
  Accel a;
  if (!parse_mocopi_packet(data, len, q, a)) {
    return;
  }

  t.quat = q;

  if (ENABLE_ACCEL_LOW_PASS_FOR_HID && t.hasFreshSample) {
    t.accel = low_pass_accel(t.accel, a, ACCEL_LPF_ALPHA);
  } else {
    t.accel = a;
  }

  t.lastSampleMs = millis();
  t.hasFreshSample = true;
  t.everStreamed = true;
  t.hostOnlineUntilMs = t.lastSampleMs + HOST_RECONNECT_GRACE_MS;
  update_initial_streaming_latch();
}


bool has_unconnected_tracker() {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (!trackers[i].connected) {
      return true;
    }
  }
  return false;
}

void restart_scanner_now() {
  if (!has_unconnected_tracker()) {
    Bluefruit.Scanner.stop();
    return;
  }

  Bluefruit.Scanner.start(0);
  lastScanRestartMs = millis();
}

void stop_scanner_if_all_connected() {
  if (!has_unconnected_tracker()) {
    Bluefruit.Scanner.stop();
  }
}

void scan_callback(ble_gap_evt_adv_report_t* report) {
  int trackerIndex = find_tracker_for_adv(report);

  if (trackerIndex < 0) {
    Bluefruit.Scanner.resume();
    return;
  }

  TrackerState& t = trackers[trackerIndex];
  uint32_t nowMs = millis();

  if (!tracker_reconnect_allowed(t, nowMs)) {
    Bluefruit.Scanner.resume();
    return;
  }

  t.connecting = true;
  t.connectingStartedMs = nowMs;
  mark_ble_topology_changed();

  bool started = Bluefruit.Central.connect(report);
  if (!started) {
    t.connecting = false;
    t.connectingStartedMs = 0;
    Bluefruit.Scanner.resume();
  }
}

void connect_callback(uint16_t connHandle) {
  BLEConnection* conn = Bluefruit.Connection(connHandle);
  if (!conn) return;

  delay(GATT_CONNECT_SETTLE_DELAY_MS);

  ble_gap_addr_t peer = conn->getPeerAddr();

  int index = -1;
  bool matchedKnownAddr = false;

  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (!trackers[i].targetAddrValid) continue;
    if (trackers[i].connected) continue;
    if (addr_matches_big_or_little(peer, trackers[i].targetAddr)) {
      index = i;
      matchedKnownAddr = true;
      break;
    }
  }

  if (index < 0) {
    int freeIndex = find_free_tracker_slot();
    if (freeIndex >= 0) {
      index = freeIndex;
      register_tracker_peer_addr(trackers[index], peer);
    }
  }

  if (index < 0) {
    for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
      trackers[i].connecting = false;
      trackers[i].connectingStartedMs = 0;
    }
    conn->disconnect();

    if (has_unconnected_tracker()) {
      restart_scanner_now();
    }

    return;
  }

  TrackerState& t = trackers[index];

  t.connHandle = connHandle;
  t.connected = true;
  mark_ble_topology_changed();
  t.connecting = false;
  t.connectingStartedMs = 0;
  t.streamStarted = false;
  t.notifyEnabled = false;
  t.hasFreshSample = false;
  t.notifyCount = 0;
  t.hidSentCount = 0;
  t.hidFailCount = 0;
  t.batteryPercent = BATTERY_PERCENT_UNKNOWN;
  t.batteryValid = false;
  t.batteryDiscovered = false;
  t.batterySetupDone = false;
  t.batterySetupAttempts = 0;
  t.lastBatteryReadMs = 0;
  t.batterySuspiciousReadCount = 0;
  t.offlineStatusUntilMs = 0;
  if (!t.everStreamed) {
    t.hostOnlineUntilMs = millis() + HOST_RECONNECT_GRACE_MS;
  }
  t.quat = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  t.accel = Accel{0.0f, 0.0f, 0.0f};

  bool ok = true;
  bool mocopiPrimaryServiceDiscovered = false;

  if (ok) {
    if (!discover_service_with_retry(
          "mocopi service",
          t.service,
          connHandle
        )) {
      ok = false;
    } else {
      mocopiPrimaryServiceDiscovered = true;
    }
  }

  if (ok) {
    if (!discover_characteristic_with_retry(
          "mocopi data characteristic",
          t.dataChar
        )) {
      ok = false;
    }
  }

  if (ok) {
    t.dataChar.setNotifyCallback(mocopi_notify_callback);

    if (!t.dataChar.enableNotify()) {
      ok = false;
    } else {
      t.notifyEnabled = true;
    }
  }

  if (ok) {
    if (!discover_service_with_retry(
          "mocopi command service",
          t.commandService,
          connHandle
        )) {
      ok = false;
    }
  }

  if (ok) {
    if (!discover_characteristic_with_retry(
          "ff03 characteristic",
          t.ff03Char
        )) {
      ok = false;
    } else {
      t.ff03Char.setNotifyCallback(mocopi_ff03_notify_callback);

      if (!t.ff03Char.enableNotify()) {
        ok = false;
      }
    }
  }

  if (ok) {
    delay(300);

    bool preStartOk = mocopi_send_pre_start_sequence_by_handle(connHandle);

    if (!preStartOk) {
      ok = false;
    }
  }

  if (ok) {
    delay(300);

    bool written = mocopi_write_start_command_by_handle(connHandle);

    if (!written) {
      ok = false;
    }

    delay(200);
  }

  if (!ok) {
    uint32_t nowMs = millis();

    t.connected = false;
    t.connecting = false;
    t.connectingStartedMs = 0;
    t.streamStarted = false;
    t.notifyEnabled = false;
    t.hasFreshSample = false;
    t.offlineStatusUntilMs = 0;
    if (!t.everStreamed) {
      t.hostOnlineUntilMs = 0;
    }
    t.batteryValid = false;
    t.batteryDiscovered = false;
    t.batterySetupDone = false;
    t.batterySetupAttempts = 0;
    t.batteryPercent = BATTERY_PERCENT_UNKNOWN;
    t.lastBatteryReadMs = 0;
    t.connHandle = BLE_CONN_HANDLE_INVALID;
    t.offlineStatusUntilMs = 0;
    schedule_reconnect_backoff(t, nowMs, true);
    mark_ble_topology_changed();

    // Do not clear a known mocopi address just because service discovery failed
    // once during a reconnect. Clearing here is what can strand one tracker or
    // make it come back under a different ID. Only reject truly unknown probe
    // candidates that never reached the host/streaming path.
    if (!mocopiPrimaryServiceDiscovered && !matchedKnownAddr && !t.hostRegistered && !t.everStreamed) {
      remember_rejected_peer_addr(peer);
      t.hostRegistered = false;
      t.everStreamed = false;
      t.hostOnlineUntilMs = 0;
      t.reconnectNotBeforeMs = 0;
      t.reconnectFailCount = 0;
      clear_tracker_addr(t);
    }

    conn->disconnect();

    if (has_unconnected_tracker()) {
      restart_scanner_now();
    }

    return;
  }

  t.streamStarted = true;
  clear_reconnect_backoff(t);
  t.hostOnlineUntilMs = millis() + HOST_RECONNECT_GRACE_MS;
  mark_ble_topology_changed();

  if (has_unconnected_tracker()) {
    restart_scanner_now();
  } else {
    stop_scanner_if_all_connected();
  }
}

void disconnect_callback(uint16_t connHandle, uint8_t reason) {
  (void)reason;

  int index = find_tracker_by_conn(connHandle);

  if (index >= 0) {
    TrackerState& t = trackers[index];
    t.connected = false;
    t.connecting = false;
    t.connectingStartedMs = 0;
    t.streamStarted = false;
    t.notifyEnabled = false;
    t.hasFreshSample = false;
    t.batteryValid = false;
    t.batteryDiscovered = false;
    t.batterySetupDone = false;
    t.batterySetupAttempts = 0;
    t.batteryPercent = BATTERY_PERCENT_UNKNOWN;
    t.lastBatteryReadMs = 0;
    t.connHandle = BLE_CONN_HANDLE_INVALID;

    // Power-off disconnect path: explicitly tell the host that this tracker is offline.
    // Important: the offline notification path sends status packets only, never stale
    // quaternion packets, because sensor packets can keep the tracker alive server-side.
    uint32_t nowMs = millis();
    t.lastDisconnectMs = nowMs;
    schedule_reconnect_backoff(t, nowMs, false);
    mark_ble_topology_changed();

    if (t.everStreamed) {
      t.hostOnlineUntilMs = nowMs + HOST_RECONNECT_GRACE_MS;
      t.offlineStatusUntilMs = nowMs + HOST_RECONNECT_GRACE_MS + DISCONNECT_STATUS_NOTIFY_MS;
    } else {
      t.hostOnlineUntilMs = 0;
      t.offlineStatusUntilMs = nowMs + DISCONNECT_STATUS_NOTIFY_MS;
    }
    t.quat = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
    t.accel = Accel{0.0f, 0.0f, 0.0f};
  }

  restart_scanner_now();
}

// ============================================================
// BLE setup and scanning
// ============================================================

void setup_trackers() {
  autoDiscoveredAddrCount = 0;
  autoDiscoveryMapFinalized = false;
  autoDiscoveryStartedMs = 0;
  initialAllTrackersStreamed = false;
  memset(autoDiscoveredAddrs, 0, sizeof(autoDiscoveredAddrs));

  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    TrackerState& t = trackers[i];

    t.id = i;
    t.connected = false;
    t.connecting = false;
    t.connectingStartedMs = 0;
    t.streamStarted = false;
    t.notifyEnabled = false;
    t.hasFreshSample = false;
    t.hostRegistered = false;
    t.offlineStatusUntilMs = 0;
    t.everStreamed = false;
    t.hostOnlineUntilMs = 0;
    t.reconnectNotBeforeMs = 0;
    t.lastDisconnectMs = 0;
    t.reconnectFailCount = 0;
    t.batterySuspiciousReadCount = 0;
    t.batteryValid = false;
    t.batteryDiscovered = false;
    t.batterySetupDone = false;
    t.batterySetupAttempts = 0;
    t.batteryPercent = BATTERY_PERCENT_UNKNOWN;
    t.lastBatteryReadMs = 0;
    t.connHandle = BLE_CONN_HANDLE_INVALID;
    t.packetsReceivedCounter = 0;
    t.packetsLostCounter = 0;
    t.notifyCount = 0;
    t.hidSentCount = 0;
    t.hidFailCount = 0;

    clear_tracker_addr(t);

    t.service.begin();
    t.dataChar.begin(&t.service);

    t.commandService.begin();
    t.cmdChar.begin(&t.commandService);
    t.ff03Char.begin(&t.commandService);

    t.batteryService.begin();
    t.batteryLevelChar.begin(&t.batteryService);
  }
}

void setup_ble() {
  Bluefruit.autoConnLed(false);

  Bluefruit.configUuid128Count(2);
  Bluefruit.configCentralConn(64, 5, 1, 1);

  Bluefruit.begin(0, TRACKER_COUNT);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("mocopi-nRF52840-HID");

  setup_trackers();
  lastBleTopologyChangeMs = millis();

  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);

  Bluefruit.Scanner.setInterval(SCAN_INTERVAL_UNITS, SCAN_WINDOW_UNITS);
  Bluefruit.Scanner.useActiveScan(true);
  autoDiscoveryStartedMs = millis();
  restart_scanner_now();

  lastScanRestartMs = millis();
}

void service_connecting_timeout() {
  uint32_t nowMs = millis();

  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    TrackerState& t = trackers[i];
    if (!t.connecting) continue;
    if (t.connectingStartedMs == 0) continue;

    if ((uint32_t)(nowMs - t.connectingStartedMs) >= CONNECTING_STALE_TIMEOUT_MS) {
      t.connecting = false;
      t.connectingStartedMs = 0;
      schedule_reconnect_backoff(t, nowMs, true);
      mark_ble_topology_changed();
      if (has_unconnected_tracker()) {
        restart_scanner_now();
      }
    }
  }
}

void service_heartbeat() {
  update_initial_streaming_latch();
}

// ============================================================
// Status LED
// ============================================================

bool status_led_usb_active() {
#if defined(USBCON) || defined(USE_TINYUSB) || defined(ARDUINO_ARCH_NRF52)
  return TinyUSBDevice.mounted();
#else
  return true;
#endif
}

bool any_tracker_connected_for_led() {
  for (uint8_t i = 0; i < TRACKER_COUNT; i++) {
    if (trackers[i].connected && trackers[i].streamStarted) {
      return true;
    }
  }
  return false;
}

void set_status_led(bool on) {
#if defined(LED_BUILTIN)
  if (STATUS_LED_ACTIVE_LOW) {
    digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
  } else {
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
  }
#else
  (void)on;
#endif
}

void service_status_led() {
#if defined(LED_BUILTIN)
  static uint32_t lastToggleMs = 0;
  static bool blinkState = false;

  if (!status_led_usb_active()) {
    blinkState = false;
    set_status_led(false);
    return;
  }

  if (any_tracker_connected_for_led()) {
    uint32_t nowMs = millis();
    if ((uint32_t)(nowMs - lastToggleMs) >= STATUS_LED_SLOW_BLINK_MS) {
      lastToggleMs = nowMs;
      blinkState = !blinkState;
      set_status_led(blinkState);
    }
    return;
  }

  blinkState = true;
  lastToggleMs = millis();
  set_status_led(true);
#endif
}

// ============================================================
// Arduino setup / loop
// ============================================================

void setup() {
#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
  set_status_led(false);
#endif

  // Serial.begin() is intentionally not called.

  setup_usb_hid();
  setup_ble();

#if defined(LED_BUILTIN)
  service_status_led();
#endif
}

void loop() {
  service_hid_reports();
  service_connecting_timeout();
  service_battery_setup();
  service_battery_reads();
  service_heartbeat();
  service_status_led();

#if defined(ARDUINO_ARCH_NRF52)
  yield();
#endif
}