#include "bthome_receiver.h"
#include "esphome/core/log.h"
#include "mbedtls/ccm.h"

#include <cstring>
#include <cmath>

#ifdef USE_BTHOME_RECEIVER_NIMBLE
#include "host/ble_gap.h"
#endif

namespace esphome {
namespace bthome_receiver {

static const char *const TAG = "bthome_receiver";

#ifdef USE_BTHOME_RECEIVER_NIMBLE
// Static instance pointer for NimBLE callbacks
BTHomeReceiverHub *BTHomeReceiverHub::instance_ = nullptr;
#endif

// BTHome v2 object type lookup table
// Format: object_id -> (data_bytes, is_signed, factor, is_sensor, is_binary_sensor)
static const std::map<uint8_t, ObjectTypeInfo> OBJECT_TYPE_MAP = {
    // Basic sensors
    {0x00, {1, false, 1, true, false}},           // packet_id
    {0x01, {1, false, 1, true, false}},           // battery
    {0x02, {2, true, 0.01, true, false}},         // temperature
    {0x03, {2, false, 0.01, true, false}},        // humidity
    {0x04, {3, false, 0.01, true, false}},        // pressure
    {0x05, {3, false, 0.01, true, false}},        // illuminance
    {0x06, {2, false, 0.01, true, false}},        // mass_kg
    {0x07, {2, false, 0.01, true, false}},        // mass_lb
    {0x08, {2, true, 0.01, true, false}},         // dewpoint
    {0x09, {1, false, 1, true, false}},           // count_uint8
    {0x0A, {3, false, 0.001, true, false}},       // energy
    {0x0B, {3, false, 0.01, true, false}},        // power
    {0x0C, {2, false, 0.001, true, false}},       // voltage
    {0x0D, {2, false, 1, true, false}},           // pm2_5
    {0x0E, {2, false, 1, true, false}},           // pm10
    {0x12, {2, false, 1, true, false}},           // co2
    {0x13, {2, false, 1, true, false}},           // tvoc
    {0x14, {2, false, 0.01, true, false}},        // moisture
    {0x2E, {1, false, 1, true, false}},           // humidity_uint8
    {0x2F, {1, false, 1, true, false}},           // moisture_uint8

    // Binary sensors
    {0x0F, {1, false, 1, false, true}},           // generic_boolean
    {0x10, {1, false, 1, false, true}},           // power
    {0x11, {1, false, 1, false, true}},           // opening
    {0x15, {1, false, 1, false, true}},           // battery_low
    {0x16, {1, false, 1, false, true}},           // battery_charging
    {0x17, {1, false, 1, false, true}},           // carbon_monoxide
    {0x18, {1, false, 1, false, true}},           // cold
    {0x19, {1, false, 1, false, true}},           // connectivity
    {0x1A, {1, false, 1, false, true}},           // door
    {0x1B, {1, false, 1, false, true}},           // garage_door
    {0x1C, {1, false, 1, false, true}},           // gas
    {0x1D, {1, false, 1, false, true}},           // heat
    {0x1E, {1, false, 1, false, true}},           // light
    {0x1F, {1, false, 1, false, true}},           // lock
    {0x20, {1, false, 1, false, true}},           // moisture_binary
    {0x21, {1, false, 1, false, true}},           // motion
    {0x22, {1, false, 1, false, true}},           // moving
    {0x23, {1, false, 1, false, true}},           // occupancy
    {0x24, {1, false, 1, false, true}},           // plug
    {0x25, {1, false, 1, false, true}},           // presence
    {0x26, {1, false, 1, false, true}},           // problem
    {0x27, {1, false, 1, false, true}},           // running
    {0x28, {1, false, 1, false, true}},           // safety
    {0x29, {1, false, 1, false, true}},           // smoke
    {0x2A, {1, false, 1, false, true}},           // sound
    {0x2B, {1, false, 1, false, true}},           // tamper
    {0x2C, {1, false, 1, false, true}},           // vibration
    {0x2D, {1, false, 1, false, true}},           // window

    // Extended sensors
    {0x3D, {2, false, 1, true, false}},           // count_uint16
    {0x3E, {4, false, 1, true, false}},           // count_uint32
    {0x3F, {2, true, 0.1, true, false}},          // rotation
    {0x40, {2, false, 1, true, false}},           // distance_mm
    {0x41, {2, false, 0.1, true, false}},         // distance_m
    {0x42, {3, false, 0.001, true, false}},       // duration
    {0x43, {2, false, 0.001, true, false}},       // current
    {0x44, {2, false, 0.01, true, false}},        // speed
    {0x45, {2, true, 0.1, true, false}},          // temperature_01
    {0x46, {1, false, 0.1, true, false}},         // uv_index
    {0x47, {2, false, 0.1, true, false}},         // volume_l_01
    {0x48, {2, false, 1, true, false}},           // volume_ml
    {0x49, {2, false, 0.001, true, false}},       // volume_flow_rate
    {0x4A, {2, false, 0.1, true, false}},         // voltage_01
    {0x4B, {3, false, 0.001, true, false}},       // gas
    {0x4C, {4, false, 0.001, true, false}},       // gas_uint32
    {0x4D, {4, false, 0.001, true, false}},       // energy_uint32
    {0x4E, {4, false, 0.001, true, false}},       // volume_l
    {0x4F, {4, false, 0.001, true, false}},       // water
    {0x50, {4, false, 1, true, false}},           // timestamp
    {0x51, {2, false, 0.001, true, false}},       // acceleration
    {0x52, {2, false, 0.001, true, false}},       // gyroscope
    {0x55, {4, false, 0.001, true, false}},       // volume_storage
    {0x56, {2, false, 1, true, false}},           // conductivity
    {0x57, {1, true, 1, true, false}},            // temperature_sint8
    {0x58, {1, true, 0.35, true, false}},         // temperature_sint8_035
    {0x59, {1, true, 1, true, false}},            // count_sint8
    {0x5A, {2, true, 1, true, false}},            // count_sint16
    {0x5B, {4, true, 1, true, false}},            // count_sint32
    {0x5C, {4, true, 0.01, true, false}},         // power_sint32
    {0x5D, {2, true, 0.001, true, false}},        // current_sint16
    {0x5E, {2, false, 0.01, true, false}},        // direction
    {0x5F, {2, false, 0.1, true, false}},         // precipitation
    {0x60, {1, false, 1, true, false}},           // channel
    {0x61, {2, false, 1, true, false}},           // rotational_speed
};

// Object ID to human-readable name mapping (for dump mode)
static const std::map<uint8_t, const char *> OBJECT_ID_NAMES = {
    // Sensors
    {0x00, "packet_id"},
    {0x01, "battery"},
    {0x02, "temperature"},
    {0x03, "humidity"},
    {0x04, "pressure"},
    {0x05, "illuminance"},
    {0x06, "mass_kg"},
    {0x07, "mass_lb"},
    {0x08, "dewpoint"},
    {0x09, "count"},
    {0x0A, "energy"},
    {0x0B, "power"},
    {0x0C, "voltage"},
    {0x0D, "pm2_5"},
    {0x0E, "pm10"},
    {0x12, "co2"},
    {0x13, "tvoc"},
    {0x14, "moisture"},
    {0x2E, "humidity_uint8"},
    {0x2F, "moisture_uint8"},
    // Binary sensors
    {0x0F, "generic_boolean"},
    {0x10, "power_binary"},
    {0x11, "opening"},
    {0x15, "battery_low"},
    {0x16, "battery_charging"},
    {0x17, "carbon_monoxide"},
    {0x18, "cold"},
    {0x19, "connectivity"},
    {0x1A, "door"},
    {0x1B, "garage_door"},
    {0x1C, "gas"},
    {0x1D, "heat"},
    {0x1E, "light"},
    {0x1F, "lock"},
    {0x20, "moisture_binary"},
    {0x21, "motion"},
    {0x22, "moving"},
    {0x23, "occupancy"},
    {0x24, "plug"},
    {0x25, "presence"},
    {0x26, "problem"},
    {0x27, "running"},
    {0x28, "safety"},
    {0x29, "smoke"},
    {0x2A, "sound"},
    {0x2B, "tamper"},
    {0x2C, "vibration"},
    {0x2D, "window"},
    // Events
    {0x3A, "button"},
    {0x3C, "dimmer"},
    // Extended sensors
    {0x3D, "count_uint16"},
    {0x3E, "count_uint32"},
    {0x3F, "rotation"},
    {0x40, "distance_mm"},
    {0x41, "distance_m"},
    {0x42, "duration"},
    {0x43, "current"},
    {0x44, "speed"},
    {0x45, "temperature_01"},
    {0x46, "uv_index"},
    {0x47, "volume_l_01"},
    {0x48, "volume_ml"},
    {0x49, "volume_flow_rate"},
    {0x4A, "voltage_01"},
    {0x4B, "gas"},
    {0x4C, "gas_uint32"},
    {0x4D, "energy_uint32"},
    {0x4E, "volume_l"},
    {0x4F, "water"},
    {0x50, "timestamp"},
    {0x51, "acceleration"},
    {0x52, "gyroscope"},
    {0x53, "text"},
    {0x54, "raw"},
    {0x55, "volume_storage"},
    {0x56, "conductivity"},
    {0x57, "temperature_sint8"},
    {0x58, "temperature_sint8_035"},
    {0x59, "count_sint8"},
    {0x5A, "count_sint16"},
    {0x5B, "count_sint32"},
    {0x5C, "power_sint32"},
    {0x5D, "current_sint16"},
    {0x5E, "direction"},
    {0x5F, "precipitation"},
    {0x60, "channel"},
    {0x61, "rotational_speed"},
};

// ============================================================================
// BTHomeReceiverHub Implementation
// ============================================================================

void BTHomeReceiverHub::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BTHome Receiver...");

#ifdef USE_BTHOME_RECEIVER_NIMBLE
  instance_ = this;
  // Defer actual BLE initialization to loop() to ensure all other components are ready
  this->nimble_initialized_ = false;
  this->init_attempted_ = false;
  ESP_LOGI(TAG, "BTHome Receiver configured, BLE init deferred to loop");
#else
  // Bluedroid setup is handled by esp32_ble_tracker
  ESP_LOGI(TAG, "Bluedroid receiver initialized");
#endif
}

#ifdef USE_BTHOME_RECEIVER_NIMBLE
// Static semaphore for waiting on NimBLE sync
static SemaphoreHandle_t nimble_sync_semaphore_ = nullptr;

void BTHomeReceiverHub::init_nimble_() {
  ESP_LOGI(TAG, "Initializing NimBLE...");

  // Create semaphore to wait for host-controller sync
  nimble_sync_semaphore_ = xSemaphoreCreateBinary();
  if (nimble_sync_semaphore_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create sync semaphore");
    this->mark_failed();
    return;
  }

  // For ESP-IDF 5.0+, nimble_port_init() handles BT controller init internally
  // Just call it directly - no manual esp_bt_controller_init/enable needed
  esp_err_t ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
    vSemaphoreDelete(nimble_sync_semaphore_);
    nimble_sync_semaphore_ = nullptr;
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "NimBLE port initialized, configuring host");

  // Configure NimBLE host callbacks
  ble_hs_cfg.reset_cb = nimble_on_reset_;
  ble_hs_cfg.sync_cb = nimble_on_sync_;

  // Start NimBLE host task
  nimble_port_freertos_init(nimble_host_task_);

  // Wait for host-controller sync (with timeout)
  ESP_LOGI(TAG, "Waiting for NimBLE host sync...");
  if (xSemaphoreTake(nimble_sync_semaphore_, pdMS_TO_TICKS(5000)) != pdTRUE) {
    ESP_LOGE(TAG, "Timeout waiting for NimBLE sync");
    vSemaphoreDelete(nimble_sync_semaphore_);
    nimble_sync_semaphore_ = nullptr;
    this->mark_failed();
    return;
  }

  // Cleanup semaphore
  vSemaphoreDelete(nimble_sync_semaphore_);
  nimble_sync_semaphore_ = nullptr;

  this->nimble_initialized_ = true;
  ESP_LOGI(TAG, "NimBLE receiver initialized successfully");
}
#endif

void BTHomeReceiverHub::dump_config() {
  ESP_LOGCONFIG(TAG, "BTHome Receiver:");
#ifdef USE_BTHOME_RECEIVER_NIMBLE
  ESP_LOGCONFIG(TAG, "  BLE Stack: NimBLE");
#else
  ESP_LOGCONFIG(TAG, "  BLE Stack: Bluedroid");
#endif
  ESP_LOGCONFIG(TAG, "  Dump Interval: %ums", this->dump_interval_);
  ESP_LOGCONFIG(TAG, "  Registered Devices: %zu", this->devices_.size());
  for (auto *device : this->devices_) {
    uint64_t addr = device->get_mac_address();
    ESP_LOGCONFIG(TAG, "    MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                  (uint8_t)((addr >> 40) & 0xFF), (uint8_t)((addr >> 32) & 0xFF),
                  (uint8_t)((addr >> 24) & 0xFF), (uint8_t)((addr >> 16) & 0xFF),
                  (uint8_t)((addr >> 8) & 0xFF), (uint8_t)(addr & 0xFF));
  }
}

void BTHomeReceiverHub::dump_advertisement_(uint64_t address, const uint8_t *data, size_t len) {
  // Format MAC address in standard format (MSB first, matches ESPHome config format)
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
           (uint8_t)((address >> 40) & 0xFF),
           (uint8_t)((address >> 32) & 0xFF),
           (uint8_t)((address >> 24) & 0xFF),
           (uint8_t)((address >> 16) & 0xFF),
           (uint8_t)((address >> 8) & 0xFF),
           (uint8_t)(address & 0xFF));

  if (len < 1) {
    return;
  }

  // Build raw hex string
  std::string raw_hex;
  for (size_t i = 0; i < len; i++) {
    char hex[4];
    snprintf(hex, sizeof(hex), "%02X ", data[i]);
    raw_hex += hex;
  }

  // First byte is device_info
  uint8_t device_info = data[0];
  bool is_encrypted = (device_info & BTHOME_DEVICE_INFO_ENCRYPTED_MASK) != 0;

  // Build measurements string
  std::string measurements;
  size_t pos = 1;  // Skip device_info

  while (pos < len) {
    if (pos + 1 > len) break;

    uint8_t object_id = data[pos++];

    // Get human-readable name
    const char *name = "?";
    auto name_it = OBJECT_ID_NAMES.find(object_id);
    if (name_it != OBJECT_ID_NAMES.end()) {
      name = name_it->second;
    }

    // Handle special types
    if (object_id == OBJECT_ID_BUTTON || object_id == OBJECT_ID_DIMMER) {
      if (pos + 1 > len) break;
      pos++;
      continue;
    }

    if (object_id == OBJECT_ID_TEXT || object_id == OBJECT_ID_RAW) {
      if (pos + 1 > len) break;
      uint8_t str_len = data[pos++];
      if (pos + str_len > len) break;
      pos += str_len;
      continue;
    }

    auto it = OBJECT_TYPE_MAP.find(object_id);
    if (it == OBJECT_TYPE_MAP.end()) break;

    const ObjectTypeInfo &type_info = it->second;
    if (pos + type_info.data_bytes > len) break;

    char val_str[32];
    if (type_info.is_binary_sensor) {
      snprintf(val_str, sizeof(val_str), "%s=%s", name, data[pos] ? "ON" : "OFF");
    } else {
      int32_t raw_value = 0;
      if (type_info.is_signed) {
        switch (type_info.data_bytes) {
          case 1: raw_value = static_cast<int8_t>(data[pos]); break;
          case 2: raw_value = static_cast<int16_t>(data[pos] | (data[pos + 1] << 8)); break;
          case 3:
            raw_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16);
            if (raw_value & 0x800000) raw_value |= 0xFF000000;
            break;
          case 4: raw_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16) | (data[pos + 3] << 24); break;
        }
      } else {
        switch (type_info.data_bytes) {
          case 1: raw_value = data[pos]; break;
          case 2: raw_value = data[pos] | (data[pos + 1] << 8); break;
          case 3: raw_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16); break;
          case 4: raw_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16) | (data[pos + 3] << 24); break;
        }
      }
      float value = raw_value * type_info.factor;
      snprintf(val_str, sizeof(val_str), "%s=%.2f", name, value);
    }
    if (!measurements.empty()) measurements += " ";
    measurements += val_str;
    pos += type_info.data_bytes;
  }

  ESP_LOGI(TAG, "[%s] %s| %s", mac_str, is_encrypted ? "ENC " : "", measurements.c_str());
}

void BTHomeReceiverHub::loop() {
#ifdef USE_BTHOME_RECEIVER_NIMBLE
  // Deferred NimBLE initialization - wait for other components to be ready
  if (!this->nimble_initialized_ && !this->init_attempted_) {
    // Wait a bit after boot before initializing BLE (use ESP-IDF timer)
    static int64_t boot_time = esp_timer_get_time();
    if ((esp_timer_get_time() - boot_time) > 2000000) {  // 2 second delay (microseconds)
      this->init_attempted_ = true;
      this->init_nimble_();
    }
  }
#endif

  // Periodic dump of all detected devices
  if (this->dump_interval_ > 0) {
    uint32_t now = esp_timer_get_time() / 1000;  // Convert microseconds to milliseconds
    if (now - this->last_dump_time_ >= this->dump_interval_) {
      this->last_dump_time_ = now;
      this->dump_all_devices_();
    }
  }
}

void BTHomeReceiverHub::register_device(BTHomeDevice *device) {
  this->devices_.push_back(device);
  ESP_LOGV(TAG, "Registered device: %012llX", device->get_mac_address());
}

BTHomeDevice *BTHomeReceiverHub::find_device_(uint64_t address) {
  for (auto *device : this->devices_) {
    if (device->get_mac_address() == address) {
      return device;
    }
  }
  return nullptr;
}

void BTHomeReceiverHub::process_advertisement_for_all_devices_(uint64_t address, const std::vector<uint8_t> &service_data) {
  // Process advertisement for all devices with matching MAC address
  // (in case multiple device instances exist, e.g., one from hub config and one from sensor platform)
  bool found_any = false;
  for (auto *device : this->devices_) {
    if (device->get_mac_address() == address) {
      found_any = true;
      device->parse_advertisement(service_data);
    }
  }
  if (!found_any) {
    ESP_LOGV(TAG, "No registered device found for address %012llX", address);
  }
}

void BTHomeReceiverHub::cache_device_data_(uint64_t address, const uint8_t *data, size_t len) {
  uint32_t now = esp_timer_get_time() / 1000;

  // Find existing entry or add new one
  for (auto &entry : this->detected_devices_) {
    if (entry.first == address) {
      entry.second.last_data.assign(data, data + len);
      entry.second.last_seen = now;
      return;
    }
  }

  // Add new device
  DetectedDevice dev;
  dev.last_data.assign(data, data + len);
  dev.last_seen = now;
  this->detected_devices_.emplace_back(address, dev);
}

void BTHomeReceiverHub::dump_all_devices_() {
  if (this->detected_devices_.empty()) {
    return;
  }

  uint32_t now = esp_timer_get_time() / 1000;

  for (const auto &entry : this->detected_devices_) {
    uint64_t address = entry.first;
    const DetectedDevice &dev = entry.second;
    uint32_t age_sec = (now - dev.last_seen) / 1000;

    // Check if registered
    bool is_registered = this->find_device_(address) != nullptr;

    // Parse and dump the cached data
    this->dump_advertisement_(address, dev.last_data.data(), dev.last_data.size());
    if (is_registered) {
      ESP_LOGI(TAG, "  ^ (last seen %us ago) [REGISTERED]", age_sec);
    }
  }
}

// ============================================================================
// NimBLE Implementation
// ============================================================================

#ifdef USE_BTHOME_RECEIVER_NIMBLE

void BTHomeReceiverHub::nimble_host_task_(void *param) {
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void BTHomeReceiverHub::nimble_on_sync_() {
  ESP_LOGI(TAG, "NimBLE host-controller synchronized");

  // Signal that sync is complete
  if (nimble_sync_semaphore_ != nullptr) {
    xSemaphoreGive(nimble_sync_semaphore_);
  }

  // Start scanning
  if (instance_ != nullptr) {
    instance_->start_scanning_();
  }
}

void BTHomeReceiverHub::nimble_on_reset_(int reason) {
  ESP_LOGW(TAG, "NimBLE host reset, reason: %d", reason);
}

int BTHomeReceiverHub::nimble_gap_event_(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
    case BLE_GAP_EVENT_DISC:
      // Advertisement received
      if (instance_ != nullptr) {
        instance_->process_nimble_advertisement(&event->disc);
      }
      break;

    case BLE_GAP_EVENT_DISC_COMPLETE:
      // Discovery completed - restart scanning
      ESP_LOGD(TAG, "Scan complete, restarting...");
      if (instance_ != nullptr) {
        instance_->start_scanning_();
      }
      break;

    default:
      break;
  }
  return 0;
}

void BTHomeReceiverHub::start_scanning_() {
  if (this->scanning_) {
    return;
  }

  struct ble_gap_disc_params disc_params;
  memset(&disc_params, 0, sizeof(disc_params));

  // Passive scanning (don't send scan requests)
  disc_params.passive = 1;
  // Filter duplicates disabled to receive all advertisements
  disc_params.filter_duplicates = 0;
  // Scan interval and window (in 0.625ms units)
  // 160 = 100ms interval, 80 = 50ms window
  disc_params.itvl = 160;
  disc_params.window = 80;
  // Limited discovery mode disabled
  disc_params.limited = 0;

  int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, nimble_gap_event_, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to start scanning: %d", rc);
    return;
  }

  this->scanning_ = true;
  ESP_LOGI(TAG, "BLE scanning started");
}

void BTHomeReceiverHub::stop_scanning_() {
  if (!this->scanning_) {
    return;
  }

  ble_gap_disc_cancel();
  this->scanning_ = false;
  ESP_LOGI(TAG, "BLE scanning stopped");
}

void BTHomeReceiverHub::process_nimble_advertisement(const struct ble_gap_disc_desc *disc) {
  // Convert address to uint64_t (little-endian)
  uint64_t address = 0;
  for (int i = 0; i < 6; i++) {
    address |= static_cast<uint64_t>(disc->addr.val[i]) << (i * 8);
  }

  // Parse advertisement data to find BTHome service data
  const uint8_t *data = disc->data;
  uint8_t data_len = disc->length_data;

  // Parse AD structures
  size_t pos = 0;
  while (pos < data_len) {
    if (pos + 1 > data_len) break;

    uint8_t len = data[pos];
    if (len == 0 || pos + 1 + len > data_len) break;

    uint8_t ad_type = data[pos + 1];
    const uint8_t *ad_data = &data[pos + 2];
    uint8_t ad_data_len = len - 1;

    // AD type 0x16 = Service Data - 16-bit UUID
    if (ad_type == 0x16 && ad_data_len >= 2) {
      // Extract 16-bit UUID (little-endian)
      uint16_t uuid = ad_data[0] | (ad_data[1] << 8);

      if (uuid == BTHOME_SERVICE_UUID) {
        // Found BTHome service data (excluding the 2-byte UUID prefix)
        const uint8_t *service_data = ad_data + 2;
        size_t service_data_len = ad_data_len - 2;

        // Cache for periodic dump
        if (this->dump_interval_ > 0) {
          this->cache_device_data_(address, service_data, service_data_len);
        }

        // Check if this device is registered
        if (this->find_device_(address) != nullptr) {
          std::vector<uint8_t> service_data_vec(service_data, service_data + service_data_len);
          ESP_LOGV(TAG, "Processing BTHome data from registered device %02X:%02X:%02X:%02X:%02X:%02X (%d bytes)",
                   (uint8_t)((address >> 40) & 0xFF), (uint8_t)((address >> 32) & 0xFF),
                   (uint8_t)((address >> 24) & 0xFF), (uint8_t)((address >> 16) & 0xFF),
                   (uint8_t)((address >> 8) & 0xFF), (uint8_t)(address & 0xFF),
                   (int)service_data_vec.size());
          this->process_advertisement_for_all_devices_(address, service_data_vec);
        }
        return;
      }
    }

    pos += 1 + len;
  }
}

#endif  // USE_BTHOME_RECEIVER_NIMBLE

// ============================================================================
// Bluedroid Implementation
// ============================================================================

#ifdef USE_BTHOME_RECEIVER_BLUEDROID

bool BTHomeReceiverHub::parse_device(const esphome::esp32_ble_tracker::ESPBTDevice &device) {
  // Check if this device has BTHome service data (UUID 0xFCD2)
  for (const auto &service_data : device.get_service_datas()) {
    if (service_data.uuid.get_uuid().uuid.uuid16 == BTHOME_SERVICE_UUID) {
      // Look up registered device by MAC address
      uint64_t address = device.address_uint64();

      // Cache for periodic dump
      if (this->dump_interval_ > 0) {
        this->cache_device_data_(address, service_data.data.data(), service_data.data.size());
      }

      if (this->find_device_(address) != nullptr) {
        ESP_LOGV(TAG, "Processing BTHome advertisement from %012llX", address);
        this->process_advertisement_for_all_devices_(address, service_data.data);
        return true;
      }
      return false;
    }
  }
  return false;
}

#endif  // USE_BTHOME_RECEIVER_BLUEDROID

// ============================================================================
// BTHomeDevice Implementation
// ============================================================================

void BTHomeDevice::set_encryption_key(const std::array<uint8_t, 16> &key) {
  this->encryption_enabled_ = true;
  this->encryption_key_ = key;
}

bool BTHomeDevice::parse_advertisement(const std::vector<uint8_t> &service_data) {
  if (service_data.size() < 1) {
    ESP_LOGW(TAG, "Invalid service data: too short");
    return false;
  }

  // Deduplicate: skip if this is an identical packet (devices often retransmit for reliability)
  if (service_data == this->last_service_data_) {
    ESP_LOGV(TAG, "Skipping duplicate packet");
    return true;  // Successfully handled (by ignoring)
  }
  this->last_service_data_ = service_data;

  // First byte is device_info
  uint8_t device_info = service_data[0];
  bool is_encrypted = (device_info & BTHOME_DEVICE_INFO_ENCRYPTED_MASK) != 0;

  ESP_LOGV(TAG, "Device info: 0x%02X, encrypted: %s", device_info, is_encrypted ? "yes" : "no");

  const uint8_t *payload_data;
  size_t payload_len;
  uint8_t decrypted_buffer[256];

  if (is_encrypted) {
    if (!this->encryption_enabled_) {
      ESP_LOGW(TAG, "Received encrypted data but no encryption key configured");
      return false;
    }

    // Encrypted format: device_info(1) + ciphertext + counter(4) + MIC(4)
    // The counter and MIC are at the end: [...ciphertext...][counter(4)][MIC(4)]
    if (service_data.size() < 9) {  // device_info(1) + min_ciphertext(0) + counter(4) + MIC(4)
      ESP_LOGW(TAG, "Encrypted data too short");
      return false;
    }

    // Extract counter from bytes [-8:-4] (4 bytes before the MIC)
    size_t counter_offset = service_data.size() - 8;
    uint32_t counter = service_data[counter_offset] | (service_data[counter_offset + 1] << 8) |
                       (service_data[counter_offset + 2] << 16) | (service_data[counter_offset + 3] << 24);

    ESP_LOGV(TAG, "Counter: %u, last counter: %u", counter, this->last_counter_);

    // Validate counter (replay protection)
    if (counter <= this->last_counter_) {
      ESP_LOGW(TAG, "Counter not increased (replay attack?): %u <= %u", counter, this->last_counter_);
      return false;
    }

    // Ciphertext is between device_info and counter
    const uint8_t *ciphertext = service_data.data() + 1;
    size_t ciphertext_len = service_data.size() - 1 - 4;  // Exclude device_info and counter+MIC

    // Get MAC address (6 bytes)
    uint8_t mac[6];
    for (int i = 0; i < 6; i++) {
      mac[i] = (this->address_ >> (i * 8)) & 0xFF;
    }

    size_t plaintext_len;
    if (!this->decrypt_payload_(ciphertext, ciphertext_len, mac, device_info, counter, decrypted_buffer,
                                 &plaintext_len)) {
      ESP_LOGW(TAG, "Decryption failed");
      return false;
    }

    // Update last counter after successful decryption
    this->last_counter_ = counter;

    payload_data = decrypted_buffer;
    payload_len = plaintext_len;
    ESP_LOGV(TAG, "Decrypted %d bytes", plaintext_len);
  } else {
    // Unencrypted: just skip device_info byte
    payload_data = service_data.data() + 1;
    payload_len = service_data.size() - 1;
  }

  // Parse measurements
  this->parse_measurements_(payload_data, payload_len);
  return true;
}

bool BTHomeDevice::decrypt_payload_(const uint8_t *ciphertext, size_t ciphertext_len, const uint8_t *mac,
                                     uint8_t device_info, uint32_t counter, uint8_t *plaintext,
                                     size_t *plaintext_len) {
  // BTHome v2 AES-CCM decryption
  // Nonce: MAC(6) + UUID(2, little-endian) + device_info(1) + counter(4) = 13 bytes
  uint8_t nonce[13];
  memcpy(nonce, mac, 6);
  nonce[6] = BTHOME_SERVICE_UUID & 0xFF;         // 0xD2
  nonce[7] = (BTHOME_SERVICE_UUID >> 8) & 0xFF;  // 0xFC
  nonce[8] = device_info;
  nonce[9] = counter & 0xFF;
  nonce[10] = (counter >> 8) & 0xFF;
  nonce[11] = (counter >> 16) & 0xFF;
  nonce[12] = (counter >> 24) & 0xFF;

  // The ciphertext_len includes the MIC (4 bytes)
  if (ciphertext_len < 4) {
    ESP_LOGE(TAG, "Ciphertext too short for MIC");
    return false;
  }

  size_t actual_ciphertext_len = ciphertext_len - 4;
  const uint8_t *mic = ciphertext + actual_ciphertext_len;

  // Prepare combined buffer for mbedtls (ciphertext + tag)
  uint8_t combined[256];
  if (ciphertext_len > sizeof(combined)) {
    ESP_LOGE(TAG, "Ciphertext too long");
    return false;
  }
  memcpy(combined, ciphertext, ciphertext_len);

  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);

  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, this->encryption_key_.data(), 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_setkey failed: %d", ret);
    mbedtls_ccm_free(&ctx);
    return false;
  }

  ret = mbedtls_ccm_auth_decrypt(&ctx, actual_ciphertext_len, nonce, sizeof(nonce), nullptr, 0, ciphertext, plaintext,
                                  mic, 4);
  mbedtls_ccm_free(&ctx);

  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_auth_decrypt failed: %d", ret);
    return false;
  }

  *plaintext_len = actual_ciphertext_len;
  return true;
}

void BTHomeDevice::parse_measurements_(const uint8_t *data, size_t len) {
  size_t pos = 0;

  // Track how many times we've seen each object_id (for indexed sensors like speed/gusts)
  // Using std::array instead of std::map for better embedded performance (no dynamic allocation)
  std::array<uint8_t, 256> object_id_counts{};  // Zero-initialized

  while (pos < len) {
    if (pos + 1 > len) {
      ESP_LOGW(TAG, "Incomplete measurement at offset %d", pos);
      break;
    }

    uint8_t object_id = data[pos++];
    ESP_LOGV(TAG, "Object ID: 0x%02X at offset %d", object_id, pos - 1);

    // Handle special types: button, dimmer, text, raw (before general indexing)
    if (object_id == OBJECT_ID_BUTTON) {
      // Button event: object_id(1) + event_type(1)
      // Button index is determined by sequence position (0 for first button, 1 for second, etc.)
      if (pos + 1 > len) {
        ESP_LOGW(TAG, "Incomplete button event");
        break;
      }
      // Get button index from sequence count (before incrementing)
      uint8_t button_index = object_id_counts[OBJECT_ID_BUTTON];
      uint8_t event_type = data[pos++];  // Full byte event type
      ESP_LOGV(TAG, "Button event: index=%d, type=0x%02X", button_index, event_type);
      this->handle_button_event_(button_index, event_type);
      // Increment count after using it
      object_id_counts[OBJECT_ID_BUTTON]++;
      continue;
    }

    // Get current index for this object_id (0 for first occurrence, 1 for second, etc.)
    uint8_t current_index = object_id_counts[object_id]++;  // Post-increment

    if (object_id == OBJECT_ID_DIMMER) {
      // Dimmer event: object_id(1) + steps(1, signed)
      if (pos + 1 > len) {
        ESP_LOGW(TAG, "Incomplete dimmer event");
        break;
      }
      int8_t steps = static_cast<int8_t>(data[pos++]);
      ESP_LOGV(TAG, "Dimmer event: steps=%d", steps);
      this->handle_dimmer_event_(steps);
      continue;
    }

    if (object_id == OBJECT_ID_TEXT) {
      // Text: object_id(1) + length(1) + UTF-8 string
      if (pos + 1 > len) {
        ESP_LOGW(TAG, "Incomplete text length");
        break;
      }
      uint8_t text_len = data[pos++];
      if (pos + text_len > len) {
        ESP_LOGW(TAG, "Incomplete text data");
        break;
      }
      std::string text(reinterpret_cast<const char *>(data + pos), text_len);
      pos += text_len;
      ESP_LOGV(TAG, "Text: '%s'", text.c_str());
      this->publish_text_value_(object_id, text);
      continue;
    }

    if (object_id == OBJECT_ID_RAW) {
      // Raw: object_id(1) + length(1) + raw bytes (display as hex)
      if (pos + 1 > len) {
        ESP_LOGW(TAG, "Incomplete raw length");
        break;
      }
      uint8_t raw_len = data[pos++];
      if (pos + raw_len > len) {
        ESP_LOGW(TAG, "Incomplete raw data");
        break;
      }
      // Convert to hex string
      std::string hex_str;
      for (uint8_t i = 0; i < raw_len; i++) {
        char hex[3];
        snprintf(hex, sizeof(hex), "%02X", data[pos + i]);
        if (i > 0)
          hex_str += " ";
        hex_str += hex;
      }
      pos += raw_len;
      ESP_LOGV(TAG, "Raw: %s", hex_str.c_str());
      this->publish_text_value_(object_id, hex_str);
      continue;
    }

    // Look up standard object type
    auto it = OBJECT_TYPE_MAP.find(object_id);
    if (it == OBJECT_TYPE_MAP.end()) {
      // Dump entire packet for debugging unknown object IDs
      std::string hex_dump;
      for (size_t i = 0; i < len; i++) {
        char hex[4];
        snprintf(hex, sizeof(hex), "%02X ", data[i]);
        hex_dump += hex;
      }
      ESP_LOGW(TAG, "Unknown object ID: 0x%02X at pos %d, full packet: %s", object_id, pos - 1, hex_dump.c_str());
      // Skip this measurement - we don't know its size, so we have to stop parsing
      break;
    }

    const ObjectTypeInfo &type_info = it->second;

    // Check if we have enough data
    if (pos + type_info.data_bytes > len) {
      ESP_LOGW(TAG, "Incomplete data for object 0x%02X (need %d bytes, have %d)", object_id, type_info.data_bytes,
               len - pos);
      break;
    }

    // Decode value based on type
    if (type_info.is_binary_sensor) {
      // Binary sensor: single byte, 0x00 or 0x01
      bool value = data[pos] != 0;
      pos += type_info.data_bytes;
      ESP_LOGV(TAG, "Binary sensor 0x%02X: %s", object_id, value ? "ON" : "OFF");
      this->publish_binary_sensor_value_(object_id, value);
    } else if (type_info.is_sensor) {
      // Numeric sensor: decode based on data_bytes and signedness
      int32_t raw_value = 0;

      if (type_info.is_signed) {
        // Signed integer (little-endian)
        switch (type_info.data_bytes) {
          case 1:
            raw_value = static_cast<int8_t>(data[pos]);
            break;
          case 2:
            raw_value = static_cast<int16_t>(data[pos] | (data[pos + 1] << 8));
            break;
          case 3:
            // sint24: sign-extend from 24 bits to 32 bits
            raw_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16);
            if (raw_value & 0x800000)
              raw_value |= 0xFF000000;  // Sign extend
            break;
          case 4:
            raw_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16) | (data[pos + 3] << 24);
            break;
        }
      } else {
        // Unsigned integer (little-endian)
        uint32_t unsigned_value = 0;
        switch (type_info.data_bytes) {
          case 1:
            unsigned_value = data[pos];
            break;
          case 2:
            unsigned_value = data[pos] | (data[pos + 1] << 8);
            break;
          case 3:
            unsigned_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16);
            break;
          case 4:
            unsigned_value = data[pos] | (data[pos + 1] << 8) | (data[pos + 2] << 16) | (data[pos + 3] << 24);
            break;
        }
        raw_value = unsigned_value;
      }

      pos += type_info.data_bytes;

      // Apply factor to convert to actual value
      float value = raw_value * type_info.factor;
      ESP_LOGV(TAG, "Sensor 0x%02X[%d]: raw=%d, value=%.3f", object_id, current_index, raw_value, value);
      this->publish_sensor_value_(object_id, current_index, value);
    }
  }
}

void BTHomeDevice::publish_sensor_value_(uint8_t object_id, uint8_t index, float value) {
#ifdef USE_SENSOR
  for (auto *sensor_obj : this->sensors_) {
    if (sensor_obj->get_object_id() == object_id && sensor_obj->get_index() == index) {
      sensor_obj->get_sensor()->publish_state(value);
      ESP_LOGD(TAG, "Published sensor value: object_id=0x%02X index=%d value=%.2f", object_id, index, value);
      return;
    }
  }
  ESP_LOGD(TAG, "No sensor registered for object ID 0x%02X index %d (have %zu sensors registered)", 
           object_id, index, this->sensors_.size());
  // Log registered sensors for debugging
  for (auto *sensor_obj : this->sensors_) {
    ESP_LOGD(TAG, "  Registered: object_id=0x%02X index=%d", 
             sensor_obj->get_object_id(), sensor_obj->get_index());
  }
#endif
}

void BTHomeDevice::publish_binary_sensor_value_(uint8_t object_id, bool value) {
#ifdef USE_BINARY_SENSOR
  for (auto *sensor_obj : this->binary_sensors_) {
    if (sensor_obj->get_object_id() == object_id) {
      sensor_obj->get_sensor()->publish_state(value);
      return;
    }
  }
#endif
  ESP_LOGV(TAG, "No binary sensor registered for object ID 0x%02X", object_id);
}

void BTHomeDevice::publish_text_value_(uint8_t object_id, const std::string &value) {
#ifdef USE_TEXT_SENSOR
  for (auto *sensor_obj : this->text_sensors_) {
    if (sensor_obj->get_object_id() == object_id) {
      sensor_obj->get_sensor()->publish_state(value);
      return;
    }
  }
#endif
  ESP_LOGV(TAG, "No text sensor registered for object ID 0x%02X", object_id);
}

void BTHomeDevice::handle_button_event_(uint8_t button_index, uint8_t event_type) {
  for (auto *trigger : this->button_triggers_) {
    if (trigger->get_button_index() == button_index && trigger->get_event_type() == event_type) {
      trigger->trigger();
    }
  }
}

void BTHomeDevice::handle_dimmer_event_(int8_t steps) {
  for (auto *trigger : this->dimmer_triggers_) {
    trigger->trigger(steps);
  }
}

}  // namespace bthome_receiver
}  // namespace esphome
