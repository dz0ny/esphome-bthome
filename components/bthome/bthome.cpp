#include "bthome.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

#if defined(USE_ESP32) || defined(USE_NRF52)

#include <algorithm>
#include <cstring>
#include <cmath>

// Platform-specific includes
#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
    #include "esp_nimble_hci.h"
    #include "nimble/nimble_port.h"
    #include "nimble/nimble_port_freertos.h"
    #include "host/ble_hs.h"
    #include "host/util/util.h"
    #include <esp_bt.h>
    #include <nvs_flash.h>
    // NimBLE uses tinycrypt for encryption
    #include "tinycrypt/ccm_mode.h"
    #include "tinycrypt/constants.h"
  #else
    #include <esp_bt_device.h>
    #include <esp_bt_main.h>
    #include <esp_gap_ble_api.h>
    #include "esphome/core/hal.h"
    #include "mbedtls/ccm.h"
  #endif
#endif

#ifdef USE_NRF52
#include <zephyr/kernel.h>
#include <tinycrypt/ccm_mode.h>
#include <tinycrypt/constants.h>
#endif

namespace esphome {
namespace bthome {

static const char *const TAG = "bthome";

#if defined(USE_ESP32) && defined(USE_BTHOME_NIMBLE)
// Static instance for NimBLE callbacks
BTHome *BTHome::instance_ = nullptr;
#endif

void BTHome::dump_config() {
  ESP_LOGCONFIG(TAG,
                "BTHome:\n"
                "  Min Interval: %ums\n"
                "  Max Interval: %ums\n"
                "  TX Power: %ddBm\n"
                "  Encryption: %s\n"
                "  Retransmit: %dx @ %ums\n"
#if defined(USE_ESP32) && defined(USE_BTHOME_NIMBLE)
                "  BLE Stack: NimBLE",
#elif defined(USE_ESP32)
                "  BLE Stack: Bluedroid",
#else
                "  BLE Stack: Zephyr",
#endif
                this->min_interval_, this->max_interval_,
#ifdef USE_ESP32
                (this->tx_power_esp32_ * 3) - 12,
#else
                this->tx_power_nrf52_,
#endif
                this->encryption_enabled_ ? "enabled" : "disabled",
                this->retransmit_count_, this->retransmit_interval_);
  if (!this->device_name_.empty()) {
    ESP_LOGCONFIG(TAG, "  Device Name: %s", this->device_name_.c_str());
  }
  if (this->has_manufacturer_id_) {
    ESP_LOGCONFIG(TAG, "  Manufacturer ID: 0x%04X", this->manufacturer_id_);
  }
  if (this->trigger_based_) {
    ESP_LOGCONFIG(TAG, "  Trigger-based: yes");
  }
#ifdef USE_SENSOR
  ESP_LOGCONFIG(TAG, "  Sensors: %d", this->measurements_.size());
#endif
#ifdef USE_BINARY_SENSOR
  ESP_LOGCONFIG(TAG, "  Binary Sensors: %d", this->binary_measurements_.size());
#endif
}

float BTHome::get_setup_priority() const {
#ifdef USE_ESP32
  return setup_priority::AFTER_BLUETOOTH;
#else
  return setup_priority::BLUETOOTH;
#endif
}

void BTHome::setup() {
  ESP_LOGD(TAG, "Setting up BTHome...");

#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
  // NimBLE stack initialization
  instance_ = this;

  // Initialize NVS (required by NimBLE)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
    this->mark_failed();
    return;
  }

  // Initialize NimBLE
  ret = nimble_port_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NimBLE port init failed: %s", esp_err_to_name(ret));
    this->mark_failed();
    return;
  }

  // Configure NimBLE host callbacks
  // Note: Device name is included directly in advertisement data (build_advertisement_data_)
  ble_hs_cfg.sync_cb = nimble_on_sync_;
  ble_hs_cfg.reset_cb = nimble_on_reset_;

  // Start NimBLE host task
  nimble_port_freertos_init(nimble_host_task_);

  ESP_LOGD(TAG, "NimBLE initialized, waiting for sync...");
  this->nimble_initialized_ = true;

  #else
  // Bluedroid stack initialization
  this->ble_adv_params_ = {
      .adv_int_min = static_cast<uint16_t>(this->min_interval_ / 0.625f),
      .adv_int_max = static_cast<uint16_t>(this->max_interval_ / 0.625f),
      .adv_type = ADV_TYPE_NONCONN_IND,  // Non-connectable, non-scannable
      .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
      .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
      .channel_map = ADV_CHNL_ALL,
      .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
  };

  global_ble->advertising_register_raw_advertisement_callback([this](bool advertise) {
    this->advertising_ = advertise;
    if (advertise) {
      this->build_advertisement_data_();
      this->build_scan_response_data_();
      this->start_advertising_();
    }
  });
  #endif
#endif

#ifdef USE_NRF52
  // nRF52: Initialize Bluetooth
  int err = bt_enable(nullptr);
  if (err) {
    ESP_LOGE(TAG, "Bluetooth init failed (err %d)", err);
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Bluetooth initialized");

  // Set up advertising parameters
  this->adv_param_ = BT_LE_ADV_PARAM_INIT(
      BT_LE_ADV_OPT_USE_IDENTITY,
      BT_GAP_ADV_FAST_INT_MIN_2,
      BT_GAP_ADV_FAST_INT_MAX_2,
      nullptr
  );
  this->adv_param_.interval_min = this->min_interval_ * 1000 / 625;
  this->adv_param_.interval_max = this->max_interval_ * 1000 / 625;
#endif

  // Register callbacks for sensor state changes
#ifdef USE_SENSOR
  for (size_t i = 0; i < this->measurements_.size(); i++) {
    auto &measurement = this->measurements_[i];
    measurement.sensor->add_on_state_callback([this, i](float) {
      if (this->measurements_[i].advertise_immediately) {
        this->trigger_immediate_advertising_(i, false);
      } else {
        this->data_changed_ = true;
#ifdef USE_ESP32
        this->enable_loop();
#endif
      }
    });
  }
#endif

#ifdef USE_BINARY_SENSOR
  for (size_t i = 0; i < this->binary_measurements_.size(); i++) {
    auto &measurement = this->binary_measurements_[i];
    measurement.sensor->add_on_state_callback([this, i](bool) {
      if (this->binary_measurements_[i].advertise_immediately) {
        this->trigger_immediate_advertising_(i, true);
      } else {
        this->data_changed_ = true;
#ifdef USE_ESP32
        this->enable_loop();
#endif
      }
    });
  }
#endif

#ifdef USE_NRF52
  // nRF52: Build and start advertising immediately
  this->build_advertisement_data_();
  this->build_scan_response_data_();
  this->start_advertising_();
#endif

#ifdef USE_ESP32
  // ESP32: Disable loop initially - only enable for immediate advertising
  this->disable_loop();
#endif
}

void BTHome::loop() {
  uint32_t now = esp_timer_get_time() / 1000;  // Convert microseconds to milliseconds

  // Handle retransmissions
  if (this->retransmit_remaining_ > 0 && this->advertising_) {
    if (now - this->last_retransmit_time_ >= this->retransmit_interval_) {
      ESP_LOGD(TAG, "Retransmitting (%d remaining)", this->retransmit_remaining_);
      this->retransmit_remaining_--;
      this->last_retransmit_time_ = now;

      // Stop and restart advertising to force a new broadcast
      this->stop_advertising_();
      this->start_advertising_();

#ifdef USE_ESP32
      // Keep loop enabled while retransmissions pending
      if (this->retransmit_remaining_ == 0) {
        this->disable_loop();
      }
#endif
    }
    return;
  }

  // Handle immediate advertising requests
  if (this->immediate_advertising_pending_) {
    this->immediate_advertising_pending_ = false;
    this->stop_advertising_();
    this->build_advertisement_data_();
    this->start_advertising_();

    // Start retransmission cycle if configured
    if (this->retransmit_count_ > 0) {
      this->retransmit_remaining_ = this->retransmit_count_;
      this->last_retransmit_time_ = now;
      // Keep loop enabled for retransmissions
    } else {
#ifdef USE_ESP32
      this->disable_loop();
#endif
    }
    return;
  }

  // Handle regular data changes
  if (this->data_changed_ && this->advertising_) {
    this->data_changed_ = false;
    this->stop_advertising_();
    this->build_advertisement_data_();
    this->start_advertising_();

    // Start retransmission cycle if configured
    if (this->retransmit_count_ > 0) {
      this->retransmit_remaining_ = this->retransmit_count_;
      this->last_retransmit_time_ = now;
      // Keep loop enabled for retransmissions
    } else {
#ifdef USE_ESP32
      this->disable_loop();
#endif
    }
  }
}

void BTHome::set_encryption_key(const std::array<uint8_t, 16> &key) {
  this->encryption_enabled_ = true;
  this->encryption_key_ = key;
}

void BTHome::set_device_name(const std::string &name) {
  if (name.length() > MAX_DEVICE_NAME_LENGTH) {
    this->device_name_ = name.substr(0, MAX_DEVICE_NAME_LENGTH);
    ESP_LOGW(TAG, "Device name truncated to %d characters", MAX_DEVICE_NAME_LENGTH);
  } else {
    this->device_name_ = name;
  }
}

#ifdef USE_SENSOR
void BTHome::add_measurement(sensor::Sensor *sensor, uint8_t object_id, uint8_t data_bytes,
                              bool is_signed, float factor, bool advertise_immediately) {
  this->measurements_.push_back({sensor, object_id, data_bytes, is_signed, factor, advertise_immediately});
}
#endif

#ifdef USE_BINARY_SENSOR
void BTHome::add_binary_measurement(binary_sensor::BinarySensor *sensor, uint8_t object_id, bool advertise_immediately) {
  this->binary_measurements_.push_back({sensor, object_id, advertise_immediately});
}
#endif

void BTHome::trigger_immediate_advertising_(uint8_t measurement_index, bool is_binary) {
  this->immediate_advertising_pending_ = true;
  this->immediate_adv_measurement_index_ = measurement_index;
  this->immediate_adv_is_binary_ = is_binary;
#ifdef USE_ESP32
  this->enable_loop();
#endif
}

void BTHome::build_advertisement_data_() {
  size_t pos = 0;

  // Flags AD element
  this->adv_data_[pos++] = 0x02;  // Length
  this->adv_data_[pos++] = 0x01;  // Type: Flags
  this->adv_data_[pos++] = 0x06;  // LE General Discoverable, BR/EDR not supported

  // Service Data AD element
  size_t service_data_len_pos = pos;
  pos++;  // Length placeholder
  this->adv_data_[pos++] = 0x16;  // Type: Service Data

  // BTHome Service UUID (little-endian)
  this->adv_data_[pos++] = BTHOME_SERVICE_UUID & 0xFF;
  this->adv_data_[pos++] = (BTHOME_SERVICE_UUID >> 8) & 0xFF;

  // Device info byte: combines encryption (bit 0) and trigger-based (bit 2) flags
  uint8_t device_info;
  if (this->trigger_based_) {
    device_info = this->encryption_enabled_ ? BTHOME_DEVICE_INFO_TRIGGER_ENCRYPTED : BTHOME_DEVICE_INFO_TRIGGER_UNENCRYPTED;
  } else {
    device_info = this->encryption_enabled_ ? BTHOME_DEVICE_INFO_ENCRYPTED : BTHOME_DEVICE_INFO_UNENCRYPTED;
  }
  this->adv_data_[pos++] = device_info;

  size_t measurement_start = pos;

  // Packet ID (object 0x00) - helps receivers deduplicate retransmissions
  // Only incremented when build_advertisement_data_() is called (new data)
  // Retransmissions reuse the same advertisement data without rebuilding
  this->adv_data_[pos++] = 0x00;  // Object ID: packet_id
  this->adv_data_[pos++] = this->packet_id_;

  // Handle immediate advertising - single sensor only
  if (this->immediate_advertising_pending_) {
#ifdef USE_BINARY_SENSOR
    if (this->immediate_adv_is_binary_) {
      auto &measurement = this->binary_measurements_[this->immediate_adv_measurement_index_];
      if (measurement.sensor->has_state()) {
        pos += this->encode_binary_measurement_(this->adv_data_ + pos, MAX_BLE_ADVERTISEMENT_SIZE - pos,
                                                 measurement.object_id, measurement.sensor->state);
      }
    }
#endif
#ifdef USE_SENSOR
    if (!this->immediate_adv_is_binary_) {
      auto &measurement = this->measurements_[this->immediate_adv_measurement_index_];
      if (measurement.sensor->has_state() && !std::isnan(measurement.sensor->state)) {
        pos += this->encode_measurement_(this->adv_data_ + pos, MAX_BLE_ADVERTISEMENT_SIZE - pos, measurement);
      }
    }
#endif
  } else {
#if defined(USE_SENSOR) || defined(USE_BINARY_SENSOR)
    // Normal path. Two independent concerns are handled in order:
    //   1. Selection (rotation): decide *which* measurements go in this packet. When all
    //      measurements don't fit in a single 31-byte advertisement they are split across
    //      successive packets; a single cursor spanning sensors and binary sensors combined keeps
    //      the rotation fair so neither category can starve the other.
    //   2. Write order: BTHome requires object ids to be non-decreasing within a payload, so the
    //      selected measurements are sorted by object_id before being encoded. Sensor and binary
    //      sensor object id ranges overlap, which is why they must be ordered together, not
    //      sequentially by category.
    struct Entry {
      uint8_t object_id;
      uint8_t size;   // encoded size in bytes: 1 + data_bytes for sensors, 2 for binary sensors
      bool is_binary;
      uint16_t idx;   // index into measurements_ / binary_measurements_
      bool valid;     // sensor currently has a (non-NaN) state
    };
    Entry entries[BTHOME_MAX_MEASUREMENTS + BTHOME_MAX_BINARY_MEASUREMENTS];
    size_t n = 0;

#ifdef USE_SENSOR
    for (size_t i = 0; i < this->measurements_.size(); i++) {
      const auto &m = this->measurements_[i];
      bool valid = m.sensor->has_state() && !std::isnan(m.sensor->state);
      entries[n++] = {m.object_id, (uint8_t) (1 + m.data_bytes), false, (uint16_t) i, valid};
    }
#endif
#ifdef USE_BINARY_SENSOR
    for (size_t i = 0; i < this->binary_measurements_.size(); i++) {
      const auto &m = this->binary_measurements_[i];
      entries[n++] = {m.object_id, 2, true, (uint16_t) i, m.sensor->has_state()};
    }
#endif

    if (n > 0) {
      // Rotate from the current cursor, selecting entries that still fit in this packet.
      size_t selected[BTHOME_MAX_MEASUREMENTS + BTHOME_MAX_BINARY_MEASUREMENTS];
      size_t selected_count = 0;
      size_t probe_pos = pos;
      size_t start_idx = this->current_index_ % n;

      for (size_t i = 0; i < n; i++) {
        size_t idx = (start_idx + i) % n;
        if (!entries[idx].valid)
          continue;
        if (probe_pos + entries[idx].size > MAX_BLE_ADVERTISEMENT_SIZE)
          break;
        probe_pos += entries[idx].size;
        selected[selected_count++] = idx;
      }

      // Advance the cursor so the next packet continues where this one left off. Only meaningful
      // when the payload was split (some measurements didn't fit); if everything fit, leave it.
      if (selected_count > 0 && selected_count < n) {
        this->current_index_ = (start_idx + selected_count) % n;
      }

      // Emit selected entries in ascending object_id order (stable: equal ids keep insertion order).
      std::stable_sort(selected, selected + selected_count,
                       [&entries](size_t a, size_t b) { return entries[a].object_id < entries[b].object_id; });

      for (size_t i = 0; i < selected_count; i++) {
        const Entry &e = entries[selected[i]];
        if (e.is_binary) {
#ifdef USE_BINARY_SENSOR
          const auto &m = this->binary_measurements_[e.idx];
          pos += this->encode_binary_measurement_(this->adv_data_ + pos, MAX_BLE_ADVERTISEMENT_SIZE - pos,
                                                   m.object_id, m.sensor->state);
#endif
        } else {
#ifdef USE_SENSOR
          pos += this->encode_measurement_(this->adv_data_ + pos, MAX_BLE_ADVERTISEMENT_SIZE - pos,
                                            this->measurements_[e.idx]);
#endif
        }
      }
    }
#endif  // USE_SENSOR || USE_BINARY_SENSOR
  }

  size_t measurement_len = pos - measurement_start;

  // Handle encryption
  if (this->encryption_enabled_ && measurement_len > 0) {
    uint8_t plaintext[MAX_BLE_ADVERTISEMENT_SIZE];
    memcpy(plaintext, this->adv_data_ + measurement_start, measurement_len);

    uint8_t ciphertext[MAX_BLE_ADVERTISEMENT_SIZE];
    size_t ciphertext_len = 0;

    if (this->encrypt_payload_(plaintext, measurement_len, ciphertext, &ciphertext_len)) {
      memcpy(this->adv_data_ + measurement_start, ciphertext, ciphertext_len);
      pos = measurement_start + ciphertext_len;

      // Add counter (4 bytes, little-endian)
      this->adv_data_[pos++] = this->counter_ & 0xFF;
      this->adv_data_[pos++] = (this->counter_ >> 8) & 0xFF;
      this->adv_data_[pos++] = (this->counter_ >> 16) & 0xFF;
      this->adv_data_[pos++] = (this->counter_ >> 24) & 0xFF;

      this->counter_++;
    }
  }

  // Set service data length
  this->adv_data_[service_data_len_pos] = pos - service_data_len_pos - 1;

  // Note: Device name is in scan response, not advertisement (to save space for sensor data)

  this->adv_data_len_ = pos;

  // Increment packet_id for next data change (wraps at 255)
  this->packet_id_++;

  ESP_LOGD(TAG, "Built advertisement data (%zu bytes, packet_id=%u)", this->adv_data_len_, (uint8_t)(this->packet_id_ - 1));
  ESP_LOGD(TAG, "  Measurement rotation index: %zu", this->current_index_);
}

void BTHome::build_scan_response_data_() {
  // Scan response is limited to 31 bytes
  // We include: TX Power (3), Manufacturer Data (8), Name (remaining ~20)
  size_t pos = 0;

  // Add TX Power Level (3 bytes) for distance estimation
  this->scan_rsp_data_[pos++] = 2;     // Length
  this->scan_rsp_data_[pos++] = 0x0A;  // Type: TX Power Level
#ifdef USE_ESP32
  // Convert ESP32 power level enum to dBm: level * 3 - 12
  int8_t tx_power_dbm = static_cast<int8_t>(this->tx_power_esp32_) * 3 - 12;
#elif defined(USE_NRF52)
  int8_t tx_power_dbm = this->tx_power_nrf52_;
#else
  int8_t tx_power_dbm = 0;
#endif
  this->scan_rsp_data_[pos++] = static_cast<uint8_t>(tx_power_dbm);

  // Add Manufacturer Specific Data with ESPHome version (8 bytes)
  if (this->has_manufacturer_id_) {
    this->scan_rsp_data_[pos++] = 7;     // Length
    this->scan_rsp_data_[pos++] = 0xFF;  // Type: Manufacturer Specific Data
    this->scan_rsp_data_[pos++] = this->manufacturer_id_ & 0xFF;
    this->scan_rsp_data_[pos++] = (this->manufacturer_id_ >> 8) & 0xFF;
    uint32_t version = ESPHOME_VERSION_CODE;
    this->scan_rsp_data_[pos++] = version & 0xFF;
    this->scan_rsp_data_[pos++] = (version >> 8) & 0xFF;
    this->scan_rsp_data_[pos++] = (version >> 16) & 0xFF;
    this->scan_rsp_data_[pos++] = (version >> 24) & 0xFF;
  }

  // Add device name, clipped to fit remaining space (31 - 11 = 20 bytes max)
  if (!this->device_name_.empty()) {
    size_t remaining = MAX_BLE_ADVERTISEMENT_SIZE - pos;
    size_t max_name_len = remaining > 2 ? remaining - 2 : 0;
    size_t name_len = std::min(this->device_name_.length(), max_name_len);
    if (name_len > 0) {
      this->scan_rsp_data_[pos++] = name_len + 1;
      // Shortened Local Name (0x08) if truncated, Complete (0x09) otherwise
      this->scan_rsp_data_[pos++] = (name_len < this->device_name_.length()) ? 0x08 : 0x09;
      memcpy(this->scan_rsp_data_ + pos, this->device_name_.c_str(), name_len);
      pos += name_len;
    }
  }

  this->scan_rsp_data_len_ = pos;
  ESP_LOGD(TAG, "Built scan response data (%zu bytes)", this->scan_rsp_data_len_);
}

void BTHome::start_advertising_() {
#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
  // NimBLE advertising
  if (!this->nimble_initialized_) {
    ESP_LOGW(TAG, "NimBLE not initialized yet");
    return;
  }

  // Set raw advertisement data
  int rc = ble_gap_adv_set_data(this->adv_data_, this->adv_data_len_);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_set_data failed: %d", rc);
    return;
  }

  // Set scan response data (device name + ESPHome version)
  if (this->scan_rsp_data_len_ > 0) {
    rc = ble_gap_adv_rsp_set_data(this->scan_rsp_data_, this->scan_rsp_data_len_);
    if (rc != 0) {
      ESP_LOGW(TAG, "ble_gap_adv_rsp_set_data failed: %d", rc);
    }
  }

  // Configure non-connectable advertising
  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;  // Non-connectable
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;  // General discoverable
  adv_params.itvl_min = static_cast<uint16_t>(this->min_interval_ / 0.625f);
  adv_params.itvl_max = static_cast<uint16_t>(this->max_interval_ / 0.625f);

  ESP_LOGD(TAG, "Starting NimBLE advertising (%zu bytes, scan_rsp %zu bytes)",
           this->adv_data_len_, this->scan_rsp_data_len_);
  rc = ble_gap_adv_start(this->nimble_own_addr_type_, nullptr, BLE_HS_FOREVER,
                         &adv_params, nullptr, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
    return;
  }

  this->advertising_ = true;
  ESP_LOGD(TAG, "NimBLE advertising started");

  #else
  // Bluedroid advertising
  // Reset synchronization flags
  this->adv_data_set_ = false;
  this->scan_rsp_data_set_ = false;

  ESP_LOGD(TAG, "Setting BLE TX power");
  esp_err_t err = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, this->tx_power_esp32_);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_tx_power_set failed: %s", esp_err_to_name(err));
  }

  ESP_LOGD(TAG, "Setting advertisement data (%zu bytes)", this->adv_data_len_);
  err = esp_ble_gap_config_adv_data_raw(this->adv_data_, this->adv_data_len_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_config_adv_data_raw failed: %s", esp_err_to_name(err));
    return;
  }

  // Set scan response data (contains service UUID and device name)
  if (this->scan_rsp_data_len_ > 0) {
    ESP_LOGD(TAG, "Setting scan response data (%zu bytes)", this->scan_rsp_data_len_);
    err = esp_ble_gap_config_scan_rsp_data_raw(this->scan_rsp_data_, this->scan_rsp_data_len_);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "esp_ble_gap_config_scan_rsp_data_raw failed: %s", esp_err_to_name(err));
    }
  }

  // Start advertising directly (don't wait for GAP events)
  ESP_LOGD(TAG, "Starting advertising directly");
  err = esp_ble_gap_start_advertising(&this->ble_adv_params_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %s", esp_err_to_name(err));
  }
  #endif
#endif

#ifdef USE_NRF52
  static uint8_t flags_data[] = {BT_LE_AD_NO_BREDR | BT_LE_AD_GENERAL};

  this->ad_[0].type = BT_DATA_FLAGS;
  this->ad_[0].data_len = sizeof(flags_data);
  this->ad_[0].data = flags_data;

  // Service data (skip flags we already added)
  this->ad_[1].type = BT_DATA_SVC_DATA16;
  this->ad_[1].data_len = this->adv_data_len_ - 3;  // Skip flags
  this->ad_[1].data = this->adv_data_ + 4;          // Skip flags + length + type

  // Set up scan response data
  size_t sd_count = 0;

  // Add BTHome service UUID to scan response
  static uint8_t svc_uuid_data[] = {BTHOME_SERVICE_UUID & 0xFF, (BTHOME_SERVICE_UUID >> 8) & 0xFF};
  this->sd_[sd_count].type = BT_DATA_UUID16_ALL;
  this->sd_[sd_count].data_len = sizeof(svc_uuid_data);
  this->sd_[sd_count].data = svc_uuid_data;
  sd_count++;

  // Add TX Power Level
  static int8_t tx_power_data;
  tx_power_data = this->tx_power_nrf52_;
  this->sd_[sd_count].type = BT_DATA_TX_POWER;
  this->sd_[sd_count].data_len = sizeof(tx_power_data);
  this->sd_[sd_count].data = reinterpret_cast<const uint8_t *>(&tx_power_data);
  sd_count++;

  // Add Appearance (Generic Sensor = 0x0540)
  static uint8_t appearance_data[] = {0x40, 0x05};  // Little-endian 0x0540
  this->sd_[sd_count].type = BT_DATA_GAP_APPEARANCE;
  this->sd_[sd_count].data_len = sizeof(appearance_data);
  this->sd_[sd_count].data = appearance_data;
  sd_count++;

  if (!this->device_name_.empty()) {
    this->sd_[sd_count].type = BT_DATA_NAME_COMPLETE;
    this->sd_[sd_count].data_len = this->device_name_.length();
    this->sd_[sd_count].data = reinterpret_cast<const uint8_t *>(this->device_name_.c_str());
    sd_count++;
  }

  if (this->has_manufacturer_id_) {
    // Manufacturer ID (2 bytes) + ESPHome version code (4 bytes)
    static uint8_t mfr_data[6];
    mfr_data[0] = this->manufacturer_id_ & 0xFF;
    mfr_data[1] = (this->manufacturer_id_ >> 8) & 0xFF;
    uint32_t version = ESPHOME_VERSION_CODE;
    mfr_data[2] = version & 0xFF;
    mfr_data[3] = (version >> 8) & 0xFF;
    mfr_data[4] = (version >> 16) & 0xFF;
    mfr_data[5] = (version >> 24) & 0xFF;
    this->sd_[sd_count].type = BT_DATA_MANUFACTURER_DATA;
    this->sd_[sd_count].data_len = sizeof(mfr_data);
    this->sd_[sd_count].data = mfr_data;
    sd_count++;
  }

  int err = bt_le_adv_start(&this->adv_param_, this->ad_, 2,
                            sd_count > 0 ? this->sd_ : nullptr, sd_count);
  if (err) {
    ESP_LOGE(TAG, "Advertising failed to start (err %d)", err);
    return;
  }

  this->advertising_ = true;
  ESP_LOGD(TAG, "BTHome advertising started");
#endif
}

void BTHome::stop_advertising_() {
#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
  if (this->advertising_) {
    ble_gap_adv_stop();
    this->advertising_ = false;
  }
  #else
  if (this->advertising_) {
    esp_ble_gap_stop_advertising();
  }
  #endif
#endif

#ifdef USE_NRF52
  if (this->advertising_) {
    bt_le_adv_stop();
    this->advertising_ = false;
  }
#endif
}

#if defined(USE_ESP32) && defined(USE_BTHOME_NIMBLE)
// NimBLE static callbacks
void BTHome::nimble_host_task_(void *param) {
  ESP_LOGD(TAG, "NimBLE host task started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

void BTHome::nimble_on_sync_() {
  ESP_LOGD(TAG, "NimBLE host synced");

  // Determine address type
  int rc = ble_hs_id_infer_auto(0, &instance_->nimble_own_addr_type_);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to infer address type: %d", rc);
    return;
  }

  // Build and start advertising
  instance_->build_advertisement_data_();
  instance_->build_scan_response_data_();
  instance_->start_advertising_();
}

void BTHome::nimble_on_reset_(int reason) {
  ESP_LOGW(TAG, "NimBLE host reset, reason: %d", reason);
  instance_->advertising_ = false;
}
#endif

#if defined(USE_ESP32) && defined(USE_BTHOME_BLUEDROID)
void BTHome::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  // GAP events are handled by ESPHome's BLE component
  // We start advertising directly in start_advertising_()
}
#endif

#ifdef USE_SENSOR
size_t BTHome::encode_measurement_(uint8_t *data, size_t max_len, const SensorMeasurement &measurement) {
  // Generic BTHome v2 sensor encoding
  // Uses data_bytes, is_signed, and factor from the measurement struct
  // See: https://bthome.io/format/

  size_t required_size = 1 + measurement.data_bytes;  // object_id + value bytes
  if (max_len < required_size) {
    return 0;
  }

  float value = measurement.sensor->state;
  size_t pos = 0;

  // Object ID
  data[pos++] = measurement.object_id;

  // Convert value to encoded integer using factor
  // Factor in Python is the resolution (e.g., 0.01 means value * 100)
  // So we divide by factor to get the encoded value
  double scaled = std::round(value / measurement.factor);

  // Encode based on data_bytes and signedness (little-endian)
  if (measurement.is_signed) {
    // Signed integer encoding
    int32_t encoded;
    switch (measurement.data_bytes) {
      case 1:
        encoded = static_cast<int8_t>(std::max(-128.0, std::min(127.0, scaled)));
        data[pos++] = encoded & 0xFF;
        break;
      case 2:
        encoded = static_cast<int16_t>(std::max(-32768.0, std::min(32767.0, scaled)));
        data[pos++] = encoded & 0xFF;
        data[pos++] = (encoded >> 8) & 0xFF;
        break;
      case 3:
        // sint24 - range: -8388608 to 8388607
        encoded = static_cast<int32_t>(std::max(-8388608.0, std::min(8388607.0, scaled)));
        data[pos++] = encoded & 0xFF;
        data[pos++] = (encoded >> 8) & 0xFF;
        data[pos++] = (encoded >> 16) & 0xFF;
        break;
      case 4:
        encoded = static_cast<int32_t>(scaled);
        data[pos++] = encoded & 0xFF;
        data[pos++] = (encoded >> 8) & 0xFF;
        data[pos++] = (encoded >> 16) & 0xFF;
        data[pos++] = (encoded >> 24) & 0xFF;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported data_bytes: %d for object 0x%02X", measurement.data_bytes, measurement.object_id);
        return 0;
    }
  } else {
    // Unsigned integer encoding
    uint32_t encoded;
    switch (measurement.data_bytes) {
      case 1:
        encoded = static_cast<uint8_t>(std::max(0.0, std::min(255.0, scaled)));
        data[pos++] = encoded & 0xFF;
        break;
      case 2:
        encoded = static_cast<uint16_t>(std::max(0.0, std::min(65535.0, scaled)));
        data[pos++] = encoded & 0xFF;
        data[pos++] = (encoded >> 8) & 0xFF;
        break;
      case 3:
        // uint24 - range: 0 to 16777215
        encoded = static_cast<uint32_t>(std::max(0.0, std::min(16777215.0, scaled)));
        data[pos++] = encoded & 0xFF;
        data[pos++] = (encoded >> 8) & 0xFF;
        data[pos++] = (encoded >> 16) & 0xFF;
        break;
      case 4:
        encoded = static_cast<uint32_t>(std::max(0.0, scaled));
        data[pos++] = encoded & 0xFF;
        data[pos++] = (encoded >> 8) & 0xFF;
        data[pos++] = (encoded >> 16) & 0xFF;
        data[pos++] = (encoded >> 24) & 0xFF;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported data_bytes: %d for object 0x%02X", measurement.data_bytes, measurement.object_id);
        return 0;
    }
  }

  return pos;
}
#endif

#ifdef USE_BINARY_SENSOR
size_t BTHome::encode_binary_measurement_(uint8_t *data, size_t max_len, uint8_t object_id, bool value) {
  // Binary sensors are always encoded as: [object_id] [0x00 or 0x01]
  // See: https://bthome.io/format/
  //
  // BTHome v2 binary sensor object IDs:
  //   0x0F - generic_boolean (generic on/off)
  //   0x10 - power (power on/off)
  //   0x11 - opening (open/closed)
  //   0x15 - battery_low (battery normal/low)
  //   0x16 - battery_charging (not charging/charging)
  //   0x17 - carbon_monoxide (CO not detected/detected)
  //   0x18 - cold (normal/cold)
  //   0x19 - connectivity (disconnected/connected)
  //   0x1A - door (closed/open)
  //   0x1B - garage_door (closed/open)
  //   0x1C - gas (clear/detected)
  //   0x1D - heat (normal/hot)
  //   0x1E - light (no light/light detected)
  //   0x1F - lock (locked/unlocked)
  //   0x20 - moisture_binary (dry/wet)
  //   0x21 - motion (clear/detected)
  //   0x22 - moving (not moving/moving)
  //   0x23 - occupancy (clear/detected)
  //   0x24 - plug (unplugged/plugged in)
  //   0x25 - presence (away/home)
  //   0x26 - problem (ok/problem)
  //   0x27 - running (not running/running)
  //   0x28 - safety (unsafe/safe)
  //   0x29 - smoke (clear/detected)
  //   0x2A - sound (clear/detected)
  //   0x2B - tamper (off/on)
  //   0x2C - vibration (clear/detected)
  //   0x2D - window (closed/open)

  if (max_len < 2) return 0;

  data[0] = object_id;
  data[1] = value ? 0x01 : 0x00;
  return 2;
}
#endif

bool BTHome::encrypt_payload_(const uint8_t *plaintext, size_t plaintext_len, uint8_t *ciphertext, size_t *ciphertext_len) {
  if (!this->encryption_enabled_) return false;

  // Build nonce: MAC (6) + UUID (2) + device info (1) + counter (4) = 13 bytes
  uint8_t nonce[13];

#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
  // NimBLE: Get MAC address from controller
  uint8_t mac[6];
  int rc = ble_hs_id_copy_addr(this->nimble_own_addr_type_, mac, nullptr);
  if (rc != 0) {
    ESP_LOGE(TAG, "Failed to get NimBLE MAC address: %d", rc);
    return false;
  }
  memcpy(nonce, mac, 6);
  #else
  // Bluedroid: Get MAC address
  const uint8_t *mac = esp_bt_dev_get_address();
  memcpy(nonce, mac, 6);
  #endif
#endif

#ifdef USE_NRF52
  bt_addr_le_t addr;
  size_t count = 1;
  bt_id_get(&addr, &count);
  memcpy(nonce, addr.a.val, 6);
#endif

  nonce[6] = BTHOME_SERVICE_UUID & 0xFF;
  nonce[7] = (BTHOME_SERVICE_UUID >> 8) & 0xFF;
  nonce[8] = this->trigger_based_ ? BTHOME_DEVICE_INFO_TRIGGER_ENCRYPTED : BTHOME_DEVICE_INFO_ENCRYPTED;
  nonce[9] = this->counter_ & 0xFF;
  nonce[10] = (this->counter_ >> 8) & 0xFF;
  nonce[11] = (this->counter_ >> 16) & 0xFF;
  nonce[12] = (this->counter_ >> 24) & 0xFF;

#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
  // NimBLE: Use tinycrypt for encryption (smaller footprint)
  struct tc_ccm_mode_struct ctx;
  struct tc_aes_key_sched_struct sched;

  if (tc_aes128_set_encrypt_key(&sched, this->encryption_key_.data()) != TC_CRYPTO_SUCCESS) {
    ESP_LOGE(TAG, "Failed to set AES key");
    return false;
  }

  if (tc_ccm_config(&ctx, &sched, nonce, sizeof(nonce), 4) != TC_CRYPTO_SUCCESS) {
    ESP_LOGE(TAG, "Failed to configure CCM");
    return false;
  }

  if (tc_ccm_generation_encryption(ciphertext, plaintext_len + 4, nullptr, 0,
                                    plaintext, plaintext_len, &ctx) != TC_CRYPTO_SUCCESS) {
    ESP_LOGE(TAG, "CCM encryption failed");
    return false;
  }
  #else
  // Bluedroid: Use mbedtls for encryption
  mbedtls_ccm_context ctx;
  mbedtls_ccm_init(&ctx);

  int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, this->encryption_key_.data(), 128);
  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_setkey failed: %d", ret);
    mbedtls_ccm_free(&ctx);
    return false;
  }

  ret = mbedtls_ccm_encrypt_and_tag(&ctx, plaintext_len, nonce, sizeof(nonce), nullptr, 0,
                                     plaintext, ciphertext, ciphertext + plaintext_len, 4);
  mbedtls_ccm_free(&ctx);

  if (ret != 0) {
    ESP_LOGE(TAG, "mbedtls_ccm_encrypt_and_tag failed: %d", ret);
    return false;
  }
  #endif
#endif

#ifdef USE_NRF52
  struct tc_ccm_mode_struct ctx;
  struct tc_aes_key_sched_struct sched;

  if (tc_aes128_set_encrypt_key(&sched, this->encryption_key_.data()) != TC_CRYPTO_SUCCESS) {
    ESP_LOGE(TAG, "Failed to set AES key");
    return false;
  }

  if (tc_ccm_config(&ctx, &sched, nonce, sizeof(nonce), 4) != TC_CRYPTO_SUCCESS) {
    ESP_LOGE(TAG, "Failed to configure CCM");
    return false;
  }

  if (tc_ccm_generation_encryption(ciphertext, plaintext_len + 4, nullptr, 0,
                                    plaintext, plaintext_len, &ctx) != TC_CRYPTO_SUCCESS) {
    ESP_LOGE(TAG, "CCM encryption failed");
    return false;
  }
#endif

  *ciphertext_len = plaintext_len + 4;
  return true;
}

}  // namespace bthome
}  // namespace esphome

#endif  // USE_ESP32 || USE_NRF52
