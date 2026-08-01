#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <array>

// Platform-specific includes
#ifdef USE_ESP32
  #ifdef USE_BTHOME_NIMBLE
    // NimBLE stack (lighter weight, broadcast-only)
    #include "esp_nimble_hci.h"
    #include "nimble/nimble_port.h"
    #include "nimble/nimble_port_freertos.h"
    #include "host/ble_hs.h"
    #include "host/util/util.h"
    #include <esp_bt.h>
  #else
    // Bluedroid stack (default)
    #include "esphome/components/esp32_ble/ble.h"
    #ifndef CONFIG_ESP_HOSTED_ENABLE_BT_BLUEDROID
      #include <esp_bt.h>
    #endif
    #include <esp_gap_ble_api.h>
  #endif
#endif  // USE_ESP32

#ifdef USE_NRF52
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#endif  // USE_NRF52

#if defined(USE_ESP32) || defined(USE_NRF52)

namespace esphome {
namespace bthome {

// BTHome v2 constants
static const uint16_t BTHOME_SERVICE_UUID = 0xFCD2;
// Device info byte format: bit 0 = encryption, bit 2 = trigger-based
static const uint8_t BTHOME_DEVICE_INFO_UNENCRYPTED = 0x40;           // Regular device, no encryption
static const uint8_t BTHOME_DEVICE_INFO_ENCRYPTED = 0x41;             // Regular device, encrypted
static const uint8_t BTHOME_DEVICE_INFO_TRIGGER_UNENCRYPTED = 0x44;   // Trigger-based device, no encryption
static const uint8_t BTHOME_DEVICE_INFO_TRIGGER_ENCRYPTED = 0x45;     // Trigger-based device, encrypted
static const size_t MAX_BLE_ADVERTISEMENT_SIZE = 31;
static const size_t MAX_DEVICE_NAME_LENGTH = 20;  // Leave room for other AD elements

#ifdef USE_SENSOR
struct SensorMeasurement {
  sensor::Sensor *sensor;
  uint8_t object_id;
  uint8_t data_bytes;      // Number of bytes to encode (1, 2, 3, or 4)
  bool is_signed;          // True for signed integers, false for unsigned
  float factor;            // Multiply raw value by this to get encoded value
  bool advertise_immediately;
};
#endif

#ifdef USE_BINARY_SENSOR
struct BinarySensorMeasurement {
  binary_sensor::BinarySensor *sensor;
  uint8_t object_id;
  bool advertise_immediately;
};
#endif

#if defined(USE_ESP32) && defined(USE_BTHOME_BLUEDROID)
using namespace esp32_ble;

class BTHome : public Component, public GAPEventHandler, public Parented<ESP32BLE> {
#else
class BTHome : public Component {
#endif
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override;

  void set_min_interval(uint16_t val) { this->min_interval_ = val; }
  void set_max_interval(uint16_t val) { this->max_interval_ = val; }
  void set_retransmit_count(uint8_t count) { this->retransmit_count_ = count; }
  void set_retransmit_interval(uint16_t interval_ms) { this->retransmit_interval_ = interval_ms; }

#ifdef USE_ESP32
  void set_tx_power(int val) { this->tx_power_esp32_ = static_cast<esp_power_level_t>(val); }
#endif
#ifdef USE_NRF52
  void set_tx_power(int8_t val) { this->tx_power_nrf52_ = val; }
#endif

  void set_device_name(const std::string &name);
  void set_manufacturer_id(uint16_t id) { this->manufacturer_id_ = id; this->has_manufacturer_id_ = true; }
  void set_trigger_based(bool trigger_based) { this->trigger_based_ = trigger_based; }

  void set_encryption_key(const std::array<uint8_t, 16> &key);
#ifdef USE_SENSOR
  void add_measurement(sensor::Sensor *sensor, uint8_t object_id, uint8_t data_bytes,
                       bool is_signed, float factor, bool advertise_immediately);
#endif
#ifdef USE_BINARY_SENSOR
  void add_binary_measurement(binary_sensor::BinarySensor *sensor, uint8_t object_id, bool advertise_immediately);
#endif

#if defined(USE_ESP32) && defined(USE_BTHOME_BLUEDROID)
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;
#endif

 protected:
  void build_advertisement_data_();
  void build_scan_response_data_();
  void start_advertising_();
  void stop_advertising_();
#ifdef USE_SENSOR
  size_t encode_measurement_(uint8_t *data, size_t max_len, const SensorMeasurement &measurement);
#endif
#ifdef USE_BINARY_SENSOR
  size_t encode_binary_measurement_(uint8_t *data, size_t max_len, uint8_t object_id, bool value);
#endif
  bool encrypt_payload_(const uint8_t *plaintext, size_t plaintext_len, uint8_t *ciphertext, size_t *ciphertext_len);
  void trigger_immediate_advertising_(uint8_t measurement_index, bool is_binary);

  // Measurements storage
#ifdef USE_SENSOR
  StaticVector<SensorMeasurement, BTHOME_MAX_MEASUREMENTS> measurements_;
#endif
#ifdef USE_BINARY_SENSOR
  StaticVector<BinarySensorMeasurement, BTHOME_MAX_BINARY_MEASUREMENTS> binary_measurements_;
#endif

  // Common settings
  uint16_t min_interval_{1000};
  uint16_t max_interval_{1000};
  bool advertising_{false};

  // Retransmission settings (for reliability, devices often send same packet multiple times)
  uint8_t retransmit_count_{0};       // Number of retransmissions (0 = disabled)
  uint16_t retransmit_interval_{500}; // Interval between retransmissions in ms
  uint8_t retransmit_remaining_{0};   // Remaining retransmissions for current packet
  uint32_t last_retransmit_time_{0};  // Last retransmission time in ms

  // Device identification
  std::string device_name_;
  uint16_t manufacturer_id_{0x02E5};  // Default: Espressif (0x02E5)
  bool has_manufacturer_id_{true};
  bool trigger_based_{false};  // True for devices that only send on events (buttons, etc.)

  // Encryption
  bool encryption_enabled_{false};
  std::array<uint8_t, 16> encryption_key_{};
  uint32_t counter_{0};

  // Packet ID for deduplication (increments only when data changes, not on retransmits)
  uint8_t packet_id_{0};

  // Advertisement data
  uint8_t adv_data_[MAX_BLE_ADVERTISEMENT_SIZE];
  size_t adv_data_len_{0};
  bool data_changed_{true};

  // Measurement rotation (for splitting across multiple packets)
  size_t current_sensor_index_{0};
  size_t current_binary_index_{0};

  // Scan response data (device name + manufacturer)
  uint8_t scan_rsp_data_[MAX_BLE_ADVERTISEMENT_SIZE];
  size_t scan_rsp_data_len_{0};

  // Immediate advertising
  bool immediate_advertising_pending_{false};
  uint8_t immediate_adv_measurement_index_{0};
  bool immediate_adv_is_binary_{false};

  // Platform-specific members
#ifdef USE_ESP32
  esp_power_level_t tx_power_esp32_{};

  #ifdef USE_BTHOME_NIMBLE
    // NimBLE-specific members
    uint8_t nimble_own_addr_type_{0};
    bool nimble_initialized_{false};
    static BTHome *instance_;  // For NimBLE callbacks
    static void nimble_host_task_(void *param);
    static void nimble_on_sync_();
    static void nimble_on_reset_(int reason);
  #else
    // Bluedroid-specific members
    esp_ble_adv_params_t ble_adv_params_;
    bool adv_data_set_{false};
    bool scan_rsp_data_set_{false};
  #endif
#endif

#ifdef USE_NRF52
  int8_t tx_power_nrf52_{0};
  struct bt_le_adv_param adv_param_;
  struct bt_data ad_[2];
  struct bt_data sd_[5];  // Scan response data (service UUID, TX power, appearance, name, manufacturer)
#endif
};

}  // namespace bthome
}  // namespace esphome

#endif  // USE_ESP32 || USE_NRF52
