#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/automation.h"

// ESP-IDF timer for time tracking
#include <esp_timer.h>

// Platform-specific includes based on BLE stack
#ifdef USE_BTHOME_RECEIVER_NIMBLE
  // NimBLE stack (lighter weight, observer-only)
  #include "nimble/nimble_port.h"
  #include "nimble/nimble_port_freertos.h"
  #include "host/ble_hs.h"
  #include "host/util/util.h"
  #include <freertos/FreeRTOS.h>
  #include <freertos/semphr.h>
#elif defined(USE_BTHOME_RECEIVER_BLUEDROID)
  // Bluedroid stack (default, via esp32_ble_tracker)
  #include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#endif

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <vector>
#include <map>
#include <array>

namespace esphome {
namespace bthome_receiver {

// BTHome v2 constants
static const uint16_t BTHOME_SERVICE_UUID = 0xFCD2;

// Device info byte format: bit 0 = encryption, bit 2 = trigger-based
static const uint8_t BTHOME_DEVICE_INFO_ENCRYPTED_MASK = 0x01;

// Special object IDs for events and variable-length data
static const uint8_t OBJECT_ID_BUTTON = 0x3A;
static const uint8_t OBJECT_ID_DIMMER = 0x3C;
static const uint8_t OBJECT_ID_TEXT = 0x53;
static const uint8_t OBJECT_ID_RAW = 0x54;

// Button event types (BTHome v2 spec object ID 0x3A)
static const uint8_t BUTTON_EVENT_NONE = 0x00;
static const uint8_t BUTTON_EVENT_PRESS = 0x01;
static const uint8_t BUTTON_EVENT_DOUBLE_PRESS = 0x02;
static const uint8_t BUTTON_EVENT_TRIPLE_PRESS = 0x03;
static const uint8_t BUTTON_EVENT_LONG_PRESS = 0x04;
static const uint8_t BUTTON_EVENT_LONG_DOUBLE_PRESS = 0x05;
static const uint8_t BUTTON_EVENT_LONG_TRIPLE_PRESS = 0x06;
static const uint8_t BUTTON_EVENT_HOLD_PRESS = 0x80;

// Encryption constants
static const size_t AES_KEY_SIZE = 16;

// Object type info for parsing BTHome data
struct ObjectTypeInfo {
  uint8_t data_bytes;
  bool is_signed;
  float factor;
  bool is_sensor;
  bool is_binary_sensor;
};

// Forward declarations
class BTHomeReceiverHub;
class BTHomeDevice;

// =============================================================================
// BTHomeSensor - Represents a numeric sensor value from a BTHome device
// Supports indexing for multiple sensors of the same object_id (e.g., speed + gusts)
// =============================================================================
#ifdef USE_SENSOR
class BTHomeSensor {
 public:
  BTHomeSensor(uint8_t object_id, uint8_t index, sensor::Sensor *sensor)
      : object_id_(object_id), index_(index), sensor_(sensor) {}

  uint8_t get_object_id() const { return this->object_id_; }
  uint8_t get_index() const { return this->index_; }
  sensor::Sensor *get_sensor() { return this->sensor_; }

 protected:
  uint8_t object_id_;
  uint8_t index_;  // For multiple sensors of same type (0=first, 1=second, etc.)
  sensor::Sensor *sensor_;
};
#endif

// =============================================================================
// BTHomeBinarySensor - Represents a boolean sensor value from a BTHome device
// =============================================================================
#ifdef USE_BINARY_SENSOR
class BTHomeBinarySensor {
 public:
  BTHomeBinarySensor(uint8_t object_id, binary_sensor::BinarySensor *sensor)
      : object_id_(object_id), sensor_(sensor) {}

  uint8_t get_object_id() const { return this->object_id_; }
  binary_sensor::BinarySensor *get_sensor() { return this->sensor_; }

 protected:
  uint8_t object_id_;
  binary_sensor::BinarySensor *sensor_;
};
#endif

// =============================================================================
// BTHomeTextSensor - Represents text/raw data from a BTHome device
// =============================================================================
#ifdef USE_TEXT_SENSOR
class BTHomeTextSensor {
 public:
  BTHomeTextSensor(uint8_t object_id, text_sensor::TextSensor *sensor)
      : object_id_(object_id), sensor_(sensor) {}

  uint8_t get_object_id() const { return this->object_id_; }
  text_sensor::TextSensor *get_sensor() { return this->sensor_; }

 protected:
  uint8_t object_id_;
  text_sensor::TextSensor *sensor_;
};
#endif

// =============================================================================
// BTHomeButtonTrigger - Automation trigger for button events
// =============================================================================
class BTHomeButtonTrigger : public Trigger<>, public Parented<BTHomeDevice> {
 public:
  explicit BTHomeButtonTrigger(BTHomeDevice *parent) : Parented(parent) {}

  void set_button_index(uint8_t index) { this->button_index_ = index; }
  void set_event_type(uint8_t event_type) { this->event_type_ = event_type; }

  uint8_t get_button_index() const { return this->button_index_; }
  uint8_t get_event_type() const { return this->event_type_; }

 protected:
  uint8_t button_index_{0};
  uint8_t event_type_{BUTTON_EVENT_PRESS};
};

// =============================================================================
// BTHomeDimmerTrigger - Automation trigger for dimmer rotation events
// =============================================================================
class BTHomeDimmerTrigger : public Trigger<int8_t>, public Parented<BTHomeDevice> {
 public:
  explicit BTHomeDimmerTrigger(BTHomeDevice *parent) : Parented(parent) {}
};

// =============================================================================
// BTHomeDevice - Represents a single BTHome BLE device being monitored
// =============================================================================
class BTHomeDevice : public Parented<BTHomeReceiverHub> {
 public:
  explicit BTHomeDevice(BTHomeReceiverHub *parent) : Parented(parent) {}

  void set_mac_address(uint64_t mac) { this->address_ = mac; }
  void set_name(const std::string &name) { this->name_ = name; }
  void set_encryption_key(const std::array<uint8_t, AES_KEY_SIZE> &key);

  uint64_t get_mac_address() const { return this->address_; }
  const std::string &get_name() const { return this->name_; }

  // Parse incoming BLE advertisement
  bool parse_advertisement(const std::vector<uint8_t> &service_data);

#ifdef USE_SENSOR
  void add_sensor(uint8_t object_id, uint8_t index, sensor::Sensor *sensor) {
    this->sensors_.push_back(new BTHomeSensor(object_id, index, sensor));
  }
#endif

#ifdef USE_BINARY_SENSOR
  void add_binary_sensor(uint8_t object_id, binary_sensor::BinarySensor *sensor) {
    this->binary_sensors_.push_back(new BTHomeBinarySensor(object_id, sensor));
  }
#endif

#ifdef USE_TEXT_SENSOR
  void add_text_sensor(uint8_t object_id, text_sensor::TextSensor *sensor) {
    this->text_sensors_.push_back(new BTHomeTextSensor(object_id, sensor));
  }
#endif

  void add_button_trigger(BTHomeButtonTrigger *trigger) { this->button_triggers_.push_back(trigger); }
  void add_dimmer_trigger(BTHomeDimmerTrigger *trigger) { this->dimmer_triggers_.push_back(trigger); }

 protected:
  // Decrypt encrypted payload using AES-128-CCM
  bool decrypt_payload_(const uint8_t *ciphertext, size_t ciphertext_len, const uint8_t *mac,
                        uint8_t device_info, uint32_t counter, uint8_t *plaintext, size_t *plaintext_len);

  // Parse measurement objects from payload
  void parse_measurements_(const uint8_t *data, size_t len);

  // Publish values to registered sensors
  void publish_sensor_value_(uint8_t object_id, uint8_t index, float value);
  void publish_binary_sensor_value_(uint8_t object_id, bool value);
  void publish_text_value_(uint8_t object_id, const std::string &value);

  // Handle events
  void handle_button_event_(uint8_t button_index, uint8_t event_type);
  void handle_dimmer_event_(int8_t steps);

  uint64_t address_{0};
  std::string name_;

  // Encryption
  bool encryption_enabled_{false};
  std::array<uint8_t, AES_KEY_SIZE> encryption_key_{};
  uint32_t last_counter_{0};

  // Deduplication - store last received service data to skip duplicate packets
  std::vector<uint8_t> last_service_data_;

  // Sensors
#ifdef USE_SENSOR
  std::vector<BTHomeSensor *> sensors_;
#endif
#ifdef USE_BINARY_SENSOR
  std::vector<BTHomeBinarySensor *> binary_sensors_;
#endif
#ifdef USE_TEXT_SENSOR
  std::vector<BTHomeTextSensor *> text_sensors_;
#endif

  // Event triggers
  std::vector<BTHomeButtonTrigger *> button_triggers_;
  std::vector<BTHomeDimmerTrigger *> dimmer_triggers_;
};

// =============================================================================
// BTHomeReceiverHub - Main component that receives BLE advertisements
// =============================================================================
#ifdef USE_BTHOME_RECEIVER_BLUEDROID
class BTHomeReceiverHub : public Component, public esphome::esp32_ble_tracker::ESPBTDeviceListener {
#else
class BTHomeReceiverHub : public Component {
#endif
 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Register a device to monitor
  void register_device(BTHomeDevice *device);

  // Set interval for periodic dump of all detected devices (in ms, 0 = disabled)
  void set_dump_interval(uint32_t interval) { this->dump_interval_ = interval; }

#ifdef USE_BTHOME_RECEIVER_BLUEDROID
  // ESPBTDeviceListener interface - called when BLE advertisement is received
  bool parse_device(const esphome::esp32_ble_tracker::ESPBTDevice &device) override;
#endif

#ifdef USE_BTHOME_RECEIVER_NIMBLE
  // Process advertisement received via NimBLE
  void process_nimble_advertisement(const struct ble_gap_disc_desc *disc);
#endif

 protected:
  // Device registry - using vector for small dataset optimization (typically <10 devices)
  std::vector<BTHomeDevice *> devices_;

  // Periodic dump interval (ms, 0 = disabled)
  uint32_t dump_interval_{0};
  uint32_t last_dump_time_{0};

  // Cache of detected BTHome devices for periodic dump
  // Stores: MAC address -> (last_data, last_seen_time)
  struct DetectedDevice {
    std::vector<uint8_t> last_data;
    uint32_t last_seen{0};
  };
  std::vector<std::pair<uint64_t, DetectedDevice>> detected_devices_;

  // Dump an advertisement to the log (for discovery mode)
  void dump_advertisement_(uint64_t address, const uint8_t *data, size_t len);

  // Find a device by MAC address (linear search, efficient for small datasets)
  BTHomeDevice *find_device_(uint64_t address);

  // Process advertisement for all devices with matching MAC address
  void process_advertisement_for_all_devices_(uint64_t address, const std::vector<uint8_t> &service_data);

  // Cache device data for periodic dump
  void cache_device_data_(uint64_t address, const uint8_t *data, size_t len);

  // Dump all cached devices (for periodic summary)
  void dump_all_devices_();

#ifdef USE_BTHOME_RECEIVER_NIMBLE
  // NimBLE-specific members
  bool nimble_initialized_{false};
  bool init_attempted_{false};
  bool scanning_{false};
  static BTHomeReceiverHub *instance_;  // For NimBLE callbacks
  static void nimble_host_task_(void *param);
  static void nimble_on_sync_();
  static void nimble_on_reset_(int reason);
  static int nimble_gap_event_(struct ble_gap_event *event, void *arg);
  void init_nimble_();
  void start_scanning_();
  void stop_scanning_();
#endif
};

}  // namespace bthome_receiver
}  // namespace esphome
