"""
BTHome v2 BLE Protocol Component for ESPHome

BTHome is an open standard for broadcasting sensor data over Bluetooth Low Energy.
It is natively supported by Home Assistant and other home automation platforms.

Protocol specification: https://bthome.io/format/
UUID 0xFCD2 sponsored by Allterco Robotics (Shelly)

Supports ESP32 (ESP-IDF) and nRF52 (Zephyr) platforms.
"""

import esphome.codegen as cg
from esphome.components import binary_sensor, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_BINARY_SENSORS,
    CONF_ID,
    CONF_SENSORS,
    CONF_TX_POWER,
    CONF_TYPE,
)
from esphome.core import CORE, TimePeriod

CODEOWNERS = ["@esphome/core"]

# Dependencies ensure USE_SENSOR and USE_BINARY_SENSOR are defined
# These are optional - only loaded if used in the config
DEPENDENCIES = []

# Auto-load these components when bthome is used
AUTO_LOAD = []

# BLE stack options for ESP32
CONF_BLE_STACK = "ble_stack"
BLE_STACK_BLUEDROID = "bluedroid"
BLE_STACK_NIMBLE = "nimble"

bthome_ns = cg.esphome_ns.namespace("bthome")
BTHome = bthome_ns.class_("BTHome", cg.Component)

# Configuration constants
CONF_ENCRYPTION_KEY = "encryption_key"
CONF_MIN_INTERVAL = "min_interval"
CONF_MAX_INTERVAL = "max_interval"
CONF_ADVERTISE_IMMEDIATELY = "advertise_immediately"
CONF_TRIGGER_BASED = "trigger_based"
CONF_RETRANSMIT_COUNT = "retransmit_count"
CONF_RETRANSMIT_INTERVAL = "retransmit_interval"

# =============================================================================
# BTHome v2 Sensor Object IDs
# See: https://bthome.io/format/
#
# Format: "type_name": (object_id, data_bytes, signed, factor)
#   - object_id: BTHome object identifier
#   - data_bytes: number of bytes (1, 2, 3, or 4)
#   - signed: True for signed integers, False for unsigned
#   - factor: multiply raw value by this to get actual value
# =============================================================================
SENSOR_TYPES = {
    # Basic sensors
    "packet_id": (0x00, 1, False, 1),           # uint8, used for deduplication
    "battery": (0x01, 1, False, 1),             # uint8, 1%
    "temperature": (0x02, 2, True, 0.01),       # sint16, 0.01°C
    "humidity": (0x03, 2, False, 0.01),         # uint16, 0.01%
    "pressure": (0x04, 3, False, 0.01),         # uint24, 0.01 hPa
    "illuminance": (0x05, 3, False, 0.01),      # uint24, 0.01 lux
    "mass_kg": (0x06, 2, False, 0.01),          # uint16, 0.01 kg
    "mass_lb": (0x07, 2, False, 0.01),          # uint16, 0.01 lb
    "dewpoint": (0x08, 2, True, 0.01),          # sint16, 0.01°C
    "count_uint8": (0x09, 1, False, 1),         # uint8
    "energy": (0x0A, 3, False, 0.001),          # uint24, 0.001 kWh
    "power": (0x0B, 3, False, 0.01),            # uint24, 0.01 W
    "voltage": (0x0C, 2, False, 0.001),         # uint16, 0.001 V
    "pm2_5": (0x0D, 2, False, 1),               # uint16, 1 µg/m³
    "pm10": (0x0E, 2, False, 1),                # uint16, 1 µg/m³
    "co2": (0x12, 2, False, 1),                 # uint16, 1 ppm
    "tvoc": (0x13, 2, False, 1),                # uint16, 1 µg/m³
    "moisture": (0x14, 2, False, 0.01),         # uint16, 0.01%
    "humidity_uint8": (0x2E, 1, False, 1),      # uint8, 1%
    "moisture_uint8": (0x2F, 1, False, 1),      # uint8, 1%

    # Extended sensors
    "count_uint16": (0x3D, 2, False, 1),        # uint16
    "count_uint32": (0x3E, 4, False, 1),        # uint32
    "rotation": (0x3F, 2, True, 0.1),           # sint16, 0.1°
    "distance_mm": (0x40, 2, False, 1),         # uint16, 1 mm
    "distance_m": (0x41, 2, False, 0.1),        # uint16, 0.1 m
    "duration": (0x42, 3, False, 0.001),        # uint24, 0.001 s
    "current": (0x43, 2, False, 0.001),         # uint16, 0.001 A
    "speed": (0x44, 2, False, 0.01),            # uint16, 0.01 m/s
    "temperature_01": (0x45, 2, True, 0.1),     # sint16, 0.1°C
    "uv_index": (0x46, 1, False, 0.1),          # uint8, 0.1
    "volume_l_01": (0x47, 2, False, 0.1),       # uint16, 0.1 L
    "volume_ml": (0x48, 2, False, 1),           # uint16, 1 mL
    "volume_flow_rate": (0x49, 2, False, 0.001),  # uint16, 0.001 m³/hr
    "voltage_01": (0x4A, 2, False, 0.1),        # uint16, 0.1 V
    "gas": (0x4B, 3, False, 0.001),             # uint24, 0.001 m³
    "gas_uint32": (0x4C, 4, False, 0.001),      # uint32, 0.001 m³
    "energy_uint32": (0x4D, 4, False, 0.001),   # uint32, 0.001 kWh
    "volume_l": (0x4E, 4, False, 0.001),        # uint32, 0.001 L
    "water": (0x4F, 4, False, 0.001),           # uint32, 0.001 L
    "timestamp": (0x50, 4, False, 1),           # uint32, seconds since epoch
    "acceleration": (0x51, 2, False, 0.001),    # uint16, 0.001 m/s²
    "gyroscope": (0x52, 2, False, 0.001),       # uint16, 0.001 °/s
    "volume_storage": (0x55, 4, False, 0.001),  # uint32, 0.001 L
    "conductivity": (0x56, 2, False, 1),        # uint16, 1 µS/cm
    "temperature_sint8": (0x57, 1, True, 1),    # sint8, 1°C
    "temperature_sint8_035": (0x58, 1, True, 0.35),  # sint8, 0.35°C
    "count_sint8": (0x59, 1, True, 1),          # sint8
    "count_sint16": (0x5A, 2, True, 1),         # sint16
    "count_sint32": (0x5B, 4, True, 1),         # sint32
    "power_sint32": (0x5C, 4, True, 0.01),      # sint32, 0.01 W
    "current_sint16": (0x5D, 2, True, 0.001),   # sint16, 0.001 A
    "direction": (0x5E, 2, False, 0.01),        # uint16, 0.01°
    "precipitation": (0x5F, 2, False, 0.1),     # uint16, 0.1 mm
    "channel": (0x60, 1, False, 1),             # uint8
    "rotational_speed": (0x61, 2, False, 1),    # uint16, 1 rpm
}

# =============================================================================
# BTHome v2 Binary Sensor Object IDs
# See: https://bthome.io/format/
#
# All binary sensors are uint8: 0x00 = off/false, 0x01 = on/true
# =============================================================================
BINARY_SENSOR_TYPES = {
    "generic_boolean": 0x0F,    # generic on/off
    "power": 0x10,              # power on/off
    "opening": 0x11,            # open/closed
    "battery_low": 0x15,        # battery normal/low
    "battery_charging": 0x16,   # not charging/charging
    "carbon_monoxide": 0x17,    # CO not detected/detected
    "cold": 0x18,               # normal/cold
    "connectivity": 0x19,       # disconnected/connected
    "door": 0x1A,               # closed/open
    "garage_door": 0x1B,        # closed/open
    "gas": 0x1C,                # clear/detected
    "heat": 0x1D,               # normal/hot
    "light": 0x1E,              # no light/light detected
    "lock": 0x1F,               # locked/unlocked
    "moisture_binary": 0x20,    # dry/wet
    "motion": 0x21,             # clear/detected
    "moving": 0x22,             # not moving/moving
    "occupancy": 0x23,          # clear/detected
    "plug": 0x24,               # unplugged/plugged in
    "presence": 0x25,           # away/home
    "problem": 0x26,            # ok/problem
    "running": 0x27,            # not running/running
    "safety": 0x28,             # unsafe/safe
    "smoke": 0x29,              # clear/detected
    "sound": 0x2A,              # clear/detected
    "tamper": 0x2B,             # off/on
    "vibration": 0x2C,          # clear/detected
    "window": 0x2D,             # closed/open
}

# TX Power levels for ESP32 (maps dBm to esp_power_level_t enum value)
ESP32_TX_POWER_LEVELS = {
    -12: 0, -9: 1, -6: 2, -3: 3, 0: 4, 3: 5, 6: 6, 9: 7,
}

# TX Power levels for nRF52 (direct dBm values)
NRF52_TX_POWER_LEVELS = {
    -40: -40, -20: -20, -16: -16, -12: -12, -8: -8, -4: -4,
    0: 0, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8,
}


def get_tx_power_levels():
    if CORE.is_esp32:
        return ESP32_TX_POWER_LEVELS
    elif CORE.is_nrf52:
        return NRF52_TX_POWER_LEVELS
    return {0: 0}


def validate_tx_power(value):
    levels = get_tx_power_levels()
    if isinstance(value, str) and value.endswith("dBm"):
        value = int(value[:-3])
    value = int(value)
    if value not in levels:
        raise cv.Invalid(f"TX power {value} dBm not supported. Valid: {list(levels.keys())}")
    return levels[value]


def validate_config(config):
    if config[CONF_MIN_INTERVAL] > config.get(CONF_MAX_INTERVAL):
        raise cv.Invalid("min_interval must be <= max_interval")
    return config


def validate_encryption_key(value):
    """Validate 16-byte (32 hex char) AES encryption key."""
    value = cv.string_strict(value)
    value = value.replace(" ", "").replace("-", "")
    if len(value) != 32:
        raise cv.Invalid("Encryption key must be 32 hexadecimal characters (16 bytes)")
    try:
        int(value, 16)
    except ValueError as e:
        raise cv.Invalid("Encryption key must be valid hexadecimal") from e
    return value.lower()


def _final_validate(config):
    if not CORE.is_esp32 and not CORE.is_nrf52:
        raise cv.Invalid("BTHome only supports ESP32 and nRF52 platforms")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BTHome),
            cv.Optional(CONF_TRIGGER_BASED, default=False): cv.boolean,
            cv.Optional(CONF_BLE_STACK, default=BLE_STACK_BLUEDROID): cv.one_of(
                BLE_STACK_BLUEDROID, BLE_STACK_NIMBLE, lower=True
            ),
            cv.Optional(CONF_MIN_INTERVAL, default="1s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=1000), max=TimePeriod(milliseconds=10240)),
            ),
            cv.Optional(CONF_MAX_INTERVAL, default="1s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=1000), max=TimePeriod(milliseconds=10240)),
            ),
            cv.Optional(CONF_TX_POWER, default=0): validate_tx_power,
            cv.Optional(CONF_ENCRYPTION_KEY): validate_encryption_key,
            cv.Optional(CONF_RETRANSMIT_COUNT, default=0): cv.int_range(min=0, max=10),
            cv.Optional(CONF_RETRANSMIT_INTERVAL, default="500ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=TimePeriod(milliseconds=100), max=TimePeriod(milliseconds=2000)),
            ),
            cv.Optional(CONF_SENSORS): cv.ensure_list(
                cv.Schema(
                    {
                        cv.Required(CONF_TYPE): cv.one_of(*SENSOR_TYPES.keys(), lower=True),
                        cv.Required(CONF_ID): cv.use_id(sensor.Sensor),
                        cv.Optional(CONF_ADVERTISE_IMMEDIATELY, default=False): cv.boolean,
                    }
                )
            ),
            cv.Optional(CONF_BINARY_SENSORS): cv.ensure_list(
                cv.Schema(
                    {
                        cv.Required(CONF_TYPE): cv.one_of(*BINARY_SENSOR_TYPES.keys(), lower=True),
                        cv.Required(CONF_ID): cv.use_id(binary_sensor.BinarySensor),
                        cv.Optional(CONF_ADVERTISE_IMMEDIATELY, default=False): cv.boolean,
                    }
                )
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_config,
)

FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    # Calculate sizes for StaticVector compile-time allocation
    num_sensors = max(1, len(config.get(CONF_SENSORS, [])))
    num_binary_sensors = max(1, len(config.get(CONF_BINARY_SENSORS, [])))
    max_packets = max(1, num_sensors + num_binary_sensors)

    # Add defines for compile-time sizes
    cg.add_define("BTHOME_MAX_MEASUREMENTS", num_sensors)
    cg.add_define("BTHOME_MAX_BINARY_MEASUREMENTS", num_binary_sensors)
    cg.add_define("BTHOME_MAX_ADV_PACKETS", max_packets)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_min_interval(config[CONF_MIN_INTERVAL]))
    cg.add(var.set_max_interval(config[CONF_MAX_INTERVAL]))
    cg.add(var.set_tx_power(config[CONF_TX_POWER]))
    cg.add(var.set_retransmit_count(config[CONF_RETRANSMIT_COUNT]))
    cg.add(var.set_retransmit_interval(config[CONF_RETRANSMIT_INTERVAL]))

    # Always use ESPHome device name
    if CORE.name:
        cg.add(var.set_device_name(CORE.name[:20]))

    if config[CONF_TRIGGER_BASED]:
        cg.add(var.set_trigger_based(True))

    if CONF_ENCRYPTION_KEY in config:
        key = config[CONF_ENCRYPTION_KEY]
        key_bytes = [cg.RawExpression(f"0x{key[i:i+2]}") for i in range(0, len(key), 2)]
        key_array = cg.RawExpression(f"std::array<uint8_t, 16>{{{', '.join(str(b) for b in key_bytes)}}}")
        cg.add(var.set_encryption_key(key_array))

    # Add sensor measurements
    if CONF_SENSORS in config:
        for measurement in config[CONF_SENSORS]:
            sensor_type = measurement[CONF_TYPE]
            type_info = SENSOR_TYPES[sensor_type]
            object_id = type_info[0]
            data_bytes = type_info[1]
            is_signed = type_info[2]
            factor = type_info[3]
            sens = await cg.get_variable(measurement[CONF_ID])
            advertise_immediately = measurement[CONF_ADVERTISE_IMMEDIATELY]
            cg.add(var.add_measurement(sens, object_id, data_bytes, is_signed, factor, advertise_immediately))

    # Add binary sensor measurements
    if CONF_BINARY_SENSORS in config:
        for measurement in config[CONF_BINARY_SENSORS]:
            sensor_type = measurement[CONF_TYPE]
            object_id = BINARY_SENSOR_TYPES[sensor_type]
            sens = await cg.get_variable(measurement[CONF_ID])
            advertise_immediately = measurement[CONF_ADVERTISE_IMMEDIATELY]
            cg.add(var.add_binary_measurement(sens, object_id, advertise_immediately))

    # Platform-specific setup
    if CORE.is_esp32:
        from esphome.components.esp32 import add_idf_sdkconfig_option

        ble_stack = config.get(CONF_BLE_STACK, BLE_STACK_BLUEDROID)

        if ble_stack == BLE_STACK_NIMBLE:
            # NimBLE stack - lighter weight (~170KB flash, ~100KB RAM savings)
            cg.add_define("USE_BTHOME_NIMBLE")
            add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
            add_idf_sdkconfig_option("CONFIG_BT_CONTROLLER_ENABLED", True)
            # Disable Bluedroid
            add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", False)
            # NimBLE roles - only broadcaster needed for BTHome
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_CENTRAL", False)
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_OBSERVER", False)
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_PERIPHERAL", False)
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_BROADCASTER", True)
            # Broadcaster needs no connections, so disable the Security Manager
            # and host-based privacy (RPA) to avoid triggering
            # an "undefined reference to ble_sm_alg_encrypt" on classic ESP32.
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SECURITY_ENABLE", False)
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_HOST_BASED_PRIVACY", False)
            # Use tinycrypt for smaller footprint (saves ~7KB)
            add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_CRYPTO_STACK_MBEDTLS", False)
        else:
            # Bluedroid stack (default)
            cg.add_define("USE_BTHOME_BLUEDROID")
            cg.add_define("USE_ESP32_BLE_UUID")
            cg.add_define("USE_ESP32_BLE_ADVERTISING")
            add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
            add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", True)
            add_idf_sdkconfig_option("CONFIG_BT_BLE_42_FEATURES_SUPPORTED", True)

    elif CORE.is_nrf52:
        from esphome.components.zephyr import zephyr_add_prj_conf

        # Enable Bluetooth
        zephyr_add_prj_conf("BT", True)
        zephyr_add_prj_conf("BT_BROADCASTER", True)
        zephyr_add_prj_conf("BT_DEVICE_NAME", f'"{CORE.name}"')

        # Enable tinycrypt for AES-CCM encryption
        zephyr_add_prj_conf("TINYCRYPT", True)
        zephyr_add_prj_conf("TINYCRYPT_AES", True)
        zephyr_add_prj_conf("TINYCRYPT_AES_CCM", True)
