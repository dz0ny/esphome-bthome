"""
BTHome v2 BLE Receiver Component for ESPHome

Receives and decodes BTHome v2 BLE advertisements from external devices.
Supports sensors, binary sensors, events (button/dimmer), and text data.

Protocol specification: https://bthome.io/format/

Supports two BLE stacks:
- Bluedroid (default): Uses esp32_ble_tracker, compatible with other ESPHome BLE components
- NimBLE: Lightweight standalone stack, smaller footprint but cannot coexist with other BLE components
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MAC_ADDRESS,
    CONF_NAME,
    PLATFORM_ESP32,
)
from esphome import automation
from esphome.core import CORE
from esphome.components.esp32 import add_idf_sdkconfig_option

CODEOWNERS = ["@esphome/core"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

# BLE stack options
CONF_BLE_STACK = "ble_stack"
BLE_STACK_BLUEDROID = "bluedroid"
BLE_STACK_NIMBLE = "nimble"

CONF_DEVICES = "devices"
CONF_ENCRYPTION_KEY = "encryption_key"
CONF_ON_BUTTON = "on_button"
CONF_ON_DIMMER = "on_dimmer"
CONF_EVENT = "event"
CONF_BUTTON_INDEX = "button_index"
CONF_DUMP_INTERVAL = "dump_interval"

bthome_receiver_ns = cg.esphome_ns.namespace("bthome_receiver")
# Note: BTHomeReceiverHub class definition depends on BLE stack at runtime
# For Bluedroid it inherits from ESPBTDeviceListener, for NimBLE it's standalone
BTHomeReceiverHub = bthome_receiver_ns.class_("BTHomeReceiverHub", cg.Component)
BTHomeDevice = bthome_receiver_ns.class_("BTHomeDevice", cg.Parented.template(BTHomeReceiverHub))
BTHomeSensor = bthome_receiver_ns.class_("BTHomeSensor")
BTHomeBinarySensor = bthome_receiver_ns.class_("BTHomeBinarySensor")
BTHomeTextSensor = bthome_receiver_ns.class_("BTHomeTextSensor")

# Event triggers
BTHomeButtonTrigger = bthome_receiver_ns.class_(
    "BTHomeButtonTrigger", automation.Trigger.template()
)
# int8_t type for dimmer trigger parameter
int8_t = cg.global_ns.class_("int8_t")
BTHomeDimmerTrigger = bthome_receiver_ns.class_(
    "BTHomeDimmerTrigger", automation.Trigger.template(int8_t)
)

# =============================================================================
# BTHome v2 Sensor Object IDs - same as broadcaster component
# Format: "type_name": (object_id, data_bytes, signed, factor)
# =============================================================================
SENSOR_TYPES = {
    # Basic sensors
    "packet_id": (0x00, 1, False, 1),
    "battery": (0x01, 1, False, 1),
    "temperature": (0x02, 2, True, 0.01),
    "humidity": (0x03, 2, False, 0.01),
    "pressure": (0x04, 3, False, 0.01),
    "illuminance": (0x05, 3, False, 0.01),
    "mass_kg": (0x06, 2, False, 0.01),
    "mass_lb": (0x07, 2, False, 0.01),
    "dewpoint": (0x08, 2, True, 0.01),
    "count_uint8": (0x09, 1, False, 1),
    "energy": (0x0A, 3, False, 0.001),
    "power": (0x0B, 3, False, 0.01),
    "voltage": (0x0C, 2, False, 0.001),
    "pm2_5": (0x0D, 2, False, 1),
    "pm10": (0x0E, 2, False, 1),
    "co2": (0x12, 2, False, 1),
    "tvoc": (0x13, 2, False, 1),
    "moisture": (0x14, 2, False, 0.01),
    "humidity_uint8": (0x2E, 1, False, 1),
    "moisture_uint8": (0x2F, 1, False, 1),
    # Extended sensors
    "count_uint16": (0x3D, 2, False, 1),
    "count_uint32": (0x3E, 4, False, 1),
    "rotation": (0x3F, 2, True, 0.1),
    "distance_mm": (0x40, 2, False, 1),
    "distance_m": (0x41, 2, False, 0.1),
    "duration": (0x42, 3, False, 0.001),
    "current": (0x43, 2, False, 0.001),
    "speed": (0x44, 2, False, 0.01),
    "temperature_01": (0x45, 2, True, 0.1),
    "uv_index": (0x46, 1, False, 0.1),
    "volume_l_01": (0x47, 2, False, 0.1),
    "volume_ml": (0x48, 2, False, 1),
    "volume_flow_rate": (0x49, 2, False, 0.001),
    "voltage_01": (0x4A, 2, False, 0.1),
    "gas": (0x4B, 3, False, 0.001),
    "gas_uint32": (0x4C, 4, False, 0.001),
    "energy_uint32": (0x4D, 4, False, 0.001),
    "volume_l": (0x4E, 4, False, 0.001),
    "water": (0x4F, 4, False, 0.001),
    "timestamp": (0x50, 4, False, 1),
    "acceleration": (0x51, 2, False, 0.001),
    "gyroscope": (0x52, 2, False, 0.001),
    "volume_storage": (0x55, 4, False, 0.001),
    "conductivity": (0x56, 2, False, 1),
    "temperature_sint8": (0x57, 1, True, 1),
    "temperature_sint8_035": (0x58, 1, True, 0.35),
    "count_sint8": (0x59, 1, True, 1),
    "count_sint16": (0x5A, 2, True, 1),
    "count_sint32": (0x5B, 4, True, 1),
    "power_sint32": (0x5C, 4, True, 0.01),
    "current_sint16": (0x5D, 2, True, 0.001),
    "direction": (0x5E, 2, False, 0.01),
    "precipitation": (0x5F, 2, False, 0.1),
    "channel": (0x60, 1, False, 1),
    "rotational_speed": (0x61, 2, False, 1),
}

# =============================================================================
# BTHome v2 Binary Sensor Object IDs
# All are uint8: 0x00 = off/false, 0x01 = on/true
# =============================================================================
BINARY_SENSOR_TYPES = {
    "generic_boolean": 0x0F,
    "power": 0x10,
    "opening": 0x11,
    "battery_low": 0x15,
    "battery_charging": 0x16,
    "carbon_monoxide": 0x17,
    "cold": 0x18,
    "connectivity": 0x19,
    "door": 0x1A,
    "garage_door": 0x1B,
    "gas": 0x1C,
    "heat": 0x1D,
    "light": 0x1E,
    "lock": 0x1F,
    "moisture_binary": 0x20,
    "motion": 0x21,
    "moving": 0x22,
    "occupancy": 0x23,
    "plug": 0x24,
    "presence": 0x25,
    "problem": 0x26,
    "running": 0x27,
    "safety": 0x28,
    "smoke": 0x29,
    "sound": 0x2A,
    "tamper": 0x2B,
    "vibration": 0x2C,
    "window": 0x2D,
}

# Button event types for automation triggers
BUTTON_EVENT_TYPES = {
    "none": 0x00,
    "press": 0x01,
    "double_press": 0x02,
    "triple_press": 0x03,
    "long_press": 0x04,
    "long_double_press": 0x05,
    "long_triple_press": 0x06,
    "hold_press": 0x80,
}


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


BUTTON_TRIGGER_SCHEMA = automation.validate_automation(
    {
        cv.GenerateID(): cv.declare_id(BTHomeButtonTrigger),
        cv.Optional(CONF_BUTTON_INDEX, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_EVENT, default="press"): cv.one_of(*BUTTON_EVENT_TYPES.keys(), lower=True),
    }
)

DIMMER_TRIGGER_SCHEMA = automation.validate_automation(
    {
        cv.GenerateID(): cv.declare_id(BTHomeDimmerTrigger),
    }
)

DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BTHomeDevice),
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Optional(CONF_NAME): cv.string,
        cv.Optional(CONF_ENCRYPTION_KEY): validate_encryption_key,
        cv.Optional(CONF_ON_BUTTON): BUTTON_TRIGGER_SCHEMA,
        cv.Optional(CONF_ON_DIMMER): DIMMER_TRIGGER_SCHEMA,
    }
)

# Import esp32_ble_tracker at module level for schema extension
# pylint: disable=wrong-import-position
from esphome.components import esp32_ble_tracker

# Base schema that applies to all modes
_BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BTHomeReceiverHub),
        cv.Optional(CONF_BLE_STACK, default=BLE_STACK_BLUEDROID): cv.one_of(
            BLE_STACK_BLUEDROID, BLE_STACK_NIMBLE, lower=True
        ),
        cv.Optional(CONF_DEVICES, default=[]): cv.ensure_list(DEVICE_SCHEMA),
        # For Bluedroid mode: optional esp32_ble_tracker ID (auto-assigned if present)
        cv.Optional(esp32_ble_tracker.CONF_ESP32_BLE_ID): cv.use_id(
            esp32_ble_tracker.ESP32BLETracker
        ),
        # Interval for periodic dump of all detected devices (0 = disabled)
        cv.Optional(CONF_DUMP_INTERVAL): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


def _final_validate(config):
    """Final validation based on BLE stack."""
    ble_stack = config.get(CONF_BLE_STACK, BLE_STACK_BLUEDROID)

    if ble_stack == BLE_STACK_NIMBLE:
        if CORE.using_arduino:
            raise cv.Invalid("NimBLE BLE stack requires ESP-IDF framework, not Arduino")
    return config


CONFIG_SCHEMA = cv.All(
    _BASE_SCHEMA,
    _final_validate,
    cv.only_on([PLATFORM_ESP32]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set dump interval for periodic summary (0 = disabled)
    if CONF_DUMP_INTERVAL in config:
        cg.add(var.set_dump_interval(config[CONF_DUMP_INTERVAL]))

    ble_stack = config.get(CONF_BLE_STACK, BLE_STACK_BLUEDROID)

    if ble_stack == BLE_STACK_NIMBLE:
        # NimBLE stack configuration
        cg.add_define("USE_BTHOME_RECEIVER_NIMBLE")

        # Enable NimBLE in ESP-IDF
        add_idf_sdkconfig_option("CONFIG_BT_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_CONTROLLER_ENABLED", True)
        add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", False)

        # Configure NimBLE roles - only observer needed for receiving
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_OBSERVER", True)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_BROADCASTER", False)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_CENTRAL", False)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_ROLE_PERIPHERAL", False)

        # Use tinycrypt for smaller footprint
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_CRYPTO_STACK_MBEDTLS", False)

        # Disable privacy/security features we don't need (avoids SM requirement)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SECURITY_ENABLE", False)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_LEGACY", False)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_SM_SC", False)

        # NimBLE memory optimization
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_MAX_CONNECTIONS", 0)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_MAX_BONDS", 0)
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_PINNED_TO_CORE", 0)

        # Disable NimBLE logging completely
        add_idf_sdkconfig_option("CONFIG_BT_NIMBLE_LOG_LEVEL", 0)  # 0 = NONE
    else:
        # Bluedroid stack - use esp32_ble_tracker
        cg.add_define("USE_BTHOME_RECEIVER_BLUEDROID")
        # These defines are needed for ESPBTDevice and ServiceData types
        cg.add_define("USE_ESP32_BLE_DEVICE")
        cg.add_define("USE_ESP32_BLE_UUID")
        # Import esp32_ble_tracker only when using Bluedroid
        # pylint: disable=import-outside-toplevel
        from esphome.components import esp32_ble_tracker

        # Register with the global BLE tracker (requires esp32_ble_tracker: in config)
        # Use the official helper so the listener is counted for the
        # ESPHOME_ESP32_BLE_TRACKER_LISTENER_COUNT StaticVector sizing.
        # Manual parent.register_listener() bypasses the count and silently
        # drops the last-registered listener at runtime (vector full).
        tracker_id = config.get(esp32_ble_tracker.CONF_ESP32_BLE_ID)
        if tracker_id is not None:
            await esp32_ble_tracker.register_ble_device(var, config)

    for device_conf in config.get(CONF_DEVICES, []):
        device_var = cg.new_Pvariable(device_conf[CONF_ID], var)
        mac = device_conf[CONF_MAC_ADDRESS]
        cg.add(device_var.set_mac_address(mac.as_hex))

        if CONF_NAME in device_conf:
            cg.add(device_var.set_name(device_conf[CONF_NAME]))

        if CONF_ENCRYPTION_KEY in device_conf:
            key = device_conf[CONF_ENCRYPTION_KEY]
            key_bytes = [cg.RawExpression(f"0x{key[i:i+2]}") for i in range(0, len(key), 2)]
            key_array = cg.RawExpression(f"std::array<uint8_t, 16>{{{{{', '.join(str(b) for b in key_bytes)}}}}}")
            cg.add(device_var.set_encryption_key(key_array))

        # Register button triggers
        for button_conf in device_conf.get(CONF_ON_BUTTON, []):
            trigger = cg.new_Pvariable(button_conf[CONF_ID], device_var)
            event_type = BUTTON_EVENT_TYPES[button_conf[CONF_EVENT]]
            cg.add(trigger.set_button_index(button_conf[CONF_BUTTON_INDEX]))
            cg.add(trigger.set_event_type(event_type))
            cg.add(device_var.add_button_trigger(trigger))
            await automation.build_automation(trigger, [], button_conf)

        # Register dimmer triggers
        for dimmer_conf in device_conf.get(CONF_ON_DIMMER, []):
            trigger = cg.new_Pvariable(dimmer_conf[CONF_ID], device_var)
            cg.add(device_var.add_dimmer_trigger(trigger))
            await automation.build_automation(trigger, [(cg.int8, "steps")], dimmer_conf)

        cg.add(var.register_device(device_var))
