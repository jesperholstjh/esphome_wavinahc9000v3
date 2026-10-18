import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_DEVICE_CLASS,
    CONF_ICON,
    CONF_UNIT_OF_MEASUREMENT,
    UNIT_PERCENT,
    DEVICE_CLASS_BATTERY,
    ICON_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
)

from . import WavinAHC9000

CONF_PARENT_ID = "wavin_ahc9000_id"
CONF_CHANNEL = "channel"


CONF_TYPE = "type"

_BATTERY_DEFAULTS = {
    CONF_DEVICE_CLASS: DEVICE_CLASS_BATTERY,
    CONF_UNIT_OF_MEASUREMENT: UNIT_PERCENT,
    CONF_ICON: ICON_BATTERY,
    CONF_ACCURACY_DECIMALS: 0,
}
# temperature, comfort_setpoint and the floor sensors share temperature meta
_TEMPERATURE_DEFAULTS = {
    CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
    CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
    CONF_ACCURACY_DECIMALS: 1,
}


def _apply_type_defaults(config):
    # Defaults go into the config so sensor.new_sensor applies them; runtime setters
    # (set_icon, set_device_class, ...) were removed from esphome::sensor::Sensor.
    defaults = _BATTERY_DEFAULTS if config[CONF_TYPE] == "battery" else _TEMPERATURE_DEFAULTS
    for key, value in defaults.items():
        config.setdefault(key, value)
    return config


CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema().extend(
        {
            cv.GenerateID(CONF_PARENT_ID): cv.use_id(WavinAHC9000),
            cv.Required(CONF_CHANNEL): cv.int_range(min=1, max=16),
        cv.Required(CONF_TYPE): cv.one_of("battery", "temperature", "comfort_setpoint", "floor_temperature", "floor_min_temperature", "floor_max_temperature", lower=True),
        }
    ),
    _apply_type_defaults,
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PARENT_ID])
    sens = await sensor.new_sensor(config)
    if config[CONF_TYPE] == "battery":
        cg.add(hub.add_channel_battery_sensor(config[CONF_CHANNEL], sens))
    # yaml_ready numeric sensor removed in favor of binary_sensor platform
    elif config[CONF_TYPE] == "comfort_setpoint":
        cg.add(hub.add_channel_comfort_setpoint_sensor(config[CONF_CHANNEL], sens))
    elif config[CONF_TYPE] == "floor_temperature":
        cg.add(hub.add_channel_floor_temperature_sensor(config[CONF_CHANNEL], sens))
    elif config[CONF_TYPE] == "floor_min_temperature":
        cg.add(hub.add_channel_floor_min_temperature_sensor(config[CONF_CHANNEL], sens))
    elif config[CONF_TYPE] == "floor_max_temperature":
        cg.add(hub.add_channel_floor_max_temperature_sensor(config[CONF_CHANNEL], sens))
    else:
        cg.add(hub.add_channel_temperature_sensor(config[CONF_CHANNEL], sens))
    cg.add(hub.add_active_channel(config[CONF_CHANNEL]))
