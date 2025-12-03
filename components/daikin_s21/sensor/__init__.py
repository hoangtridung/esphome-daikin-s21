"""
Sensor component for Daikin S21, supporting inside, outside, coil temperatures, and fan speed.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    UNIT_CELSIUS,
    ICON_THERMOMETER,
    DEVICE_CLASS_SPEED,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_CLIENT_SCHEMA,
    DaikinS21Client,
)

DaikinS21Sensor = daikin_s21_ns.class_(
    "DaikinS21Sensor", cg.PollingComponent, DaikinS21Client
)

CONF_INSIDE_TEMP = "inside_temperature"
CONF_OUTSIDE_TEMP = "outside_temperature"
CONF_COIL_TEMP = "coil_temperature"
CONF_FAN_SPEED = "fan_speed"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(DaikinS21Sensor),
            cv.Optional(CONF_INSIDE_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_OUTSIDE_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COIL_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                icon=ICON_THERMOMETER,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FAN_SPEED): sensor.sensor_schema(
                unit_of_measurement="rpm",
                icon="mdi:fan",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_SPEED,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.positive_time_period_seconds,
        }
    )
    .extend(S21_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("5s"))
)


async def to_code(config):
    """Generate main.cpp code"""

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    s21_var = await cg.get_variable(config[CONF_S21_ID])
    cg.add(var.set_s21(s21_var))
    if s21_var is None:
        raise cg.Invalid("s21_id must be set and valid!")
    
    
    if CONF_UPDATE_INTERVAL in config:
        cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL].total_milliseconds))   

    if CONF_INSIDE_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_INSIDE_TEMP])
        cg.add(var.set_temp_inside_sensor(sens))

    if CONF_OUTSIDE_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_OUTSIDE_TEMP])
        cg.add(var.set_temp_outside_sensor(sens))

    if CONF_COIL_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_COIL_TEMP])
        cg.add(var.set_temp_coil_sensor(sens))

    if CONF_FAN_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_FAN_SPEED])
        cg.add(var.set_fan_speed_sensor(sens))
