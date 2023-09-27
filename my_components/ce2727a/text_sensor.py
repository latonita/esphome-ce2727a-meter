import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
)

from . import CE2727aComponent, CONF_CE2727A_ID

AUTO_LOAD = ["ce2727a"]
CODEOWNERS = ["@latonita"]

CONF_ELECTRO_TARIFF = "electricity_tariff"
CONF_DATE = "date"
CONF_TIME = "time"
CONF_NETWORK_ADDRESS = "network_address"
CONF_SERIAL_NR = "serial_nr"


TEXT_SENSORS = [
    CONF_ELECTRO_TARIFF,
    CONF_DATE,
    CONF_TIME,
    CONF_NETWORK_ADDRESS,
    CONF_SERIAL_NR,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CE2727A_ID): cv.use_id(CE2727aComponent),
        cv.Optional(CONF_ELECTRO_TARIFF): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_DATE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_TIME): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_NETWORK_ADDRESS): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SERIAL_NR): text_sensor.text_sensor_schema(),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_CE2727A_ID])
    for key in TEXT_SENSORS:
        if key in config:
            conf = config[key]
            sens = cg.new_Pvariable(conf[CONF_ID])
            await text_sensor.register_text_sensor(sens, conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
