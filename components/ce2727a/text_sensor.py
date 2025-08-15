import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CE2727aComponent, CONF_CE2727A_ID

AUTO_LOAD = ["ce2727a"]
CODEOWNERS = ["@latonita"]

CONF_TARIFF = "tariff"
CONF_DATE = "date"
CONF_TIME = "time"
CONF_NETWORK_ADDRESS = "network_address"
CONF_SERIAL_NR = "serial_nr"
CONF_READING_STATE = "reading_state"


TEXT_SENSORS = [
    CONF_TARIFF,
    CONF_DATE,
    CONF_TIME,
    CONF_NETWORK_ADDRESS,
    CONF_SERIAL_NR,
    CONF_READING_STATE,
]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_CE2727A_ID): cv.use_id(CE2727aComponent),
        cv.Optional(CONF_TARIFF): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_DATE): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_TIME): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_NETWORK_ADDRESS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_SERIAL_NR): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_READING_STATE): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
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
