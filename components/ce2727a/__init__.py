import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart, time
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_FLOW_CONTROL_PIN,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    CONF_TIME_ID,
)

CODEOWNERS = ["@latonita"]

MULTI_CONF = True

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_CE2727A_ID = "ce2727a_id"

ce2727a_ns = cg.esphome_ns.namespace("esphome::ce2727a")
CE2727aComponent = ce2727a_ns.class_("CE2727aComponent", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CE2727aComponent),
            cv.Required(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_ADDRESS, default=0): cv.int_range (min=0x0, max=0xffffffff),
            cv.Optional(CONF_RECEIVE_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_UPDATE_INTERVAL, default="30s"): cv.update_interval,
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA),
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ce2727a", 
    require_rx=True,
    require_tx=True,
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_receive_timeout(config[CONF_RECEIVE_TIMEOUT].total_milliseconds))
    cg.add(var.set_requested_meter_address(config[CONF_ADDRESS]))

    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_))
