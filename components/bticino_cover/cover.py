import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import cover
from .. import bticino_bus
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_OPEN_DURATION,
    CONF_CLOSE_DURATION,
    CONF_ASSUMED_STATE,
)

AUTO_LOAD = ["bticino_bus"]

bticino_ns = cg.esphome_ns.namespace("bticino")
BticinoCover = bticino_ns.class_(
    "BticinoCover", cover.Cover, cg.Component, bticino_bus.BticinoDevice
)

CONFIG_SCHEMA = (
    cover.COVER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(BticinoCover),
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ASSUMED_STATE, default=True): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(bticino_bus.BTICINO_DEVICE_SCHEMA)
)


def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield cover.register_cover(var, config)
    yield bticino_bus.register_bticino_device(var, config)

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_assumed_state(config[CONF_ASSUMED_STATE]))
