import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
)

bticino_ns = cg.esphome_ns.namespace("bticino")
BticinoBus = bticino_ns.class_("BticinoBus", cg.Component)
BticinoDevice = bticino_ns.class_("BticinoDevice")
MULTI_CONF = True

CONF_BTICINO_BUS_ID = "bticino_bus_id"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BticinoBus),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    cg.add_global(bticino_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)


BTICINO_DEVICE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_BTICINO_BUS_ID): cv.use_id(BticinoBus),
        cv.Required(CONF_ADDRESS): cv.hex_uint8_t,
    }
)


async def register_bticino_device(var, config):
    parent = await cg.get_variable(config[CONF_BTICINO_BUS_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(parent.register_device(var))
