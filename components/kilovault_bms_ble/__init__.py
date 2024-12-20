import esphome.codegen as cg
from esphome.components import ble_client
import esphome.config_validation as cv
from esphome.const import CONF_ID

# CONFIG_VALIDATION: Uses an underlying system  called voluptuous here:
#   https://github.com/alecthomas/voluptuous

# This is a magic symbol required by the esphome core that referrs to the original 
#   dev of this component. 
CODEOWNERS = ["@syssi"]

# Automatically load a component if the user hasn't added it manually
AUTO_LOAD = ["binary_sensor", "sensor", "switch", "text_sensor"]

# Mark this component to accept an array of configurations. If this is an integer 
#   instead of a boolean, validating will only permit the given number of entries. 
MULTI_CONF = True

CONF_KILOVAULT_BMS_BLE_ID = "kilovault_bms_ble_id"

kilovault_bms_ble_ns = cg.esphome_ns.namespace("kilovault_bms_ble")

KilovaultBmsBle = kilovault_bms_ble_ns.class_(
    "KilovaultBmsBle", ble_client.BLEClientNode, cg.PollingComponent
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(KilovaultBmsBle),
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("10s"))
)

# Code Generation
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

