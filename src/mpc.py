import math

from .mpc_control import MPC, ControlMPC
from typing import Any

class PrinterMPC:
    _heaters: dict[str, "MPCSection"] = {}

    def __init__(self, config):
        self.printer = config.get_printer()
        # load mpc_calibrate:
        self.printer.load_object(config, 'mpc_calibrate')
        self._heaters = {}

        # register gcode commands:
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('MPC_ENABLE', self.cmd_MPC_ENABLE, desc=self.cmd_MPC_ENABLE_help)
        gcode.register_command('MPC_UPDATE', self.cmd_MPC_UPDATE, desc=self.cmd_MPC_UPDATE_help)

    cmd_MPC_ENABLE_help = "Enable MPC algorithm for heating"
    def cmd_MPC_ENABLE(self, gcmd):
        heater_name = gcmd.get('HEATER', default="extruder")

        # find the options for the heater:
        try:
            mpc_section = self.lookup_object(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))

        self.printer.lookup_object('toolhead').get_last_move_time()

        heater = mpc_section.heater

        heater.set_control(ControlMPC(heater, mpc_section.config))

        gcmd.respond_info(f"Enabled MPC for {mpc_section.heater_name}, restart klipper to disable it.")

    cmd_MPC_UPDATE_help = "Update values for the MPC algorithm"
    def cmd_MPC_UPDATE(self, gcmd):
        params: dict[str, Any] = { str(k).lower():v for (k, v) in dict(gcmd.get_command_parameters()).items() }
        heater_name = params.pop('heater', "extruder")

        should_delete = bool(params.pop('delete', False))
        recalibrate = bool(params.pop('recalibrate', False))

        try:
            mpc_section = self.lookup_object(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))

        if should_delete:
            configfile = self.printer.lookup_object('configfile')
            configfile.remove_section(mpc_section.name())
            del self._heaters[heater_name]

            gcmd.respond_info(
                f"Profile [{mpc_section.name()}] removed from storage for this session.\n"
                "The SAVE_CONFIG command will update the printer\n"
                "configuration and restart the printer")

            return

        mpc_section.mpc.update(params)

        mpc = mpc_section.mpc
        # use the runtime values for recalculating the constants
        if recalibrate:
            # TODO: recalculate ambient_xfer_coeff

            # Calculate a new and better asymptotic temperature and re-evaluate the other constants

            # TODO: is mpc.data.heater_power correct? Shouldnt it be current draw * mpc.data.heater_power?
            asymp_temp = mpc.ambient_temp + mpc.data.heater_power / mpc.data.ambient_xfer_coeff_fan0
            # block_responsiveness = h_a / C_b where h_a = ambient_xfer_coeff and C_b = block_heat_capacity
            # TODO: the block_responsiveness can not be recalculated, because t3 and t1 are missing :/
            block_responsiveness = (-math.log((self.t3 - asymp_temp) / (self.t1 - asymp_temp))) / (self.t3_time - self.t1_time)

            # Update analytic tuning values based on the above
            mpc.data.block_heat_capacity = self.mpc.data.ambient_xfer_coeff_fan0 / block_responsiveness
            mpc.data.sensor_responsiveness = block_responsiveness / (1.0 - (self.mpc.ambient_temp - asymp_temp) * math.exp(-block_responsiveness * self.t1_time) / (self.t1 - asymp_temp))


    def register(self, name: str, section: "MPCSection"):
        self._heaters[name] = section

    def lookup_object(self, name: str) -> "MPCSection":
        if name not in self._heaters:
            raise self.printer.config_error(f"MPC is not defined in the config for '{name}'. You should add an '[mpc {name}]' section to the config.")

        return self._heaters[name]

class MPCSection:
    heater_name: str
    mpc: MPC

    def __init__(self, config):
        # parse the <name> in the section [mpc <name>] and error if the name contains whitespace, like [mpc extruder abc]
        config_parts = config.get_name().split()
        if len(config_parts) > 2:
            raise config.error(f"Name of section '{config.get_name()}' contains illegal whitespace")

        self.heater_name = config_parts[-1]

        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')

        # load the options defined in the section:
        self.mpc = MPC.from_config(config)

        # This initializes the PrinterMPC class if it has not already been initialized
        printer_mpc = self.printer.load_object(config, 'mpc')
        # Register this class with it. This is necessary, so that the calibrate command can
        # find this class.
        printer_mpc.register(self.heater_name, self)

        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def name(self) -> str:
        return self.config.get_name()

    def handle_connect(self):
        # find the heater (ensures that it exists as well)
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)

def load_config(config):
    return PrinterMPC(config)

def load_config_prefix(config):
    return MPCSection(config)
