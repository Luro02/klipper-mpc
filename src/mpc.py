from .mpc_control import MPC

class PrinterMPC:
    _heaters: dict[str, "MPCSection"] = {}

    def __init__(self, config):
        self.printer = config.get_printer()
        # load mpc_calibrate:
        self.printer.load_object(config, 'mpc_calibrate')
        self._heaters = {}

    def register(self, name: str, section: "MPCSection"):
        self._heaters[name] = section

    def lookup_object(self, name: str) -> "MPCSection":
        if name not in self._heaters:
            raise self.printer.config_error(f"MPC is not defined in the config for '{name}'. You should add an '[mpc {name}]' section to the config.")

        return self._heaters[name]

class MPCSection:
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

    def handle_connect(self):
        # find the heater (ensures that it exists as well)
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)

def load_config(config):
    return PrinterMPC(config)

def load_config_prefix(config):
    return MPCSection(config)