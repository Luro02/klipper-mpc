import logging
import math

from dataclasses import dataclass, fields, MISSING
from typing import Optional

# This class keeps track of relevant data for the other classes that rely on this.
class ControlTemperature:
    last_heater_pwm: Optional[float]

    # A list of recorded measurements. Each entry is a tuple of (read_time, read_temperature),
    # the time is in seconds.
    records: list[(float, float)]

    # Recording every single reading while ControlTemperature is in use, will result in
    # an out of memory error eventually. To prevent this, one has to explicitly indicate
    # that one wants to record measurements.
    _recorders: list[float]

    _temp_records: list[(float, float)]

    def __init__(self, printer, heater, fan = None) -> None:
        self.printer = printer
        self.heater = heater
        self.heater_max_power = float(heater.get_max_power())
        self.fan = fan

        self.gcode = self.printer.lookup_object('gcode')

        self.last_heater_pwm = self.heater.last_pwm_value / self.heater_max_power

        self.records = []
        self._recorders = []
        self._temp_records = []

    def start_recording(self, time: float):
        if len(self._recorders) == 0 and len(self._temp_records) > 0 and self._temp_records[0][0] >= time:
            self.records.append(self._temp_records[0])
            self._temp_records.clear()

        self._recorders.append(time)

    def stop_recording(self, start_time: float) -> list[(float, float)]:
        # remove the recorder from the list (NOTE: this will only remove the first occurence):
        self._recorders.remove(start_time)

        # fetch all records after or at start time, sort them by time and make them relative to the start_time
        records = sorted([(time - start_time, temp) for (time, temp) in self.records if time >= start_time], key=lambda e: e[0])

        if len(self._recorders) > 0:
            earliest_start_time = min(self._recorders)

            # only keep recors after the earliest start time:
            self.records = [(time, temp) for (time, temp) in self.records if time >= earliest_start_time]
        else:
            # if no recorders a present, one can remove all records:
            self.records = []

        return records

    def error(self, message: str):
        # TODO: decide on better way to error?
        return self.printer.config_error(message)

    def log(self, message: str) -> None:
        if self.gcode is not None:
            self.gcode.respond_info(message)

        logging.info(message)

    def ensure_within_range(self, value: float, start: float, end: float, context: str):
        if value < start or value > end:
            raise self.error(f"{context}: The value '{value}' must be in range [{start}, {end}]")

    # returns a percentage between 0.0 and 1.0, where 0.0 = off and 1.0 = maximum speed
    def get_fan_speed(self) -> float:
        return float(self.fan.last_fan_value)

    def set_fan_speed(self, time: float, value: float):
        self.ensure_within_range(value, 0.0, 1.0, "fan speed")

        if self.fan is not None:
            self.fan.set_speed_from_command(value)

    def get_heater_pwm(self) -> float:
        return self.heater.last_pwm_value / self.heater_max_power

    def set_heater_pwm(self, time: float, value: Optional[float]):
        if value is None:
            self.last_heater_pwm = None
            return

        self.ensure_within_range(value, 0.0, 1.0, "heater pwm")

        # self.log(f"[{time:.04f}] Updating heater PWM to {value} (max_power = {self.heater_max_power})")

        self.last_heater_pwm = value * self.heater_max_power

    def record(self, read_time, temp, target_temp):
        # To prevent missed records, the record function should
        # always be called before any other code in the temperature_update
        # method.
        #
        # It could then happen that a recording is started (with the same
        # start time), but the record function had already discarded the
        # measurement, because there was no recorder at the time.
        #
        # An extra list is used as a temporary storage, that will be transferred
        # to the records, if a recorder is registered.

        if len(self._recorders) > 0:
            self.records.append((read_time, temp))
        else:
            self._temp_records = [(read_time, temp)]

    def update_heater(self, read_time, temp, target_temp):
        if self.last_heater_pwm is not None:
            self.heater.set_pwm(read_time, self.last_heater_pwm)

class MPC:
    # Contains constants that either never change or only through calibration
    data: "MPCData"
    # If the part cooling fan should be included in the model
    include_fan: bool

    # Model Data:
    
    ambient_temp: float
    block_temp: float
    sensor_temp: float

    last_e_position: float
    prev_temp_time: float

    def __init__(self, data, include_fan=True, ambient_temp=None) -> None:
        self.data = data
        self.include_fan = include_fan

        self.ambient_temp = ambient_temp
        self.block_temp = None
        self.sensor_temp = None

        self.last_e_position = 0.
        self.prev_temp_time = 0.

    @classmethod
    def from_config(cls, config, **kwargs) -> "MPC":
        args = dict(kwargs)
        
        include_fan = args.pop('include_fan', None)
        if include_fan is None:
            include_fan = config.getboolean('include_fan', default=True)

        data = MPCData.from_config(config, **args)

        return cls(data, include_fan)

    def save(self, config, log=False):
        self.data.save(config, log=log)

    def __str__(self) -> str:
        attrs = [(attr, getattr(self, attr)) for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("__")]
        
        return f"{self.__class__.__name__}({', '.join([f'{name}={value}' for (name, value) in attrs])})"

@dataclass
class MPCData:
    # (W) Heat cartridge powers.
    heater_power: float = 40.0
    # (K/s per ∆K) Rate of change of sensor temperature from heat block.
    sensor_responsiveness: float = 0.0
    # (J/K) Heat block heat capacities.
    block_heat_capacity: float = 0.0
    # (W/K) Heat transfer coefficients from heat block to room air with fan off.
    ambient_xfer_coeff_fan0: float = 0.0
    # (W/K) Heat transfer coefficients from heat block to room air with fan on full.
    ambient_xfer_coeff_fan255: Optional[float] = None

    # Advanced options, all of them have fallbacks:

    # Filament Heat Capacity (joules/kelvin/mm)
    # Set at runtime with M306 H<value>
    # TODO: implement that^
    #
    # Here are more values:
    # 0.0056 J/K/mm for 1.75mm PLA (0.0149 J/K/mm for 2.85mm PLA).
    # 0.0036 J/K/mm for 1.75mm PETG (0.0094 J/K/mm for 2.85mm PETG).
    # 0.00515 J/K/mm for 1.75mm ABS (0.0137 J/K/mm for 2.85mm ABS).
    # 0.00522 J/K/mm for 1.75mm Nylon (0.0138 J/K/mm for 2.85mm Nylon).
    filament_heat_capacity: float = 0.0056 # 1.75mm PLA
    # (0.0...1.0) Noisy temperature sensors may need a lower value for stabilization.
    smoothing_factor: float = 0.5
    # (K/s) Temperature change rate for steady state logic to be enforced.
    steadystate: float = 0.5
    # (K/s) Modeled ambient temperature rate of change, when correcting model inaccuracies.
    min_ambient_change: float = 1.0

    @classmethod
    def from_config(cls, config, **kwargs) -> "MPCData":
        args = dict(kwargs)
        for field in fields(cls):
            # TODO: call correct method based on field type?
            option = field.name
            if option not in args:
                value = config.getfloat(option, default=None)

                if value is not None:
                    args[option] = value
                elif field.default is not MISSING:
                    args[option] = field.default
                else:
                    raise config.error(f"missing value for option '{option}' in config section '{config.get_name()}'")

        return cls(**args)

    def fan255_adjustment(self):
        if self.ambient_xfer_coeff_fan255 is None:
            return 0.0

        return self.ambient_xfer_coeff_fan255 - self.ambient_xfer_coeff_fan0

    def save(self, config, log=False):
        # Store results for SAVE_CONFIG
        printer = config.get_printer()
        configfile = printer.lookup_object('configfile')
        section_name = config.get_name()

        # round to 5 decimal places, because some values might be very small
        values = [(field.name, f"{getattr(self, field.name):.5f}") for field in fields(self)]

        for (option, value) in values:
            configfile.set(section_name, option, value)


        if log:
            output = ', '.join([k + '=' + v for (k, v) in values])
            logging.info(f"MPC parameters: {output}")

            printer.lookup_object('gcode').respond_info(
                f"MPC parameters: {output}\n"
                "The SAVE_CONFIG command will update the printer config file\n"
                "with these parameters and restart the printer."
            )

class ControlMPC:
    mpc: MPC

    def __init__(self, heater, config, mpc: Optional[MPC] = None):
        self.printer = config.get_printer()
        self.toolhead = self.printer.lookup_object('toolhead')
 
        self.heater = heater
        self.heater_max_power = heater.get_max_power()

        self.mpc = mpc or MPC.from_config(config)

        required_data_fields = ["sensor_responsiveness", "block_heat_capacity", "ambient_xfer_coeff_fan0"]
        undefined_data_fields = []
        for field in required_data_fields:
            if getattr(self.mpc.data, field) == 0.0:
                undefined_data_fields.append(field)

        if len(undefined_data_fields) > 0:
            raise config.error(f"Missing values for the fields {', '.join(required_data_fields)}. Execute MPC_CALIBRATE to determine these values.")

        fan = None
        if self.mpc.include_fan:
            printer_fan = self.printer.lookup_object('fan')
            fan = printer_fan.fan

        self.next_output_time = 0.0
        self.control_temp = ControlTemperature(self.printer, self.heater, fan=fan)
        self.control_temp.last_heater_pwm = 0.0

    # TODO: ensure that this is correct?
    def steps_to_mm(self, steps: float) -> float:
        # this should be an extruder object from kinematics, a PrinterExtruder
        printer_extruder = self.toolhead.get_extruder()
        extruder_stepper = printer_extruder.extruder_stepper

        # This should be how much a single step is in mm:
        step_dist = float(extruder_stepper.stepper.get_step_dist())

        return step_dist * steps

    def max_e_velocity(self) -> float:
        return self.toolhead.get_extruder().max_e_velocity 

    def _internal_temperature_update(self, read_time, temp, target_temp):
        time_diff = read_time - self.mpc.prev_temp_time

        # On the first call, the prev_temp_time will be 0. If the read_time is very large,
        # the modeled block_temp will be very large as well (like 2300°C).
        #
        # To prevent this, the time_diff is set to a fixed number for the first update.
        if self.mpc.prev_temp_time == 0.0:
            # the temperature_update is called every 0.3s
            time_diff = 0.3

        # initialize model temperatures on first call:
        if self.mpc.block_temp is None:
            if self.mpc.ambient_temp is None:
                # limit room temperature to at most 30°C
                self.mpc.ambient_temp = min(30.0, temp)
            self.mpc.block_temp = temp
            self.mpc.sensor_temp = temp

        ambient_xfer_coeff = self.mpc.data.ambient_xfer_coeff_fan0

        if self.mpc.include_fan:
            ambient_xfer_coeff += self.control_temp.get_fan_speed() * self.mpc.data.fan255_adjustment()

        # this should be an extruder object from kinematics, a PrinterExtruder
        printer_extruder = self.toolhead.get_extruder()
        extruder_stepper = printer_extruder.extruder_stepper
        e_position = extruder_stepper.find_past_position(read_time)

        e_speed = self.steps_to_mm(e_position - self.mpc.last_e_position) / time_diff

        # the position can appear to make big jumps when, e.g. homing
        if math.fabs(e_speed) > self.max_e_velocity(): # TODO: mm/s or mm/min, has to be in mm/s?
            self.mpc.last_e_position = e_position
        elif e_speed > 0.0: # ignore retract/recover moves
            # if (!MPC::e_paused) ambient_xfer_coeff += e_speed * mpc.filament_heat_capacity_permm;
            ambient_xfer_coeff += e_speed * self.mpc.data.filament_heat_capacity
            self.mpc.last_e_position = e_position

        last_pwm_value = self.control_temp.get_heater_pwm()

        # update the modeled temperatures
        blocktempdelta = last_pwm_value * self.mpc.data.heater_power * time_diff / self.mpc.data.block_heat_capacity
        blocktempdelta += (self.mpc.ambient_temp - self.mpc.block_temp) * ambient_xfer_coeff * time_diff / self.mpc.data.block_heat_capacity
        self.mpc.block_temp += blocktempdelta

        # const float sensortempdelta = (hotend.modeled_block_temp - hotend.modeled_sensor_temp) * (constants.sensor_responsiveness * MPC_dT);
        # hotend.modeled_sensor_temp += sensortempdelta;
        sensortempdelta = (self.mpc.block_temp - self.mpc.sensor_temp) * (self.mpc.data.sensor_responsiveness * time_diff)
        self.mpc.sensor_temp += sensortempdelta

        # Any delta between sensor_temp and current temperature is either model
        # error diverging slowly or (fast) noise. Slowly correct towards this temperature
        # and noise will average out.
        delta_to_apply = (temp - self.mpc.sensor_temp) * self.mpc.data.smoothing_factor
        self.mpc.block_temp += delta_to_apply
        self.mpc.sensor_temp += delta_to_apply

        # // Only correct ambient when close to steady state (output power is not clipped or asymptotic temperature is reached)
        # if (WITHIN(hotend.soft_pwm_amount, 1, 126) || fabs(blocktempdelta + delta_to_apply) < (MPC_STEADYSTATE * MPC_dT))
        #   hotend.modeled_ambient_temp += delta_to_apply > 0.f ? _MAX(delta_to_apply, MPC_MIN_AMBIENT_CHANGE * MPC_dT) : _MIN(delta_to_apply, -MPC_MIN_AMBIENT_CHANGE * MPC_dT);

        # only correct ambient when close to steady state (output power is not clipped or asymptotic temperature is reached)
        # TODO: ambient_temp can become negative
        if (last_pwm_value > 0.0 and last_pwm_value < 1.0) or math.fabs(blocktempdelta + delta_to_apply) < self.mpc.data.steadystate * time_diff:
            if delta_to_apply > 0.0:
                self.mpc.ambient_temp += max(delta_to_apply, self.mpc.data.min_ambient_change * time_diff)
            else:
                self.mpc.ambient_temp += min(delta_to_apply, -self.mpc.data.min_ambient_change * time_diff)

        # float power = 0.0;
        # if (hotend.target != 0 && TERN1(HEATER_IDLE_HANDLER, !heater_idle[ee].timed_out)) {
        #   // plan power level to get to target temperature in 2 seconds
        #   power = (hotend.target - hotend.modeled_block_temp) * constants.block_heat_capacity / 2.0f;
        #   power -= (hotend.modeled_ambient_temp - hotend.modeled_block_temp) * ambient_xfer_coeff;
        # }
        power = 0.0
        if target_temp > 0.0:
            # plan power level to get to target temperature in 2 seconds:
            power = (target_temp - self.mpc.block_temp) * self.mpc.data.block_heat_capacity / 2.0
            power -= (self.mpc.ambient_temp - self.mpc.block_temp) * ambient_xfer_coeff

        # float pid_output = power * 254.0f / mpc.heater_power + 1.0f;        // Ensure correct quantization into a range of 0 to 127
        # pid_output = constrain(pid_output, 0, 255);

        # pid_output = power * 254.0 / self.mpc.data.heater_power + 1.0
        # pid_output = float(int(max(0.0, min(255.0, pid_output)))//2) / 127.0

        pid_output = max(0.0, min(1.0, power / self.mpc.data.heater_power))

        if read_time > self.next_output_time:
            self.control_temp.log(f"MPC Control: ({read_time}, {temp}, {target_temp}): {last_pwm_value} -> {pid_output}, {self.mpc}")
            self.next_output_time = read_time + 1.0

        self.control_temp.set_heater_pwm(read_time, pid_output)

    def temperature_update(self, read_time, temp, target_temp):
        self.control_temp.record(read_time, temp, target_temp)
        
        self._internal_temperature_update(read_time, temp, target_temp)
        
        self.control_temp.update_heater(read_time, temp, target_temp)
        self.mpc.prev_temp_time = read_time

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return math.fabs(temp_diff) < 1.0
