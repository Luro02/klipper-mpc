# scp -r * lucas@printer.local:~/klipper-mpc
import math

import traceback
import textwrap

from enum import Enum, auto
from .mpc_control import MPC, ControlMPC, ControlTemperature, DEFAULT_CYCLE_TIME, has_settled_at_target

def interp(x: float, fp: list[(float, float)]) -> float:
    xs, ys = zip(*fp)

    if len(xs) <= 1:
        raise ValueError(f"Not enough values for interpolation: {xs}")

    if not all(xs[i] < xs[i + 1] for i in range(len(xs) - 1)):
        raise ValueError(f"the given x-coordinates are not sorted: {xs}")

    # find the two closest xs to the x, where one is smaller than and the other is larger than x
    previous_index = max((i for i in range(len(xs)) if xs[i] <= x), key=lambda i: xs[i], default=None)
    next_index = min((i for i in range(len(xs)) if xs[i] >= x), key=lambda i: xs[i], default=None)

    # if x is outside of the measured points:
    if previous_index is None:
        return float(ys[0])

    if next_index is None:
        return float(ys[-1])

    # check if the value is in the measurements:
    if previous_index == next_index:
        return float(ys[previous_index])

    (x0, y0) = (float(xs[previous_index]), float(ys[previous_index]))
    (x1, y1) = (float(xs[next_index]), float(ys[next_index]))

    return (y0 * (x1 - x) + y1 * (x - x0)) / (x1 - x0)

class MPCTuningType(Enum):
    AUTO = auto()
    FORCE_ASYMPTOTIC = auto()
    FORCE_DIFFERENTIAL = auto()

    @classmethod
    def from_str(cls, value: str) -> "MPCTuningType":
        tuning_types = { variant.name.lower():variant for variant in cls }

        variant = tuning_types.get(value, None)

        if variant is None:
            raise ValueError(f"tuning_type '{value}' is not known, available: [{', '.join(tuning_types.keys())}]")

        return variant

class MPCCalibrate:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('MPC_CALIBRATE', self.cmd_MPC_CALIBRATE, desc=self.cmd_MPC_CALIBRATE_help)

    cmd_MPC_CALIBRATE_help = "Run MPC calibration test"
    def cmd_MPC_CALIBRATE(self, gcmd):
        heater_name = gcmd.get('HEATER', default="extruder")
        target = 200.0
        if heater_name != "extruder":
            target = gcmd.get_float('TARGET', default=200.0)

        try:
            tuning_type = MPCTuningType.from_str(gcmd.get('TYPE', "auto"))
        except ValueError as e:
            raise gcmd.error(str(e))

        # find the options for the heater:
        try:
            mpc_section = self.printer.lookup_object('mpc').lookup_object(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))

        eventtime = self.printer.lookup_object('toolhead').get_last_move_time()

        heater = mpc_section.heater
        config = mpc_section.config
        mpc: MPC = mpc_section.mpc

        (ambient_temp, target_temp) = heater.get_temp(eventtime)
        if target_temp > 0.:
            raise gcmd.error(f"Heater must be cold, target set to {target_temp}, but should be 0.")

        mpc.ambient_temp = ambient_temp
        pheaters = self.printer.lookup_object('heaters')

        # insert new control algorithm:
        calibrate = ControlAutoTune(heater, target, config, tuning_type, mpc)
        old_control = heater.set_control(calibrate)
        try:
            # This call instructs the printer to heat the heater to the target temperature.
            # It delegates this to heater.set_temp(target) and then blocks until the
            # temperature is reached.
            #
            # heater.set_temp(target) only sets the attribute `heater.target_temp` to the provided
            # argument (and ensures that it is in range)
            #
            # It waits for the temperature by calling control.check_busy() until it returns False
            pheaters.set_temperature(heater, target, True)
        finally:
            # Always restore the old_control algorithm (even if an error occurs):
            heater.set_control(old_control)

        if calibrate.check_busy(0., 0., 0.):
            raise gcmd.error("mpc_calibrate interrupted")

        # Save MPC values and report them on the command line:
        mpc.save(config, log=True)

class ControlAutoTuneState(Enum):
    DETERMINE_AMBIENT_TEMPERATURE = auto()
    MEASURE_HEATING = auto()
    STABILIZE_SYSTEM = auto()
    DETERMINE_HEATLOSS = auto()

    def next(self):
        cls = self.__class__
        members = list(cls)
        index = members.index(self) + 1
        if index >= len(members):
            return None

        return members[index]

class ControlAutoTune:
    def __init__(self, heater, target, config, tuning_type: MPCTuningType, mpc: MPC):
        self.config = config
        self.printer = config.get_printer()

        self.heater = heater
        self.calibrate_temp = target

        self.mpc = mpc

        self.tuning_type = tuning_type

        fan = None
        if self.mpc.include_fan:
            printer_fan = self.printer.lookup_object('fan')
            fan = printer_fan.fan
        self.control_temp = ControlTemperature(self.printer, self.heater, fan=fan)

        self.heating = False
        self.settle_window = []

        self.state = ControlAutoTuneState.DETERMINE_AMBIENT_TEMPERATURE

    def respond_info(self, message):
        self.control_temp.log(message)

    def determine_ambient_temperature(self, read_time, temp, target_temp):
        # When the printer is cooling down, it never reaches 0°C, but stops at
        # around room temperature. This is called the ambient temperature.
        #
        # While the printer is cooling, one can only measure the temperature at
        # the heater. The calibration code might be called while the printer
        # is cooling down, so the measurement is not guranteed to be ambient
        # temperature.

        # On the first call the target_temp, would be somewhere around ~200°C.
        # Before heating to that temperature, it is important to figure out the
        # room temperature.
        #
        # The target_temp is set by this method to 1°C, because then the heater fan
        # stays on. This speeds up finding the ambient temperature.
        if target_temp > 1.0:
            # Assume the currently measured temperature is the ambient temp (if not it will later be replaced)
            self.ambient_target = temp

            self.respond_info("Setting target temp to 1.0 for finding ambient temperature")
            self.heater.alter_target(1.)

            # indicate that it should not heat:
            self.control_temp.set_heater_pwm(read_time, 0.0)

            # enable part cooling fan for faster cooldown
            self.control_temp.set_fan_speed(read_time, 1.0)
            return


        if has_settled_at_target(read_time, temp, self.ambient_target, self.settle_window, window_size=20, tolerance=0.2):
            # ensure that the ambient temp is not above 30°C:
            self.mpc.ambient_temp = min(self.ambient_target, 30.0, temp)
            self.control_temp.set_fan_speed(read_time, 0.0)
            # advance to the next state of the calibration:
            self.state = self.state.next()
            self.heater.alter_target(0.)

            self.settle_window = []

            self.respond_info(f"Found ambient_temp={self.mpc.ambient_temp}")
            return

        # The measured temperature is lower than the targeted ambient temperature
        # => use the middle between those values as new ambient target
        if temp < self.ambient_target:
            self.ambient_target = (self.ambient_target + temp) / 2.0

        self.mpc.ambient_temp = temp

    def measure_heating(self, read_time, temp, target_temp):
        # When this function is called, the printer is at ambient temperature (target set to 0):
        if target_temp == 0.0:
            self.respond_info(f"Starting to measure heating")
            self.control_temp.set_heater_pwm(read_time, 1.0)
            self.heater.alter_target(self.calibrate_temp)
            self.heating = True

            # control_temp will take the measurements:
            self.control_temp.start_recording(read_time)
            # record the time at which it started heating
            self.heat_start_time = read_time
            return

        # if it did not reach the target temperature, continue measuring/heating
        if temp < target_temp:
            return

        # samples contains all recorded temperatures since it started heating:
        samples = self.control_temp.stop_recording(self.heat_start_time)
        self.respond_info(f"Samples: [{', '.join('(' + str(k) + ', ' + str(v) + ')' for (k, v) in samples)}]")

        self.respond_info("Finished collecting data for calculating constants. Temporarily setting pwm to 0.0")
        # turn off the heater
        self.control_temp.set_heater_pwm(read_time, 0.0)
        self.heating = False

        # Three evenly spaced samples t1, t2 and t3 are required, with exactly delta time between them:
        # t1_time + delta = t2_time
        # t2_time + delta = t3_time
        #
        # If one calculates the delta as (t3_time - t1_time) / 2, the above might not hold, depending on what
        # value delta is.
        #
        # Therefore the span is first rounded down to the nearest whole number, then it is adjusted to the next
        # even number => there will be a whole number as delta

        first_phase = [(time, temp) for (time, temp) in samples if temp < 100.0]
        second_phase = [(time, temp) for (time, temp) in samples if temp >= 100.0 and temp < 200.0]

        # the first time it measured a temperature over 100.0
        self.t1_time = second_phase[0][0]
        self.respond_info(f"t1_time = {self.t1_time}")

        last_sample = second_phase[-1]
        time_span = math.floor(last_sample[0] - self.t1_time)
        if time_span % 2 != 0:
            time_span -= 1
        delta = float(time_span / 2.0)

        self.t2_time = delta + self.t1_time
        self.t3_time = 2.0 * delta + self.t1_time

        self.t1 = interp(self.t1_time, samples)
        self.t2 = interp(self.t2_time, samples)
        self.t3 = interp(self.t3_time, samples)

        asymp_temp = (self.t2 * self.t2 - self.t1 * self.t3) / (2 * self.t2 - self.t1 - self.t3)
        # block_responsiveness = -log((t2 - asymp_temp) / (t1 - asymp_temp)) / tuner.get_sample_interval();
        # elapsed_time was (self.sample_distance * (self.sample_count // 2))
        block_responsiveness = (-math.log((self.t3 - asymp_temp) / (self.t1 - asymp_temp))) / (self.t3_time - self.t1_time)

        self.respond_info(f"""The following data has been calculated from the samples:
(t1_time, t1) = ({self.t1_time}, {self.t1})
(t2_time, t2) = ({self.t2_time}, {self.t2})
(t3_time, t3) = ({self.t3_time}, {self.t3})

asymp_temp = {asymp_temp}
block_responsiveness = {block_responsiveness}
""")


        # Make initial guess at transfer coefficients
        ambient_xfer_coeff_fan0 = self.mpc.data.heater_power / (asymp_temp - self.mpc.ambient_temp)
        if self.tuning_type == MPCTuningType.AUTO or self.tuning_type == MPCTuningType.FORCE_ASYMPTOTIC:
            # Analytic tuning
            block_heat_capacity = ambient_xfer_coeff_fan0 / block_responsiveness
            sensor_responsiveness = block_responsiveness / (1.0 - (self.mpc.ambient_temp - asymp_temp) * math.exp(-block_responsiveness * self.t1_time) / (self.t1 - asymp_temp))

            self.respond_info(f"block_heat_capacity={block_heat_capacity}\nsensor_responsiveness={sensor_responsiveness}")

        # If analytic tuning fails, fall back to differential tuning
        if self.tuning_type == MPCTuningType.AUTO and (sensor_responsiveness <= 0 or block_heat_capacity <= 0):
            self.tuning_type = MPCTuningType.FORCE_DIFFERENTIAL
            self.respond_info("Analytic tuning failed, using different calibration method.")

        if self.tuning_type == MPCTuningType.FORCE_DIFFERENTIAL:
            (block_heat_capacity, sensor_responsiveness) = self.calculate_differential(first_phase)

        elapsed_heating_time = read_time - self.heat_start_time
        self.respond_info(f"elapsed_heating_time = {elapsed_heating_time}")

        self.mpc.block_temp = asymp_temp + (self.mpc.ambient_temp - asymp_temp) * math.exp(-block_responsiveness * elapsed_heating_time)
        self.mpc.sensor_temp = temp

        self.respond_info(f"block_temp={self.mpc.block_temp}")
        self.respond_info(f"sensor_temp={self.mpc.sensor_temp}")
        self.respond_info(f"asymp_temp={asymp_temp}")

        self.mpc.data.sensor_responsiveness = sensor_responsiveness
        self.mpc.data.block_heat_capacity = block_heat_capacity
        self.mpc.data.ambient_xfer_coeff_fan0 = ambient_xfer_coeff_fan0

        self.respond_info("Updated MPC values.")

        self.control_mpc = ControlMPC(self.heater, self.config, self.mpc)

        self.respond_info(f"Calculated Values: {str(self.mpc)}")

        # Allow the system to stabilize under MPC, then get a better measure of ambient loss with and without fan
        self.state = self.state.next()

    def calculate_differential(self, measurements: list[(float, float)]) -> (float, float):
        temp_samples = [measurements[0][1], measurements[0][1], measurements[0][1]]
        prev_time = 0.0

        rate_fastest = 0.0
        temp_fastest = 0.0
        time_fastest = 0.0

        for (read_time, temp) in measurements:
            time_diff = read_time - prev_time

            if prev_time == 0.0:
                time_diff = DEFAULT_CYCLE_TIME

            # Measure rate of change of heating for differential tuning
            temp_samples[0] = temp_samples[1]
            temp_samples[1] = temp_samples[2]
            temp_samples[2] = temp

            # Measure the rate of change of temperature, https://en.wikipedia.org/wiki/Symmetric_derivative
            current_rate = (temp_samples[2] - temp_samples[0]) / (2.0 * time_diff)

            if current_rate > rate_fastest:
                rate_fastest = current_rate
                temp_fastest = temp_samples[1]
                time_fastest = read_time

        # calculate the constants:

        block_heat_capacity = self.mpc.data.heater_power / rate_fastest
        sensor_responsiveness = rate_fastest / (rate_fastest * time_fastest + self.mpc.ambient_temp - time_fastest)

        self.respond_info(
            "differential tuning results:\n"
            f"rate_fastest: {rate_fastest:.04f}\n"
            f"temp_fastest: {temp_fastest:.04f}\n"
            f"time_fastest: {time_fastest:.04f}\n"
            f"block_heat_capacity: {block_heat_capacity:.04f}\n"
            f"sensor_responsiveness: {sensor_responsiveness:.04f}\n"
        )

        return (block_heat_capacity, sensor_responsiveness)

    def stabilize_system(self, read_time, temp, target_temp):
        time_diff = read_time - self.mpc.prev_temp_time
        if not self.heating:
            self.respond_info(f"Stabilizing System: target={self.mpc.block_temp}")
            self.heating = True
            # control_mpc will set the pwm now:
            self.control_temp.set_heater_pwm(read_time, None)

            # error if the calculated target is too high or too low:
            if self.mpc.block_temp < self.calibrate_temp or self.mpc.block_temp >= self.heater.max_temp:
                raise self.control_temp.error(f"the calculated target temperature '{self.mpc.block_temp}' must be greater than {self.calibrate_temp:.02f} and lower than {self.heater.max_temp:.02f}.")

            # use the estimated overshoot of the temperature as the target to achieve
            self.heater.alter_target(self.mpc.block_temp)

            # update the mpc values:
            self.mpc.block_temp = temp
            self.mpc.sensor_temp = temp

            self.wait_for_settle = True
            self.max_sample_count = int(math.ceil(20.0 / time_diff))
            self.sample_count = 0

            self.fan0_done = False
            self.total_energy_fan0 = 0.0
            self.fan0_measurements = 0.0
            self.total_energy_fan255 = 0.0
            self.fan255_measurements = 0.0

            self.fan0_ambient_temp = 0.0
            self.fan255_ambient_temp = 0.0
            self.original_ambient_temp = self.mpc.ambient_temp

            self.last_temp = temp

            self.settle_window = []
            self.wait_until_time = None

            self.heater.set_pwm(read_time, 0.0)
            return

        # Delegate to the MPC algorithm:
        self.control_mpc.temperature_update(read_time, temp, target_temp)

        if self.wait_until_time is not None and read_time < self.wait_until_time:
            return
        elif self.wait_until_time is not None and read_time >= self.wait_until_time:
            self.wait_until_time = None

        if self.wait_for_settle and not has_settled_at_target(read_time, temp, target_temp, self.settle_window, window_size=30, tolerance=0.1):
            return
        elif self.wait_for_settle:
            self.respond_info(f"[{read_time:.3f}] Algorithm settled around target {target_temp:.2f}, starting measurements...")
            self.wait_for_settle = False
            self.settle_window = []

        heater_pwm = self.control_temp.get_heater_pwm()

        # This state uses the previously calculated values as base for the mpc control algorithm.
        # It will heat to some target temperature and after some time has passed, it should stabilize
        # around the target temperature.
        #
        # When it stabilized, there is some power percentage like 43% that will maintain the target
        # temperature. This is measured in this test (power_fan0, when the fan is off
        # and power_fan255 when it is on)

        # Check if the test is over:
        if self.sample_count >= self.max_sample_count and (self.fan0_done or not self.mpc.include_fan):
            self.respond_info(f"[{read_time:.3f}] Finished measuring power requirements.")
            # calculate how much power is required to maintain the target temperature with the fan on/off
            self.power_fan0 = self.total_energy_fan0 / self.fan0_measurements

            if self.fan255_measurements != 0.0:
                self.power_fan255 = self.total_energy_fan255 / self.fan255_measurements
            else:
                self.power_fan255 = self.power_fan0

            self.respond_info(f"[{read_time:.3f}] power_fan0 = {self.power_fan0:.03f}, power_fan255 = {self.power_fan255:.03f}")

            # turn off the fan:
            if self.mpc.include_fan:
                self.control_temp.set_fan_speed(read_time, 0.0)

            self.respond_info(f"Finished stabilizing system. Now calculating final values.")
            self.state = self.state.next()
            return

        if self.sample_count < self.max_sample_count and not self.fan0_done:
            # original calculation:
            # self.total_energy_fan0 += self.mpc.data.heater_power * heater_pwm * time_diff
            #                        + (self.last_temp - temp) * self.mpc.data.block_heat_capacity
            # where W * percentage * s + temp_diff °C * J/K
            #
            # self.total_energy_fan0 += self.mpc.data.heater_power * heater_pwm * time_diff + (self.last_temp - temp) * self.mpc.data.block_heat_capacity

            # This calculates the average PWM value that is required to maintain the target temperature
            # when the model has stabilized.
            #
            # TODO: how much of a difference is there, when you use the old formula?
            self.total_energy_fan0 += self.mpc.data.heater_power * heater_pwm * time_diff + (self.last_temp - temp) * self.mpc.data.block_heat_capacity
            self.fan0_measurements += time_diff
            self.sample_count += 1
            self.fan0_ambient_temp = self.mpc.ambient_temp
        elif self.sample_count < self.max_sample_count and self.mpc.include_fan:
            self.total_energy_fan255 += self.mpc.data.heater_power * heater_pwm * time_diff + (self.last_temp - temp) * self.mpc.data.block_heat_capacity
            self.fan255_measurements += time_diff
            self.sample_count += 1
            self.fan255_ambient_temp = self.mpc.ambient_temp

        if self.sample_count >= self.max_sample_count and self.mpc.include_fan and not self.fan0_done:
            self.respond_info(f"[{read_time:.3f}] Now measuring power requirements with fan on.")
            self.control_temp.set_fan_speed(read_time, 1.0)
            self.fan0_done = True
            self.wait_for_settle = True
            self.settle_window = []
            self.sample_count = 0
            # it takes a bit of time for the fan to start spinning, wait a bit for the algorithm
            # to take into account the fan:
            self.wait_until_time = read_time + 20.0

        self.last_temp = temp

    def determine_heatloss(self, read_time, temp, target_temp):
        old_xfer = self.mpc.data.ambient_xfer_coeff_fan0
        self.mpc.data.ambient_xfer_coeff_fan0 = self.power_fan0 / (target_temp - self.fan0_ambient_temp)
        self.respond_info(f"fan0_ambient_temp = {self.fan0_ambient_temp:.05f}, ambient_temp = {self.mpc.ambient_temp:.05f}, original_ambient_temp = {self.original_ambient_temp:.05f}")
        self.respond_info(f"ambient_xfer_coeff_fan0 = {self.mpc.data.ambient_xfer_coeff_fan0}")
        self.mpc.data.ambient_xfer_coeff_fan0 = old_xfer * (target_temp - self.fan0_ambient_temp) / (target_temp - self.original_ambient_temp)
        self.respond_info(f"ambient_xfer_coeff_fan0 = {self.mpc.data.ambient_xfer_coeff_fan0}")

        if self.mpc.include_fan:
            # self.mpc.data.ambient_xfer_coeff_fan255 = self.power_fan255 / (target_temp - self.mpc.ambient_temp)
            # self.mpc.data.ambient_xfer_coeff_fan255 = old_xfer * (target_temp - self.fan255_ambient_temp) / (target_temp - self.original_ambient_temp)
            self.mpc.data.ambient_xfer_coeff_fan255 = self.power_fan255 / (target_temp - self.mpc.ambient_temp)

        # TODO: recalculate the constants?

        if self.tuning_type == MPCTuningType.AUTO or self.tuning_type == MPCTuningType.FORCE_ASYMPTOTIC:
            # Calculate a new and better asymptotic temperature and re-evaluate the other constants

            asymp_temp = self.mpc.ambient_temp + self.mpc.data.heater_power / self.mpc.data.ambient_xfer_coeff_fan0
            block_responsiveness = (-math.log((self.t3 - asymp_temp) / (self.t1 - asymp_temp))) / (self.t3_time - self.t1_time)

            # Update analytic tuning values based on the above
            self.mpc.data.block_heat_capacity = self.mpc.data.ambient_xfer_coeff_fan0 / block_responsiveness
            self.mpc.data.sensor_responsiveness = block_responsiveness / (1.0 - (self.mpc.ambient_temp - asymp_temp) * math.exp(-block_responsiveness * self.t1_time) / (self.t1 - asymp_temp))

        # Tuning is over, move into None state:
        self.state = None

    def temperature_update(self, read_time, temp, target_temp):
        try:
            self.control_temp.record(read_time, temp, target_temp)
            states = {
                ControlAutoTuneState.DETERMINE_AMBIENT_TEMPERATURE: self.determine_ambient_temperature,
                ControlAutoTuneState.MEASURE_HEATING: self.measure_heating,
                ControlAutoTuneState.STABILIZE_SYSTEM: self.stabilize_system,
                ControlAutoTuneState.DETERMINE_HEATLOSS: self.determine_heatloss,
            }

            # The time_diff is used in multiple calculations. When the prev_temp_time is 0.0,
            # the script has not been run before. This could result in wrong calculations, therefore
            # it is manually set to 0.3s in the past (how long a klipper cycle takes).
            if self.mpc.prev_temp_time == 0.0:
                self.mpc.prev_temp_time = max(0.0, read_time - DEFAULT_CYCLE_TIME)

            if self.state is not None:
                states[self.state](read_time, temp, target_temp)
                self.mpc.prev_temp_time = read_time

            self.control_temp.update_heater(read_time, temp, target_temp)
        except Exception as e:
            # Coding mistakes that result in a crash can happen.
            #
            # For some reason klippy does not forcibly turn off the printer,
            # instead the exception will be logged and everything resumes like
            # nothing happend. You might not even be aware that something has
            # crashed.
            #
            # This is very dangerous. For example an exception could be raised
            # while the printer is currently heating up. There will be nothing
            # to stop the printer from continuing to heat.
            #
            # TODO: add this to mpc_control as well?
            # TODO: how about limiting the message size so it fits comfortably in the console?
            def wrap_line(line: str) -> list[str]:
                return textwrap.wrap(line, width=52, tabsize=4, replace_whitespace=False, break_long_words=False)

            def wrap_multiline(lines: list[str]) -> str:
                result = []

                current = ""
                for line in lines:
                    if len(line.strip()) == 0:
                        if len(current) > 0:
                            result.extend(wrap_line(current))
                            current = ""

                        result.append("")
                    else:
                        if len(current) == 0:
                            current = line.strip()
                        else:
                            current += " " + line.strip()

                if len(current) > 0:
                    result.extend(wrap_line(current))

                return '\n'.join(result)

            short_message = f"error: {str(e)}"
            message = traceback.format_exc()
            self.respond_info(wrap_multiline(message.splitlines()))

            self.printer.invoke_shutdown(short_message)
            raise self.printer.command_error(str(e))


    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.state is not None

def load_config(config):
    return MPCCalibrate(config)
