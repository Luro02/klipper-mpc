import math

from enum import Enum, auto
import numpy as np

from .mpc_control import MPC, ControlMPC, ControlTemperature

# TODO: use this function for finding t1, t2, t3
def interp(x: float, fp: list[(float, float)]) -> float:
    xs, ys = zip(*fp)

    return float(np.interp(x, xs, ys))

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
        gcode.register_command('MPC_ENABLE', self.cmd_MPC_ENABLE, desc=self.cmd_MPC_CALIBRATE_help)

    cmd_MPC_ENABLE_help = "Enable MPC algorithm for heating"
    def cmd_MPC_ENABLE(self, gcmd):
        heater_name = gcmd.get('HEATER', default="extruder")
        
        # find the options for the heater:
        try:
            mpc_section = self.printer.lookup_object('mpc').lookup_object(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))

        self.printer.lookup_object('toolhead').get_last_move_time()

        heater = mpc_section.heater

        heater.set_control(ControlMPC(heater, mpc_section.config, mpc=mpc_section.mpc))

        gcmd.respond_info(f"Enabled MPC for {mpc_section.heater_name}, restart klipper to disable it.")

    cmd_MPC_CALIBRATE_help = "Run MPC calibration test"
    def cmd_MPC_CALIBRATE(self, gcmd):
        heater_name = gcmd.get('HEATER', default="extruder")
        target = 200.0
        if heater_name != "extruder":
            target = gcmd.get_float('TARGET', default=200.0)

        write_file = gcmd.get_int('WRITE_FILE', 0) == 1
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

        # #define MPC_TUNING_POS { X_CENTER, Y_CENTER, 1.0f } // (mm) M306 Autotuning position, ideally bed center at first layer height.
        # #define MPC_TUNING_END_Z 10.0f                      // (mm) M306 Autotuning final Z position.

        # precondition for tuning is that the heater is off and part cooling fan as well:
        # TODO: raise an error if the fan is on or just turn it off?

        if heater_name == "extruder":
            gcode = self.printer.lookup_object('gcode')
            # TODO: make this position configurable?
            gcode.run_script_from_command(f"G28\nG0 X128 Y128 Z1")

        (ambient_temp, target_temp) = heater.get_temp(eventtime)
        if target_temp > 0.:
            # TODO: enable automatic cooldown
            raise gcmd.error("Heater must be cold")

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

        if write_file:
            calibrate.write_file('/tmp/heattest.txt')
        if calibrate.check_busy(0., 0., 0.):
            raise gcmd.error("mpc_calibrate interrupted")

        # Save MPC values and report them on the command line:
        mpc.save(config, log=True)


class ControlAutoTuneState(Enum):
    DETERMINE_AMBIENT_TEMPERATURE = auto()
    DETERMINE_PHYSICAL_CONSTANTS = auto()
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
        self.temp_samples = [0] * 16
        self.sample_count = 0

        # How often samples should be collected (in seconds)
        self.test_sample_time = 1.0
        # When the last temperature value has been read
        self.next_test_time = 0.0
        self.sample_distance = 1

        self.measurements = []

        self.state = ControlAutoTuneState.DETERMINE_AMBIENT_TEMPERATURE

    def respond_info(self, message):
        self.control_temp.log(message)

    def determine_ambient_temperature(self, read_time, temp, target_temp):
        # TODO: This code does not work reliably. It does not really cooldown the printer fully to ambient temperature
        #       Cooling it down entirely could take forever as well. At least for testing, it makes sense to hardcode
        #       a sensible default value like 25°C. The extruder must be below 100°C and for differential tuning it
        #       should be even lower.

        if self.mpc.ambient_temp < 30.0:
            self.heater.alter_target(0.0)
            self.state = self.state.next()
            return

        # if not already done, start cooldown

        # On the first call the target_temp, would be somewhere around ~200°C.
        # Before heating to that temperature, it is important to figure out the
        # room temperature.
        #
        # The target_temp is set by this method to 1°C, because then the heater fan
        # stays on. This speeds up finding the ambient temperature.
        if target_temp > 1.0:
            self.respond_info("Setting target temp to 1.0 for finding ambient temperature")
            self.heater.alter_target(1.)
            # Indicate that we do not want to heat:
            self.control_temp.set_heater_pwm(read_time, 0.0)

            # Measuring the ambient temperature every time this method is called, might result
            # in the condition being true, because of sensor fluctuations. For example the
            # current temperature could be 50.0°C, then on the next invocation the temperature
            # is 50.1°C
            # => 50.0°C is the lowest temperature, therefore it must be the room temperature
            #
            # To prevent this, only every second a measurement is done.
            # This ensures that there is enough time for the fans to turn on.
            self.next_measurement = read_time + 1.0

            # enable fan for faster cooldown
            self.control_temp.set_fan_speed(read_time, 1.0)
            return

        if self.next_measurement > read_time:
            return

        # When the printer is cooling down, it never reaches 0°C, but stops at
        # around room temperature. This is called the ambient temperature.
        #
        # While the printer is cooling, one only gets the current temperature. The
        # next temperature could be even lower or not.
        #
        # To determine if ambient temperature is reached, self.mpc.ambient_temp
        # is used to keep track of what the previous temperature was.

        # When room temperature is reached, the current measurement should not be lower
        # than the last measured temperature (otherwise that would be the ambient_temperature).
        if temp >= self.mpc.ambient_temp or math.fabs(temp - self.mpc.ambient_temp) <= 0.2:
            # The current measurement is slightly higher than the previously measured temperature.
            # The middle of the two measurements will be used as the ambient temperature.
            self.mpc.ambient_temp = (self.mpc.ambient_temp + temp) / 2.0
            self.control_temp.set_fan_speed(read_time, 0.0)
            # advance to the next stage of the calibration:
            self.state = self.state.next()
            self.heater.alter_target(0.)

            # ensure that the ambient temp is not above 30°C:
            self.mpc.ambient_temp = min(self.mpc.ambient_temp, 30.0)

            self.respond_info(f"Found ambient_temp={self.mpc.ambient_temp}")

            return

        self.next_measurement = read_time + 1.0
        self.mpc.ambient_temp = temp

    def determine_physical_constants(self, read_time, temp, target_temp):
        # When this function is called, the printer is at ambient temperature (target set to 0):
        if target_temp == 0.0:
            self.respond_info(f"Measuring physical constants...")
            self.control_temp.set_heater_pwm(read_time, 1.0)
            self.heater.alter_target(self.calibrate_temp)
            self.heating = True

            # control_temp will take the measurements:
            self.control_temp.start_recording(read_time)
            # record the time at which it started heating
            self.heat_start_time = read_time

            # TODO: could these be removed?
            self.temp_samples[0] = temp
            self.temp_samples[1] = temp
            self.temp_samples[2] = temp

            self.time_fastest = 0.0
            self.rate_fastest = 0.0
            self.temp_fastest = 0.0
            return

        # The marlin implementation only records a temperature sample every second
        # and limits the total amount of samples to at most 16.
        #
        # This seems to be, because of memory constraints. This restriction does not
        # apply to klippy, therefore all encountered samples are recorded.

        # TODO: what happens when you disable this timer?
        if read_time < self.next_test_time:
            return

        self.next_test_time = read_time + self.test_sample_time

        if temp < 100.0:
            # Measure rate of change of heating for differential tuning

            self.temp_samples[0] = self.temp_samples[1]
            self.temp_samples[1] = self.temp_samples[2]
            self.temp_samples[2] = temp

            # Measure the rate of change of temperature, https://en.wikipedia.org/wiki/Symmetric_derivative
            # TODO: test_sample_time might be wrong
            # TODO: check unit of h, currently in seconds
            h = self.test_sample_time
            current_rate = (self.temp_samples[2] - self.temp_samples[0]) / (2.0 * h)

            if current_rate > self.rate_fastest:
                self.rate_fastest = current_rate
                self.temp_fastest = self.temp_samples[1]
                self.time_fastest = read_time - self.heat_start_time

            return
        elif temp < target_temp:
            # Measure 3 points to determine asymptotic temperature

            # If there are too many samples, space them more widely
            if self.sample_count == len(self.temp_samples):
                for i in range(len(self.temp_samples) // 2):
                    self.temp_samples[i] = self.temp_samples[i * 2]
                self.sample_count /= 2
                self.sample_distance *= 2

            if self.sample_count == 0:
                self.t1_time = read_time - self.heat_start_time
                self.control_temp.start_recording(self.t1_time + self.heat_start_time)
                self.respond_info(f"t1_time = {self.t1_time}")

            self.temp_samples[self.sample_count] = temp
            self.sample_count += 1
            self.t3_time = read_time - self.heat_start_time

            self.next_test_time += self.test_sample_time * self.sample_distance

            return

        # finished collecting samples, can now calculate the physical constants:

    	# TODO: one could use linear interpolation between the sample points and then define t1_time = 0, t2_time = end // 2, t3_time = end if even or end - 1 if odd
        samples = self.control_temp.stop_recording(self.t1_time + self.heat_start_time)
        # TODO: this contains every sample recorded since it started heating
        self.control_temp.stop_recording(self.heat_start_time)
        # TODO: is the recording correctly implemented?

        self.respond_info("Finished collecting data for calculating constants. Temporarily setting pwm to 0.0")
        # turn off the heater
        self.control_temp.set_heater_pwm(read_time, 0.0)
        self.heating = False

        self.respond_info(f"heat_start_time = {self.heat_start_time}")

        elapsed_heating_time = read_time - self.heat_start_time

        self.respond_info(f"elapsed_heating_time = {elapsed_heating_time}")

        if self.sample_count == 0:
            # TODO: raise correct error
            self.respond_info("Failed to collect samples")
            raise ValueError("Failed to collect samples")
        if self.sample_count % 2 == 0:
            self.sample_count -= 1

        self.t1 = self.temp_samples[0]
        self.t2 = self.temp_samples[(self.sample_count - 1) // 2]
        self.t3 = self.temp_samples[self.sample_count - 1]

        self.respond_info(f"t1={self.t1}, t2={self.t2}, t3={self.t3}")
        elapsed_time = self.t3_time - self.t1_time
        self.respond_info(f"elapsed_time={elapsed_time}")

        asymp_temp = (self.t2 * self.t2 - self.t1 * self.t3) / (2 * self.t2 - self.t1 - self.t3)
        # block_responsiveness = -log((t2 - asymp_temp) / (t1 - asymp_temp)) / tuner.get_sample_interval();
        # elapsed_time was (self.sample_distance * (self.sample_count // 2))
        block_responsiveness = (-math.log((self.t3 - asymp_temp) / (self.t1 - asymp_temp))) / elapsed_time

        # Make initial guess at transfer coefficients
        ambient_xfer_coeff_fan0 = self.mpc.data.heater_power / (asymp_temp - self.mpc.ambient_temp)

        if self.tuning_type == MPCTuningType.AUTO or self.tuning_type == MPCTuningType.FORCE_ASYMPTOTIC:
            # Analytic tuning
            block_heat_capacity = ambient_xfer_coeff_fan0 / block_responsiveness
            # TODO: most likely the * t1_time is wrong here!?
            # TODO: ^ how does the corresponding formula look?
            # TODO: if it is wrong, determine_heatloss code must be updated as well
            sensor_responsiveness = block_responsiveness / (1.0 - (self.mpc.ambient_temp - asymp_temp) * math.exp(-block_responsiveness * self.t1_time) / (self.t1 - asymp_temp))

            self.respond_info(f"block_heat_capacity={block_heat_capacity}\nsensor_responsiveness={sensor_responsiveness}")

        # If analytic tuning fails, fall back to differential tuning
        if self.tuning_type == MPCTuningType.AUTO and (sensor_responsiveness <= 0 or block_heat_capacity <= 0):
            self.tuning_type = MPCTuningType.FORCE_DIFFERENTIAL
            self.respond_info("Analytic tuning failed, using different calibration method.")

        if self.tuning_type == MPCTuningType.FORCE_DIFFERENTIAL:
            # differential tuning
            block_heat_capacity = self.mpc.data.heater_power / self.rate_fastest
            sensor_responsiveness = self.rate_fastest / (self.rate_fastest * self.time_fastest + self.mpc.ambient_temp - self.time_fastest)

            self.respond_info(f"block_heat_capacity={block_heat_capacity}\nsensor_responsiveness={sensor_responsiveness}")

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

    def stabilize_system(self, read_time, temp, target_temp):
        time_diff = read_time - self.mpc.prev_temp_time
        if not self.heating:
            self.respond_info(f"Stabilizing System: target={self.mpc.block_temp}")
            self.heating = True
            # control_mpc will set the pwm now:
            self.control_temp.set_heater_pwm(read_time, None)

            # Use the estimated overshoot of the temperature as the target to achieve.
            # TODO: error if it is not overshooting (happens when the ambient_temp is wrong)
            self.heater.alter_target(self.mpc.block_temp)

            self.next_test_time = read_time + time_diff
            self.settle_time = 20.0 # in seconds
            self.test_length = 20.0 # in seconds

            self.settle_end = read_time + self.settle_time
            self.test_end = self.settle_end + self.test_length

            self.fan0_done = False
            self.total_energy_fan0 = 0.0
            self.total_energy_fan255 = 0.0

            self.last_temp = temp

            self.heater.set_pwm(read_time, 0.0)
            return

        # Delegate to the current MPC algorithm:
        self.control_mpc.temperature_update(read_time, temp, target_temp)

        # do nothing until the next test time is reached:
        if read_time < self.next_test_time:
            return

        heater_pwm = self.control_temp.get_heater_pwm()

        if read_time >= self.settle_end and read_time < self.test_end and not self.fan0_done:
            self.total_energy_fan0 += self.mpc.data.heater_power * heater_pwm * time_diff + (self.last_temp - temp) * self.mpc.data.block_heat_capacity
        elif self.mpc.include_fan and read_time >= self.test_end and not self.fan0_done:
            self.control_temp.set_fan_speed(read_time, 1.0)
            self.settle_end = read_time + self.settle_time
            self.test_end = self.settle_end + self.test_length
            self.fan0_done = True
        elif self.mpc.include_fan and read_time >= self.settle_end and read_time < self.test_end:
            self.total_energy_fan255 += self.mpc.data.heater_power * heater_pwm * time_diff + (self.last_temp - temp) * self.mpc.data.block_heat_capacity
        elif read_time >= self.test_end:
            # calculate final values:
            self.power_fan0 = self.total_energy_fan0 / self.test_length
            self.power_fan255 = self.total_energy_fan255 / self.test_length

            self.respond_info(f"Finished stabilizing system. Now calculating final values.")
            self.state = self.state.next()

            return

        self.last_temp = temp
        self.next_test_time += time_diff

    def determine_heatloss(self, read_time, temp, target_temp):
        # TODO: should the heater be turned off?
        # The heater is no longer required, therefore it is turned off
        # self.control_temp.set_heater_pwm(read_time, 0.0)
        # self.heater.alter_target(0.)

        self.mpc.data.ambient_xfer_coeff_fan0 = self.power_fan0 / (target_temp - self.mpc.ambient_temp)
        if self.mpc.include_fan:
            self.mpc.data.ambient_xfer_coeff_fan255 = self.power_fan255 / (target_temp - self.mpc.ambient_temp)

        if self.tuning_type == MPCTuningType.AUTO or self.tuning_type == MPCTuningType.FORCE_ASYMPTOTIC:
            # Calculate a new and better asymptotic temperature and re-evaluate the other constants

            asymp_temp = self.mpc.ambient_temp + self.mpc.data.heater_power / self.mpc.data.ambient_xfer_coeff_fan0
            elapsed_time = self.t3_time - self.t1_time
            block_responsiveness = (-math.log((self.t3 - asymp_temp) / (self.t1 - asymp_temp))) / elapsed_time

            # Update analytic tuning values based on the above
            self.mpc.data.block_heat_capacity = self.mpc.data.ambient_xfer_coeff_fan0 / block_responsiveness
            self.mpc.data.sensor_responsiveness = block_responsiveness / (1.0 - (self.mpc.ambient_temp - asymp_temp) * math.exp(-block_responsiveness * self.t1_time) / (self.t1 - asymp_temp))

        # Tuning is over, move into None state:
        self.state = None

    def temperature_update(self, read_time, temp, target_temp):
        self.control_temp.record(read_time, temp, target_temp)
        if temp < 200.0 and temp > 100.0:
            self.measurements.append((read_time, temp))

        states = {
            ControlAutoTuneState.DETERMINE_AMBIENT_TEMPERATURE: self.determine_ambient_temperature,
            ControlAutoTuneState.DETERMINE_PHYSICAL_CONSTANTS: self.determine_physical_constants,
            ControlAutoTuneState.STABILIZE_SYSTEM: self.stabilize_system,
            ControlAutoTuneState.DETERMINE_HEATLOSS: self.determine_heatloss,
        }

        if self.state is not None:
            states[self.state](read_time, temp, target_temp)

            self.mpc.prev_temp_time = read_time

        self.control_temp.update_heater(read_time, temp, target_temp)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.state is not None

    # Offline analysis helper
    def write_file(self, filename):
        with open(filename, "w") as fd:
            fd.writelines([f"{k}={v}\n" for (k, v) in self.measurements])

def load_config(config):
    return MPCCalibrate(config)
