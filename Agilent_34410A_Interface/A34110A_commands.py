import time
import threading

class DMMCommands:
    def __init__(self, instrument):
        self.instrument = instrument
        self._stop_event = threading.Event()
        self._measurement_data = []
        self.measurement_type = None  # Initialize measurement type attribute

    def get_identity(self):
        return self.instrument.query('*IDN?')

    def reset(self):
        self.instrument.write('*RST')

    def clear_status(self):
        self.instrument.write('*CLS')

    def configure_dc_voltage(self):
        self.instrument.write('CONF:VOLT:DC')

    def configure_ac_voltage(self):
        self.instrument.write('CONF:VOLT:AC')

    def configure_dc_current(self):
        self.instrument.write('CONF:CURR:DC')

    def configure_ac_current(self):
        self.instrument.write('CONF:CURR:AC')

    def configure_resistance(self):
        self.instrument.write('CONF:RES')

    def configure_capacitance(self):
        self.instrument.write('CONF:CAP')

    def configure_temperature(self, probe_type="TC"):
        self.instrument.write(f'CONF:TEMP {probe_type}')

    def configure_frequency(self):
        self.instrument.write('CONF:FREQ')

    def zero_calibration(self):
        self.instrument.write('CAL:ZERO:ALL')

    def measure_dc_voltage(self):
        return self.instrument.query('MEAS:VOLT:DC?')

    def measure_ac_voltage(self):
        return self.instrument.query('MEAS:VOLT:AC?')

    def measure_dc_current(self):
        return self.instrument.query('MEAS:CURR:DC?')

    def measure_ac_current(self):
        return self.instrument.query('MEAS:CURR:AC?')

    def measure_resistance(self):
        return self.instrument.query('MEAS:RES?')

    def measure_capacitance(self):
        return self.instrument.query('MEAS:CAP?')

    def measure_temperature(self):
        return self.instrument.query('MEAS:TEMP?')

    def measure_frequency(self):
        return self.instrument.query('MEAS:FREQ?')

    def fetch_data(self):
        return self.instrument.query('FETC?')

    def fetch_status(self):
        return self.instrument.query('STAT:OPER:COND?')

    def initiate(self):
        self.instrument.write('INIT')

    def abort(self):
        self.instrument.write('ABOR')

    def stop_measurement(self):
        self._stop_event.set()

    def calibrate(self, measurement_type):
        print(f"Starting calibration for {measurement_type}...")
        if measurement_type == "DC":
            self.configure_dc_voltage()
        elif measurement_type == "AC":
            self.configure_ac_voltage()
        elif measurement_type == "CURR_DC":
            self.configure_dc_current()
        elif measurement_type == "CURR_AC":
            self.configure_ac_current()
        elif measurement_type == "RES":
            self.configure_resistance()
        elif measurement_type == "CAP":
            self.configure_capacitance()
        elif measurement_type == "TEMP":
            self.configure_temperature()
        elif measurement_type == "FREQ":
            self.configure_frequency()
        else:
            print("Invalid measurement type for calibration.")
            return

        self.zero_calibration()
        print(f"Calibration completed for {measurement_type}.")

    def handle_dc_voltage_measurement(self):
        self._stop_event.clear()
        self.calibrate("DC")
        self.measurement_type = "DC Voltage"
        print("Configured for DC voltage measurement")
        self._perform_measurement(self.measure_dc_voltage, "Voltage")

    def handle_ac_voltage_measurement(self):
        self._stop_event.clear()
        self.calibrate("AC")
        self.measurement_type = "AC Voltage"
        print("Configured for AC voltage measurement")
        self._perform_measurement(self.measure_ac_voltage, "Voltage")

    def handle_dc_current_measurement(self):
        self._stop_event.clear()
        self.calibrate("CURR_DC")
        self.measurement_type = "DC Current"
        print("Configured for DC current measurement")
        self._perform_measurement(self.measure_dc_current, "Current")

    def handle_ac_current_measurement(self):
        self._stop_event.clear()
        self.calibrate("CURR_AC")
        self.measurement_type = "AC Current"
        print("Configured for AC current measurement")
        self._perform_measurement(self.measure_ac_current, "Current")

    def handle_resistance_measurement(self):
        self._stop_event.clear()
        self.calibrate("RES")
        self.measurement_type = "Resistance"
        print("Configured for resistance measurement")
        self._perform_measurement(self.measure_resistance, "Resistance")

    def handle_capacitance_measurement(self):
        self._stop_event.clear()
        self.calibrate("CAP")
        self.measurement_type = "Capacitance"
        print("Configured for capacitance measurement")
        self._perform_measurement(self.measure_capacitance, "Capacitance")

    def handle_temperature_measurement(self):
        self._stop_event.clear()
        self.calibrate("TEMP")
        self.measurement_type = "Temperature"
        print("Configured for temperature measurement")
        self._perform_measurement(self.measure_temperature, "Temperature")

    def handle_frequency_measurement(self):
        self._stop_event.clear()
        self.calibrate("FREQ")
        self.measurement_type = "Frequency"
        print("Configured for frequency measurement")
        self._perform_measurement(self.measure_frequency, "Frequency")

    def _perform_measurement(self, measure_func, data_label):
        self._measurement_data = []  # Clear previous measurement data
        try:
            while not self._stop_event.is_set():
                measurement = measure_func()
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
                self._measurement_data.append((timestamp, measurement))
                print(f"{timestamp} - {data_label}: {measurement}", end="")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print(f"{data_label} measurement stopped by user.")

    def save_measurements(self, filename):
        # Use the measurement type set during the measurement process
        measurement_type = self.measurement_type if self.measurement_type else "Measurement"

        with open(filename, "w") as file:
            file.write(f"Time,{measurement_type}\n")  # Header line with automatically determined measurement type
            for timestamp, measurement in self._measurement_data:
                file.write(f"{timestamp} - {measurement.strip()}\n")  # Write the timestamp and measurement
        print(f"Measurements saved to {filename}.")
