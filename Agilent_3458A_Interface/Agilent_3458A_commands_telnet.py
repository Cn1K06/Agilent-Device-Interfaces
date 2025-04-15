import telnetlib
import time
import threading

class DMMCommands:
    def __init__(self, telnet_connection, measurement_frequency=1):
        self.telnet = telnet_connection
        self._stop_event = threading.Event()
        self._measurement_data = []
        self.measurement_type = None
        self.measurement_frequency = measurement_frequency

    def send_command(self, command):
        """Send a command via Telnet and return the response."""
        self.telnet.write(f"{command}\n".encode('ascii'))
        return self.telnet.read_until(b'\n', timeout=3).decode('ascii').strip()

    def get_identity(self):
        """Query the instrument identity."""
        return self.send_command('ID?')

    def reset(self):
        """Reset the instrument."""
        self.send_command('RESET')

    def clear_status(self):
        """Clear the instrument status."""
        self.send_command('*CLS')

    def configure_dc_voltage(self):
        """Configure for DC Voltage measurement."""
        self.send_command('DCV AUTO')

    def configure_ac_voltage(self):
        """Configure for AC Voltage measurement."""
        self.send_command('ACV AUTO')

    def configure_dc_current(self):
        """Configure for DC Current measurement."""
        self.send_command('DCI AUTO')

    def configure_ac_current(self):
        """Configure for AC Current measurement."""
        self.send_command('ACI AUTO')

    def configure_resistance(self):
        """Configure for Resistance measurement."""
        self.send_command('OHM AUTO')

    def configure_frequency(self):
        """Configure for Frequency measurement."""
        self.send_command('FREQ')

    def measure_dc_voltage(self):
        """Measure DC Voltage."""
        self.configure_dc_voltage()
        return self.send_command('TARM SGL, DCV?')

    def measure_ac_voltage(self):
        """Measure AC Voltage."""
        self.configure_ac_voltage()
        return self.send_command('TARM SGL, ACV?')

    def measure_dc_current(self):
        """Measure DC Current."""
        self.configure_dc_current()
        return self.send_command('TARM SGL, DCI?')

    def measure_ac_current(self):
        """Measure AC Current."""
        self.configure_ac_current()
        return self.send_command('TARM SGL, ACI?')

    def measure_resistance(self):
        """Measure Resistance."""
        self.configure_resistance()
        return self.send_command('TARM SGL, OHM?')

    def measure_frequency(self):
        """Measure Frequency."""
        self.configure_frequency()
        return self.send_command('TARM SGL, FREQ?')

    def enable_auto_zero(self, enable=True):
        """Enable or disable auto zero feature."""
        if enable:
            self.send_command('AZERO ON')
        else:
            self.send_command('AZERO OFF')
        print(f"Auto zero {'enabled' if enable else 'disabled'}.")

    def calibrate(self, measurement_type):
        print(f"Starting calibration for {measurement_type}...")
        if measurement_type == "DCV":
            print("Performing DC voltage calibration (~3 minutes)...")
            self.send_command('ACAL 1')  # Perform DC voltage calibration
            time.sleep(3 * 60)  # Wait 3 minutes for calibration
        elif measurement_type == "AC":
            print("Performing AC voltage calibration (~3 minutes)...")
            self.send_command('ACAL 2')  # Perform AC voltage calibration
            time.sleep(3 * 60)  # Wait 3 minutes for calibration
        elif measurement_type == "OHMS":
            print("Performing resistance calibration (~12 minutes)...")
            self.send_command('ACAL 4')  # Perform resistance calibration
            time.sleep(12 * 60)  # Wait 12 minutes for calibration
        elif measurement_type == "ALL":
            print("Performing full calibration (~16 minutes)...")
            self.send_command('ACAL 0')  # Perform full calibration
            time.sleep(16 * 60)  # Wait 16 minutes for calibration
        else:
            print("Invalid measurement type for calibration.")
            return

    def _perform_measurement(self, measure_func, data_label, stop_flag):
        """Perform the continuous measurement until stopped."""
        self._measurement_data = []  # Clear previous measurement data
        try:
            while not stop_flag.is_set():  # Check if the stop flag is set
                measurement = measure_func()
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
                self._measurement_data.append((timestamp, measurement))
                print(f"{timestamp} - {data_label}: {measurement}")
                time.sleep(self.measurement_frequency)  # Control measurement frequency
        except KeyboardInterrupt:
            print(f"{data_label} measurement stopped by user.")

    def save_measurements(self, filename):
        # Use the measurement type set during the measurement process
        measurement_type = self.measurement_type if self.measurement_type else "Measurement"

        with open(filename, "w") as file:
            file.write(f"Time,{measurement_type}\n")  # Header line with automatically determined measurement type
            for timestamp, measurement in self._measurement_data:
                file.write(f"{timestamp}, {measurement.strip()}\n")  # Write the timestamp and measurement
        print(f"Measurements saved to {filename}.")



    def stop_measurement(self):
        """Stop the current measurement."""
        self._stop_event.set()

    def close(self):
        """Close the Telnet connection."""
        self.telnet.close()
