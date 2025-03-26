import time
import threading
import socket

class DMMCommands:
    def __init__(self, ip_address, port, measurement_frequency=0.5):
        self.ip_address = ip_address
        self.port = port
        self._stop_event = threading.Event()
        self._measurement_data = []
        self.measurement_type = None
        self.measurement_frequency = measurement_frequency
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.ip_address, self.port))

    def _send_command(self, command):
        try:
            self.sock.sendall(f"{command}\n".encode('ascii'))  # Send the command
        except (socket.error, socket.timeout):
            print("Connection lost, attempting to reconnect...")
            self.sock.connect((self.ip_address, self.port))  # Reconnect if necessary
            self.sock.sendall(f"{command}\n".encode('ascii'))  # Retry sending the command

    def _query(self, command):
        self._send_command(command)
        return self.sock.recv(1024).decode('ascii').strip()

    def get_identity(self):
        return self._query('ID?')

    def reset(self):
        self._send_command('RESET')

    def clear_status(self):
        self._send_command('*CLS')  # Clear the instrument status

    def configure_dc_voltage(self):
        self._send_command('DCV AUTO')

    def configure_ac_voltage(self):
        self._send_command('ACV AUTO')

    def configure_dc_current(self):
        self._send_command('DCI AUTO')

    def configure_ac_current(self):
        self._send_command('ACI AUTO')

    def configure_resistance(self):
        self._send_command('OHMF AUTO')

    def configure_frequency(self):
        self._send_command('FREQ AUTO')

    def measure_dc_voltage(self):
        self.configure_dc_voltage()
        return self._query('TARM SGL, DCV?')

    def measure_ac_voltage(self):
        self.configure_ac_voltage()
        return self._query('TARM SGL, ACV?')

    def measure_dc_current(self):
        self.configure_dc_current()
        return self._query('TARM SGL, DCI?')

    def measure_ac_current(self):
        self.configure_ac_current()
        return self._query('TARM SGL, ACI?')

    def measure_resistance(self):
        self.configure_resistance()
        return self._query('TARM SGL, OHMF?')

    def measure_frequency(self):
        self.configure_frequency()
        return self._query('TARM SGL, FREQ?')

    def stop_measurement(self):
        self._stop_event.set()

    def calibrate(self, measurement_type):
        print(f"Starting calibration for {measurement_type}...")
        if measurement_type == "DCV":
            print("Performing DC voltage calibration (~3 minutes)...")
            self._send_command('ACAL 1')  # Perform DC voltage calibration
            time.sleep(3 * 60)  # Wait 3 minutes for calibration
        elif measurement_type == "AC":
            print("Performing AC voltage calibration (~3 minutes)...")
            self._send_command('ACAL 2')  # Perform AC voltage calibration
            time.sleep(3 * 60)  # Wait 3 minutes for calibration
        elif measurement_type == "OHMS":
            print("Performing resistance calibration (~12 minutes)...")
            self._send_command('ACAL 4')  # Perform resistance calibration
            time.sleep(12 * 60)  # Wait 12 minutes for calibration
        elif measurement_type == "ALL":
            print("Performing full calibration (~16 minutes)...")
            self._send_command('ACAL 0')  # Perform full calibration
            time.sleep(16 * 60)  # Wait 16 minutes for calibration
        else:
            print("Invalid measurement type for calibration.")
            return

    def handle_dc_voltage_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "DC Voltage"
        print("Configured for DC voltage measurement")
        self._perform_measurement(self.measure_dc_voltage, "Voltage")

    def handle_ac_voltage_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "AC Voltage"
        print("Configured for AC voltage measurement")
        self._perform_measurement(self.measure_ac_voltage, "Voltage")

    def handle_dc_current_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "DC Current"
        print("Configured for DC current measurement")
        self._perform_measurement(self.measure_dc_current, "Current")

    def handle_ac_current_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "AC Current"
        print("Configured for AC current measurement")
        self._perform_measurement(self.measure_ac_current, "Current")

    def handle_resistance_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "Resistance"
        print("Configured for resistance measurement")
        self._perform_measurement(self.measure_resistance, "Resistance")

    def handle_frequency_measurement(self):
        self._stop_event.clear()
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
                print(f"{timestamp} - {data_label}: {measurement}")
                time.sleep(self.measurement_frequency)
        except KeyboardInterrupt:
            print(f"{data_label} measurement stopped by user.")

    def save_measurements(self, filename):
        measurement_type = self.measurement_type if self.measurement_type else "Measurement"
        with open(filename, "w") as file:
            file.write(f"Time,{measurement_type}\n")
            for timestamp, measurement in self._measurement_data:
                file.write(f"{timestamp}, {measurement.strip()}\n")
        print(f"Measurements saved to {filename}.")

    def close(self):
        self.sock.close()
