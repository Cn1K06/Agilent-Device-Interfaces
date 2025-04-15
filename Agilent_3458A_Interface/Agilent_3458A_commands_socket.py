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
            self.sock.sendall(f"{command}\n".encode('ascii'))
        except (socket.error, socket.timeout):
            print("Connection lost, attempting to reconnect...")
            self.sock.connect((self.ip_address, self.port))
            self.sock.sendall(f"{command}\n".encode('ascii'))

    def _clear_buffer(self):
        try:
            self.sock.setblocking(0)
            while True:
                data = self.sock.recv(1024)
                if not data:
                    break
        except BlockingIOError:
            pass
        finally:
            self.sock.setblocking(1)

    def _query(self, command):
        self._clear_buffer()
        self._send_command(command)
        return self.sock.recv(1024).decode('ascii').strip()

    def get_identity(self):
        return self._query('ID?')

    def reset(self):
        self._send_command('RESET')

    def clear_status(self):
        self._send_command('*CLS')

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
        self._send_command('TARM SGL')
        time.sleep(0.1)
        return self._query('DCV?')

    def measure_ac_voltage(self):
        self.configure_ac_voltage()
        self._send_command('TARM SGL')
        time.sleep(0.1)
        return self._query('ACV?')

    def measure_dc_current(self):
        self.configure_dc_current()
        self._send_command('TARM SGL')
        time.sleep(0.1)
        return self._query('DCI?')

    def measure_ac_current(self):
        self.configure_ac_current()
        self._send_command('TARM SGL')
        time.sleep(0.1)
        return self._query('ACI?')

    def measure_resistance(self):
        self.configure_resistance()
        self._send_command('TARM SGL')
        time.sleep(0.1)
        return self._query('OHMF?')

    def measure_frequency(self):
        self.configure_frequency()
        self._send_command('TARM SGL')
        time.sleep(0.1)
        return self._query('FREQ?')

    def enable_auto_zero(self, enable=True):
        self._send_command('AZERO ON' if enable else 'AZERO OFF')
        print(f"Auto zero {'enabled' if enable else 'disabled'}.")

    def calibrate(self, measurement_type):
        print(f"Starting calibration for {measurement_type}...")
        if measurement_type == "DCV":
            self._send_command('ACAL 1')
            time.sleep(3 * 60)
        elif measurement_type == "AC":
            self._send_command('ACAL 2')
            time.sleep(3 * 60)
        elif measurement_type == "OHMS":
            self._send_command('ACAL 4')
            time.sleep(12 * 60)
        elif measurement_type == "ALL":
            self._send_command('ACAL 0')
            time.sleep(16 * 60)
        else:
            print("Invalid measurement type for calibration.")

    def handle_dc_voltage_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "DC Voltage"
        self._perform_measurement(self.measure_dc_voltage, "Voltage")

    def handle_ac_voltage_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "AC Voltage"
        self._perform_measurement(self.measure_ac_voltage, "Voltage")

    def handle_dc_current_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "DC Current"
        self._perform_measurement(self.measure_dc_current, "Current")

    def handle_ac_current_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "AC Current"
        self._perform_measurement(self.measure_ac_current, "Current")

    def handle_resistance_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "Resistance"
        self._perform_measurement(self.measure_resistance, "Resistance")

    def handle_frequency_measurement(self):
        self._stop_event.clear()
        self.measurement_type = "Frequency"
        self._perform_measurement(self.measure_frequency, "Frequency")

    def _perform_measurement(self, measure_func, data_label):
        self._measurement_data = []
        try:
            while not self._stop_event.is_set():
                measurement = measure_func()
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
                self._measurement_data.append((timestamp, measurement))
                print(f"{timestamp} - {data_label}: {measurement.strip()}")
                time.sleep(self.measurement_frequency)
        except KeyboardInterrupt:
            print(f"{data_label} measurement stopped by user.")

    def stop_measurement(self):
        self._stop_event.set()

    def save_measurements(self, filename):
        measurement_type = self.measurement_type if self.measurement_type else "Measurement"
        with open(filename, "w") as file:
            file.write(f"Time,{measurement_type}\n")
            for timestamp, measurement in self._measurement_data:
                file.write(f"{timestamp}, {measurement.strip()}\n")
        print(f"Measurements saved to {filename}.")

    def close(self):
        self.sock.close()
