import time
import telnetlib


class Counter_Commands:
    def __init__(self, ip_address, port):
        """
        Initialize the MeasurementDevice with IP address and port.
        """
        self.ip_address = ip_address
        self.port = port
        self.tn = None

    def send_command(self, command):
        """
        Send a command via Telnet and return the response.
        """
        try:
            self.tn.write(f"{command}\n".encode('ascii'))
            response = self.tn.read_until(b"\n", timeout=3).decode('ascii').strip()
            return response
        except Exception as e:
            print(f"Failed to send command: {e}")
            return None

    def get_identity(self):
        """
        Query the instrument's identity.
        """
        return self.send_command('*IDN?')

    def reset_instrument(self):
        """
        Reset the instrument.
        """
        self.send_command('*RST')
        print("Instrument reset complete.")

    def clear_status(self):
        """
        Clear the instrument status.
        """
        self.send_command('*CLS')
        print("Status cleared.")

    def measure_frequency(self, channel):
        """
        Measure frequency on a specific channel.
        """
        command = f"MEAS:FREQ? (@{channel})"
        return self._log_measurement(channel, "Frequency", command, "Hz")

    def measure_period(self, channel):
        """
        Measure period on a specific channel.
        """
        command = f"MEAS:PER? (@{channel})"
        return self._log_measurement(channel, "Period", command, "s")

    def measure_duty_cycle(self, channel):
        """
        Measure duty cycle on a specific channel.
        """
        command = f"MEAS:DCYC? (@{channel})"
        return self._log_measurement(channel, "Duty Cycle", command, "%")

    def measure_pulse_width_pos(self, channel):
        """
        Measure positive pulse width on a specific channel.
        """
        command = f"MEAS:PWID? (@{channel})"
        return self._log_measurement(channel, "Positive Pulse Width", command, "s")

    def measure_pulse_width_neg(self, channel):
        """
        Measure negative pulse width on a specific channel.
        """
        command = f"MEAS:NWID? (@{channel})"
        return self._log_measurement(channel, "Negative Pulse Width", command, "s")

    def measure_rise_time(self, channel):
        """
        Measure rise time on a specific channel.
        """
        command = f"MEAS:RTIM? (@{channel})"
        return self._log_measurement(channel, "Rise Time", command, "s")

    def measure_fall_time(self, channel):
        """
        Measure fall time on a specific channel.
        """
        command = f"MEAS:FTIM? (@{channel})"
        return self._log_measurement(channel, "Fall Time", command, "s")

    def configure_input(self, channel):
        """
        Configure input settings for a specific channel.
        """
        print(f"Configuring Input Channel {channel}...")
        coupling = input(f"Select coupling for Channel {channel} (AC/DC): ").strip().upper()
        coupling = coupling if coupling in ['AC', 'DC'] else 'DC'
        self.send_command(f"INP{channel}:COUP {coupling}")

        impedance = input(f"Set impedance for Channel {channel} (50/1000000): ").strip()
        impedance = impedance if impedance in ['50', '1000000'] else '50'
        self.send_command(f"INP{channel}:IMP {impedance}")

        filter_state = input(f"Enable filter for Channel {channel} (ON/OFF): ").strip().upper()
        filter_state = filter_state if filter_state in ['ON', 'OFF'] else 'OFF'
        self.send_command(f"INP{channel}:FILT {filter_state}")

        attenuation = input(f"Set attenuation for Channel {channel} (1/10): ").strip()
        attenuation = attenuation if attenuation in ['1', '10'] else '1'
        self.send_command(f"INP{channel}:ATT {attenuation}")

        slope = input(f"Set slope for Channel {channel} (POS/NEG): ").strip().upper()
        slope = slope if slope in ['POS', 'NEG'] else 'POS'
        self.send_command(f"INP{channel}:SLOP {slope}")
        print(f"Channel {channel} configured.")

    def list_files_on_device(self):
        """
        List all files stored in the device's memory.
        """
        command = ":MMEM:CATalog?"
        response = self.send_command(command)
        if response:
            print("Files on device:")
            print(response)
        else:
            print("No files found or failed to retrieve file list.")

    def _log_measurement(self, channel, measurement_name, command, unit):
        """
        Helper function to perform and log measurements.
        """
        result = self.send_command(command)
        if result:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            print(f"[{timestamp}] {measurement_name} on Channel {channel}: {result} {unit}")
        return result

    def enable_math(self):
        """Ensure MATH mode is enabled."""
        #command = "CALC:MATH:STAT ON"
        command = f":CALC1:MATH:STATE ON"
        response = self.send_command(command)
        print(f"Math mode enabled.")
        return response

    def process_math(self):
        """# Process results using scale and offset."""
        command =f":CALC:IMM:AUTO ON" 
        response = self.send_command(command)
        return response


    def enable_statistics(self, channel):
        """Enable statistics mode for the specified channel."""
        command = f":CALC{channel}:STAT:STAT ON"
        response = self.send_command(command)
        print(f"Statistics mode enabled for Channel {channel}.")
        return response

    def get_statistics(self, channel, stat_type):
        """Retrieve statistical data (mean, std, min, max)."""
        # Ensure statistics mode is enabled
        self.enable_statistics(channel)

        # Fetch the requested statistical data
        stat_command = {
            "MEAN": f":CALC{channel}:STAT:MEAN?",
            "STDDEV": f":CALC{channel}:STAT:STD?",
            "MIN": f":CALC{channel}:STAT:MIN?",
            "MAX": f":CALC{channel}:STAT:MAX?",
        }.get(stat_type.upper())
        if stat_command:
            return self.send_command(stat_command)
        return "Invalid statistic type."

        
    def set_scale(self, channel, scale_factor):
        """Set a scale factor for a channel."""
        try:
            #command = f":TRACE SCALE, {scale_factor}" 
            command = f":CALC:MATH:SCAL {scale_factor}"
            return self.send_command(command)
        except Exception as e:
            print(f"Failed to send command: {e}")
            return None        


    def set_offset(self, channel, offset_value):
        """Set an offset for a channel."""
        #command = f":TRACE OFFSET, {offset_value}"
        command = f":CALC:MATH:OFFS {offset_value}"
        return self.send_command(command)

    def enable_limit_test(self, channel, upper_limit, lower_limit):
        """Enable limit testing for a channel."""
        self.send_command(f":CALC{channel}:LIM:UPP {upper_limit}")
        self.send_command(f":CALC{channel}:LIM:LOW {lower_limit}")
        return self.send_command(f":CALC{channel}:LIM:STAT ON")

    def disable_limit_test(self, channel):
        """Disable limit testing for a channel."""
        return self.send_command(f":CALC{channel}:LIM:STAT OFF")

    def save_measurements(self, memory_location):
 
        try:
            # Ensure the memory location is within the valid range
            if not (1 <= memory_location <= 20):
                raise ValueError("Memory location must be between 1 and 20.")
        
            command = f"*SAV {memory_location}"
            response = self.send_command(command)
            print(f"Settings saved to memory location {memory_location}.")
            return response
        except Exception as e:
            print(f"Failed to save measurements: {e}")
            return None


