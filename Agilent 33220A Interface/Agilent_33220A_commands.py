import time

class WaveformGenerator:
    def __init__(self, instrument):
        self.instrument = instrument

    def get_identity(self):
        return self.instrument.query('*IDN?')

    def reset(self):
        self.instrument.write('*RST')

    def clear_status(self):
        self.instrument.write('*CLS')

    def enable_output(self):
        self.instrument.write("OUTP ON")
        print("Output enabled.")

    def disable_output(self):
        self.instrument.write("OUTP OFF")
        print("Output disabled.")

    def configure_sin_wave(self, frequency=1000, amplitude=1):
        self.instrument.write("FUNC SIN")
        self.instrument.write(f"FREQ {frequency}")
        self.instrument.write(f"VOLT {amplitude}")
        print("Sin wave configured.")

    def configure_square_wave(self, frequency=1000, amplitude=1, duty_cycle=50):
        self.instrument.write("FUNC SQU")
        self.instrument.write(f"FREQ {frequency}")
        self.instrument.write(f"VOLT {amplitude}")
        self.instrument.write(f"FUNC:SQU:DCYC {duty_cycle}")
        print("Square wave configured.")

    def configure_ramp_wave(self, frequency=1000, amplitude=1, symmetry=100):
        self.instrument.write("FUNC RAMP")
        self.instrument.write(f"FREQ {frequency}")
        self.instrument.write(f"VOLT {amplitude}")
        instrument.write(f"FUNC:RAMP:SYMM {symmetry}")
        print("Ramp wave configured.")

    def configure_pulse_wave(self, frequency=1000, amplitude=1, width=1e-3, edge_time=50e-9):
        self.instrument.write("FUNC PULS")
        self.instrument.write(f"FREQ {frequency}")
        self.instrument.write(f"VOLT {amplitude}")
        self.instrument.write(f"PULS:WIDTH {width}")
        self.instrument.write(f"PULS:TRAN {edge_time}")
        print("Pulse wave configured.")

    def configure_noise_wave(self, amplitude=1):
        self.instrument.write("FUNC NOI")  # Set waveform to noise
        self.instrument.write(f"VOLT {amplitude}")
        print("Noise wave configured.")

    def configure_arbitrary_wave(self, frequency=1000, amplitude=1, waveform_points=None):
        self.instrument.write("FUNC ARB")
        self.instrument.write(f"FREQ {frequency}")
        self.instrument.write(f"VOLT {amplitude}")
        
        # Example arbitrary waveform if none is provided
        if waveform_points is None:
            waveform_points = [0, 0.5, 1, 0.5, 0, -0.5, -1, -0.5]  # Basic custom waveform pattern
        
        # Convert points to string format for transmission
        points_string = ", ".join(map(str, waveform_points))
        self.instrument.write(f"DATA:ARB {points_string}")
        print("Arbitrary wave configured.")



    def configure_am_modulation(self, carrier_waveform, modulating_waveform, carrier_freq=1000, carrier_amp=1, mod_freq=100, depth=50):
        self.instrument.write(f"FUNC {carrier_waveform}")  # Set carrier waveform based on user input
        self.instrument.write(f"FREQ {carrier_freq}")
        self.instrument.write(f"VOLT {carrier_amp}")
        self.instrument.write("AM:STATE ON")
        self.instrument.write("AM:SOUR INT")  # Internal modulation
        self.instrument.write(f"AM:INT:FUNC {modulating_waveform}")
        self.instrument.write(f"AM:FREQ {mod_freq}")
        self.instrument.write(f"AM:DEPTH {depth}")
        print(f"AM modulation configured with {carrier_waveform} carrier.")

    def configure_fm_modulation(self, carrier_waveform, modulating_waveform, carrier_freq=1000, carrier_amp=1, mod_freq=100, deviation=100):
        self.instrument.write(f"FUNC {carrier_waveform}")
        self.instrument.write(f"FREQ {carrier_freq}")
        self.instrument.write(f"VOLT {carrier_amp}")
        self.instrument.write("FM:STATE ON")
        self.instrument.write("FM:SOUR INT")  # Internal modulation
        self.instrument.write(f"FM:INT:FUNC {modulating_waveform}")
        self.instrument.write(f"FM:FREQ {mod_freq}")
        self.instrument.write(f"FM:DEV {deviation}")
        print(f"FM modulation configured with {carrier_waveform} carrier.")

    def configure_pm_modulation(self, carrier_waveform, modulating_waveform, carrier_freq=1000, carrier_amp=1, mod_freq=100, deviation=30):
        self.instrument.write(f"FUNC {carrier_waveform}")
        self.instrument.write(f"FREQ {carrier_freq}")
        self.instrument.write(f"VOLT {carrier_amp}")
        self.instrument.write("PM:STATE ON")
        self.instrument.write("PM:SOUR INT")  # Internal modulation
        self.instrument.write(f"PM:INT:FUNC {modulating_waveform}")
        self.instrument.write(f"PM:FREQ {mod_freq}")
        self.instrument.write(f"PM:DEV {deviation}")
        print(f"PM modulation configured with {carrier_waveform} carrier.")

    def configure_fsk_modulation(self, carrier_waveform, carrier_freq=1000, carrier_amp=1, shift_freq=1200, rate=100):
        self.instrument.write(f"FUNC {carrier_waveform}")
        self.instrument.write(f"FREQ {carrier_freq}")
        self.instrument.write(f"VOLT {carrier_amp}")
        self.instrument.write("FSK:STATE ON")
        self.instrument.write("FSK:SOUR INT")  # Internal modulation
        self.instrument.write(f"FSK:FREQ {shift_freq}")
        self.instrument.write(f"FSK:RATE {rate}")
        print(f"FSK modulation configured with {carrier_waveform} carrier.")


    def configure_pwm_modulation(self, carrier_waveform, modulating_waveform, carrier_freq=1000, carrier_amp=1, mod_freq=100, deviation=50):
        self.instrument.write(f"FUNC {carrier_waveform}")
        self.instrument.write(f"FREQ {carrier_freq}")
        self.instrument.write(f"VOLT {carrier_amp}")
        self.instrument.write("PWM:STATE ON")
        self.instrument.write("PWM:SOUR INT")     # Internal modulation
        self.instrument.write(f"PWM:INT:FUNC {modulating_waveform}")
        self.instrument.write(f"PWM:FREQ {mod_freq}")
        self.instrument.write(f"PWM:DEV {deviation}")
        print("PWM modulation configured.")



    def configure_sweep(self, start_freq, stop_freq, sweep_time, sweep_type="LIN"):
        self.instrument.write("SWE:STATE ON")
        self.instrument.write("FUNC SIN")  # Set waveform to sine for sweep
        self.instrument.write(f"FREQ:STAR {start_freq}")  # Start frequency
        self.instrument.write(f"FREQ:STOP {stop_freq}")  # Stop frequency
        self.instrument.write(f"SWE:TIME {sweep_time}")  # Sweep time
        if sweep_type == "LIN":
            self.instrument.write("SWE:SPAC LIN")  # Linear sweep
        elif sweep_type == "LOG":
            self.instrument.write("SWE:SPAC LOG")  # Logarithmic sweep

    def configure_burst(self, carrier_freq, carrier_amp, burst_count, burst_period, trigger_source="INT"):
        self.instrument.write("BURST:STATE ON")
        self.instrument.write("FUNC SIN")  # Set waveform to sine for burst
        self.instrument.write(f"FREQ {carrier_freq}")  # Carrier frequency
        self.instrument.write(f"VOLT {carrier_amp}")  # Carrier amplitude
        self.instrument.write(f"BURST:NCYC {burst_count}")  # Burst cycles per burst
        self.instrument.write(f"BURST:INT:PER {burst_period}")  # Burst period
        if trigger_source == "INT":
            self.instrument.write("BURST:TRIG:SOUR INT")  # Internal trigger
        elif trigger_source == "EXT":
            self.instrument.write("BURST:TRIG:SOUR EXT")  # External trigger