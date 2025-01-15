import pyvisa
import time
from Agilent_33220A_commands import WaveformGenerator

# Initialize the VISA resource manager
rm = pyvisa.ResourceManager()

# List all connected devices and select the Agilent 33220A
resources = rm.list_resources()
print("Available resources:", resources)
resource = resources[0]  # Adjust this if multiple devices are connected

try:
    instrument = rm.open_resource(resource)
    waveform_gen = WaveformGenerator(instrument)
    
    # Print instrument identity
    print(f"Instrument identity: {waveform_gen.get_identity()}")
    time.sleep(1)

    # Reset the instrument
    waveform_gen.reset()
    print("Instrument reset")
    time.sleep(1)

    # Clear the instrument status
    waveform_gen.clear_status()
    print("Status cleared")
    time.sleep(1)


    while True:
        command = input("Enter waveform type (SIN/SQUARE/RAMP/PULSE/NOISE/ARB/MODULATION/SWEEP/BURST) or 'EXIT' to quit: ").strip().upper()
        
        if command == "SIN":
            freq = float(input("Enter frequency (Hz): "))
            amp = float(input("Enter amplitude (Vpp): "))
            waveform_gen.configure_sin_wave(frequency=freq, amplitude=amp)
            waveform_gen.enable_output()
        
        elif command == "SQUARE":
            freq = float(input("Enter frequency (Hz): "))
            amp = float(input("Enter amplitude (Vpp): "))
            duty_cycle = float(input("Enter duty cycle (%): "))
            waveform_gen.configure_square_wave(frequency=freq, amplitude=amp, duty_cycle=duty_cycle)
            waveform_gen.enable_output()

        elif command == "RAMP":
            freq = float(input("Enter frequency (Hz): "))
            amp = float(input("Enter amplitude (Vpp): "))
            symm = float(input("Enter symmetry (%): "))
            waveform_gen.configure_ramp_wave(frequency=freq, amplitude=amp, symmetry = symm)
            waveform_gen.enable_output()

        elif command == "PULSE":
            freq = float(input("Enter frequency (Hz): "))
            amp = float(input("Enter amplitude (Vpp): "))
            width = float(input("Enter pulse width (s): "))
            edge_time = float(input("Enter edge time (s): "))
            waveform_gen.configure_pulse_wave(frequency=freq, amplitude=amp, width=width, edge_time=edge_time)
            waveform_gen.enable_output()

        elif command == "NOISE":
            amp = float(input("Enter amplitude (Vpp): "))
            waveform_gen.configure_noise_wave(amplitude=amp)
            waveform_gen.enable_output()

        elif command == "ARB":
            freq = float(input("Enter frequency (Hz): "))
            amp = float(input("Enter amplitude (Vpp): "))
            points = input("Enter arbitrary waveform points separated by commas (e.g., 0,0.5,1,0.5,0): ")
            points_list = [float(x) for x in points.split(',')]
            waveform_gen.configure_arbitrary_wave(frequency=freq, amplitude=amp, waveform_points=points_list)
            waveform_gen.enable_output()

        elif command == "MODULATION":
            mod_type = input("Enter modulation type (AM/FM/PM/FSK/PWM): ").strip().upper()

            # Carrier waveform selection based on modulation type
            if mod_type == "PWM":
                print("PWM modulation requires PULSE waveform as the carrier.")
                carrier_waveform = "PULS"  # Force PULSE for PWM
                modulating_waveform = input("Choose modulating waveform (SIN/SQUARE/RAMP/TRIANGLE/NRAMP): ").strip().upper()
                # Validate and map the modulating waveform2
                if modulating_waveform not in ["SIN", "SQUARE", "RAMP", "TRIANGLE","NRAMP"]:
                    print("Invalid carrier waveform. Defaulting to SIN.")
                    modulating_waveform = "SIN"
                else:
                    modulating_waveform_map = {
                        "SIN": "SIN",
                        "SQUARE": "SQU",
                        "RAMP": "RAMP",
                        "TRIANGLE": "TRI",
                        "NRAMP": "NRAM"
                    }
                    modulating_waveform = modulating_waveform_map[modulating_waveform]

            elif mod_type == "FSK":
                print("FSK modulation selected. Modulating waveform is not needed for FSK.")
                carrier_waveform = input("Choose carrier waveform (SIN/SQUARE/RAMP): ").strip().upper()
                
                # Validate and map the carrier waveform for FSK
                if carrier_waveform not in ["SIN", "SQUARE", "RAMP"]:
                    print("Invalid carrier waveform. Defaulting to SIN.")
                    carrier_waveform = "SIN"
                else:
                    waveform_map = {
                        "SIN": "SIN",
                        "SQUARE": "SQU",
                        "RAMP": "RAMP"
                    }
                    carrier_waveform = waveform_map[carrier_waveform]


            else:
                carrier_waveform = input("Choose carrier waveform (SIN/SQUARE/RAMP): ").strip().upper()
                modulating_waveform = input("Choose modulating waveform (SIN/SQUARE/RAMP/TRIANGLE/NRAMP): ").strip().upper()

                # Validate and map the carrier waveform
                if carrier_waveform not in ["SIN", "SQUARE", "RAMP"]:
                    print("Invalid carrier waveform. Defaulting to SIN.")
                    carrier_waveform = "SIN"
                else:
                    waveform_map = {
                        "SIN": "SIN",
                        "SQUARE": "SQU",
                        "RAMP": "RAMP"
                    }
                    carrier_waveform = waveform_map[carrier_waveform]

                # Validate and map the modulating waveform
                if modulating_waveform not in ["SIN", "SQUARE", "RAMP", "TRIANGLE","NRAMP"]:
                    print("Invalid carrier waveform. Defaulting to SIN.")
                    modulating_waveform = "SIN"
                else:
                    modulating_waveform_map = {
                        "SIN": "SIN",
                        "SQUARE": "SQU",
                        "RAMP": "RAMP",
                        "TRIANGLE": "TRI",
                        "NRAMP": "NRAM"
                    }
                    modulating_waveform = modulating_waveform_map[modulating_waveform]


            carrier_freq = float(input("Enter carrier frequency (Hz): "))
            carrier_amp = float(input("Enter carrier amplitude (Vpp): "))

            if mod_type == "AM":
                mod_freq = float(input("Enter modulation frequency (Hz): "))
                depth = float(input("Enter modulation depth (%): "))
                waveform_gen.configure_am_modulation(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, depth)

            elif mod_type == "FM":
                mod_freq = float(input("Enter modulation frequency (Hz): "))
                deviation = float(input("Enter frequency deviation (Hz): "))
                waveform_gen.configure_fm_modulation(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, deviation)

            elif mod_type == "PM":
                mod_freq = float(input("Enter modulation frequency (Hz): "))
                deviation = float(input("Enter phase deviation (degrees): "))
                waveform_gen.configure_pm_modulation(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, deviation)

            elif mod_type == "FSK":
                shift_freq = float(input("Enter shift frequency (Hz): "))
                rate = float(input("Enter FSK rate (Hz): "))
                waveform_gen.configure_fsk_modulation(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, shift_freq, rate)

            elif mod_type == "PWM":
                mod_freq = float(input("Enter modulation frequency (Hz): "))
                deviation = float(input("Enter pulse width deviation (%): "))
                waveform_gen.configure_pwm_modulation(carrier_waveform, modulating_waveform, carrier_freq, carrier_amp, mod_freq, deviation)
            else:
                print("Invalid modulation type.")
            waveform_gen.enable_output()
        
        elif command == "SWEEP":
            start_freq = float(input("Enter start frequency in Hz:"))
            stop_freq = float(input("Enter stop frequency in Hz: "))
            sweep_time = float(input("Enter sweep time in seconds: "))
            sweep_type = input("Enter sweep type (LIN for linear, LOG for logarithmic): ").upper()
            waveform_gen.configure_sweep(start_freq=start_freq, stop_freq=stop_freq, sweep_time=sweep_time, sweep_type=sweep_type)
            waveform_gen.enable_output()

        elif command == "BURST":
            # Burst configuration
            carrier_freq = float(input("Enter carrier frequency (Hz): "))
            carrier_amp = float(input("Enter carrier amplitude (Vpp): "))
            burst_count = int(input("Enter number of cycles per burst: "))
            burst_period = float(input("Enter burst period in seconds: "))
            trigger_source = input("Enter trigger source (INT for internal, EXT for external): ").upper()
            waveform_gen.configure_burst(carrier_freq=carrier_freq, carrier_amp=carrier_amp, burst_count=burst_count, burst_period=burst_period, trigger_source=trigger_source)
            waveform_gen.enable_output()

        elif command == "EXIT":
            print("Exiting program.")
            break

        else:
            print("Invalid command. Please enter one of the following: SIN, SQUARE, RAMP, PULSE, NOISE, ARB, EXIT.")

    waveform_gen.disable_output()

    # Reset the instrument
    waveform_gen.reset()
    print("Instrument reset")
    time.sleep(1)
except Exception as e:
    print(f"Could not communicate with {resource}: {e}")
