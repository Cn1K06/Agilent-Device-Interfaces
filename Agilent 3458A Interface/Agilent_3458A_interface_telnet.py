import telnetlib
import time
import threading
from Agilent_3458A_commands_telnet import DMMCommands

# Define the IP address and port of the Agilent 3458A
ip_address = '172.17.2.21'
port = 1234

# Create a Telnet connection to the device
try:
    with telnetlib.Telnet(ip_address, port) as telnet:
        dmm = DMMCommands(telnet)

        # Print instrument identity
        print(f"Instrument identity: {dmm.get_identity()}")
        time.sleep(1)

        # Clear the instrument status
        dmm.clear_status()
        print("Status cleared")
        time.sleep(1)

        # Reset the instrument
        dmm.reset()
        print("Instrument reset")
        time.sleep(1)

        # Ask whether calibration should be performed
        perform_calibration = input("Do you want to perform calibration? (yes/no): ").strip().lower()

        if perform_calibration == "yes":
            calib_type = input("Enter the type of calibration (ALL/DCV/AC/OHMS): ").strip().upper()
            dmm.calibrate(calib_type)

        # "Enable auto zero" ensures that any offset errors internal to the multimeter are
        # nulled from subsequent DC or ohms measurements. 
        # Enable auto zero after calibration
        auto_zero_choice = input("Enable auto zero before measurement? (yes/no): ").strip().lower()
        if auto_zero_choice == "yes":
            dmm.enable_auto_zero(True)
            print("Internal Offset errors will be nulled from subsequent DC or Ohms measurements")
        else:
            dmm.enable_auto_zero(False)

        stop_measurement_flag = threading.Event()  # Flag to stop measurement

        def start_measurement(command, num_readings):
            """Start a measurement based on the command."""
            stop_measurement_flag.clear()  # Clear the stop flag before starting measurement

            if command == "DC":
                dmm.measurement_type = "DC Voltage"
                dmm._perform_measurement(dmm.measure_dc_voltage, "DC Voltage", num_readings, stop_measurement_flag)
            elif command == "AC":
                dmm.measurement_type = "AC Voltage"
                dmm._perform_measurement(dmm.measure_ac_voltage, "AC Voltage", num_readings, stop_measurement_flag)
            elif command == "CURR_DC":
                dmm.measurement_type = "DC Current"
                dmm._perform_measurement(dmm.measure_dc_current, "DC Current", num_readings, stop_measurement_flag)
            elif command == "CURR_AC":
                dmm.measurement_type = "AC Current"
                dmm._perform_measurement(dmm.measure_ac_current, "AC Current", num_readings, stop_measurement_flag)
            elif command == "RES":
                dmm.measurement_type = "Resistance"
                dmm._perform_measurement(dmm.measure_resistance, "Resistance", num_readings, stop_measurement_flag)
            elif command == "FREQ":
                dmm.measurement_type = "Frequency"
                dmm._perform_measurement(dmm.measure_frequency, "Frequency", num_readings, stop_measurement_flag)
            print('Measurement completed.')

        def prompt_save():
            """Ask if the user wants to save the measurements after measurement completion."""
            save = input("Do you want to save the measurements? (yes/no): ").strip().lower()
            if save == "yes":
                filename = input("Enter filename (e.g., measurements.txt): ").strip()
                dmm.save_measurements(filename)
            else:
                print("Measurements not saved.")

        measurement_thread = None
        num_readings = None  # Initialize the number of readings

        while True:
            command = input("Enter command (DC/AC/CURR_DC/CURR_AC/RES/FREQ/STOP/EXIT): ").strip().upper()

            if command in ["DC", "AC", "CURR_DC", "CURR_AC", "RES", "FREQ"]:
                if measurement_thread is None or not measurement_thread.is_alive():
                    num_readings = int(input(f"Enter number of readings for {command}: ").strip())
                    measurement_thread = threading.Thread(target=start_measurement, args=(command, num_readings))
                    measurement_thread.start()
                    measurement_thread.join()  # Wait for the measurement thread to finish
                    prompt_save()  # Ask for saving once measurement is complete
                else:
                    print("Measurement is already running. Please stop it before starting a new one.")

            elif command == "STOP":
                if measurement_thread and measurement_thread.is_alive():
                    stop_measurement_flag.set()  # Signal the thread to stop the measurement
                    measurement_thread.join()  # Wait for the thread to finish
                    print("Measurement stopped.")
                    prompt_save()  # Prompt to save immediately after stopping
                else:
                    print("No measurement is currently running.")

            elif command == "EXIT":
                print("Exiting the program.")
                if measurement_thread and measurement_thread.is_alive():
                    stop_measurement_flag.set()  # Signal the thread to stop the measurement
                    measurement_thread.join()  # Ensure the measurement thread finishes before exiting
                break

            else:
                print("Invalid command. Please enter a valid command.")

except Exception as e:
    print(f"Failed to communicate with the device: {e}")
