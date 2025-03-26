import pyvisa
import time
import threading
from A34110A_commands import DMMCommands

# Initialize the VISA resource manager
rm = pyvisa.ResourceManager()

# List all connected devices
resources = rm.list_resources()
print("Available resources:", resources)
resource = resources[0]  # Get the resource
# Try to communicate with the resource
try:
    instrument = rm.open_resource(resource)
    dmm = DMMCommands(instrument)
    
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

    # Calibrate the instrument
    dmm.zero_calibration()
    print("Instrument calibrated")
    time.sleep(1)
    
    # Function to start measurement in a separate thread
    def start_measurement(command):
        if command == "DC":
            dmm.handle_dc_voltage_measurement()
        elif command == "AC":
            dmm.handle_ac_voltage_measurement()
        elif command == "CURR_DC":
            dmm.handle_dc_current_measurement()
        elif command == "CURR_AC":
            dmm.handle_ac_current_measurement()
        elif command == "RES":
            dmm.handle_resistance_measurement()
        elif command == "CAP":
            dmm.handle_capacitance_measurement()
        elif command == "TEMP":
            dmm.handle_temperature_measurement()
        elif command == "FREQ":
            dmm.handle_frequency_measurement()
    measurement_thread = None
    while True:
        command = input("Enter command (DC/AC/CURR_DC/CURR_AC/RES/CAP/TEMP/FREQ/CALIBRATE/STOP/EXIT): ").strip().upper()
        
        if command in ["DC", "AC", "CURR_DC", "CURR_AC", "RES", "CAP", "TEMP", "FREQ"]:
            if measurement_thread is None or not measurement_thread.is_alive():
                measurement_thread = threading.Thread(target=start_measurement, args=(command,))
                measurement_thread.start()
            else:
                print("Measurement is already running. Please stop it before starting a new one.")
        elif command == "CALIBRATE":
            calib_type = input("Enter the type of calibration (DC/AC/CURR_DC/CURR_AC/RES/CAP/TEMP/FREQ): ").strip().upper()
            dmm.calibrate(calib_type)
        elif command == "STOP":
            if measurement_thread and measurement_thread.is_alive():
                dmm.stop_measurement()
                measurement_thread.join()
                print("Measurement stopped.")
                save = input("Do you want to save the measurements? (yes/no): ").strip().lower()
                if save == "yes":
                    filename = input("Enter filename (e.g., dc_voltage.txt): ").strip()
                    dmm.save_measurements(filename)
                else:
                    print("Measurements not saved.")
            else:
                print("No measurement is currently running.")
        elif command == "EXIT":
            print("Exiting the program.")
            if measurement_thread and measurement_thread.is_alive():
                dmm.stop_measurement()
                measurement_thread.join()
                
            break  # Exit the while loop to terminate the program
        else:
            print("Invalid command. Please enter one of the following: DC, AC, CURR_DC, CURR_AC, RES, CAP, TEMP, FREQ, CALIBRATE, STOP.")

except Exception as e:
    print(f"Could not communicate with {resource}: {e}")