import time
from Agilent_3458A_commands_socket import DMMCommands

# Set IP and Port for the Agilent 3458A via Prologix adapter
ip_address = '172.17.2.21'
port = 1234

measurement_frequency = 1  # Frequency of measurement can be changed here

try:
    # Initialize the DMMCommands object with socket communication
    dmm = DMMCommands(ip_address, port, measurement_frequency)

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

    # Now continue to measurements
    while True:
        command = input("Enter command (DC/AC/CURR_DC/CURR_AC/RES/TEMP/FREQ/STOP/EXIT): ").strip().upper()

        if command in ["DC", "AC", "CURR_DC", "CURR_AC", "RES", "FREQ"]:
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
            elif command == "FREQ":
                dmm.handle_frequency_measurement()

        elif command == "STOP":
            dmm.stop_measurement()
            print("Measurement stopped.")
            save = input("Do you want to save the measurements? (yes/no): ").strip().lower()
            if save == "yes":
                filename = input("Enter filename (e.g., measurements.txt): ").strip()
                dmm.save_measurements(filename)
            else:
                print("Measurements not saved.")

        elif command == "EXIT":
            print("Exiting the program.")
            break

        else:
            print("Invalid command. Please enter a valid command (DC/AC/CURR_DC/CURR_AC/RES/CAP/TEMP/FREQ/STOP/EXIT).")

except Exception as e:
    print(f"Could not communicate with the device: {e}")
finally:
    dmm.close()
