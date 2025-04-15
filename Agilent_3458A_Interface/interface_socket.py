import time
import threading
from Agilent_3458A_commands_socket import DMMCommands

ip_address = '172.17.2.21'
port = 1234
measurement_frequency = 1

try:
    dmm = DMMCommands(ip_address, port, measurement_frequency)

    print(f"Instrument identity: {dmm.get_identity()}")
    time.sleep(1)

    dmm.clear_status()
    print("Status cleared")
    time.sleep(1)

    dmm.reset()
    print("Instrument reset")
    time.sleep(1)

    perform_calibration = input("Do you want to perform calibration? (yes/no): ").strip().lower()
    if perform_calibration == "yes":
        calib_type = input("Enter the type of calibration (ALL/DCV/AC/OHMS): ").strip().upper()
        dmm.calibrate(calib_type)

    auto_zero_choice = input("Enable auto zero before measurement? (yes/no): ").strip().lower()
    dmm.enable_auto_zero(auto_zero_choice == "yes")

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
        elif command == "FREQ":
            dmm.handle_frequency_measurement()

    measurement_thread = None

    while True:
        command = input("Enter command (DC/AC/CURR_DC/CURR_AC/RES/FREQ/STOP/EXIT): ").strip().upper()
        if command in ["DC", "AC", "CURR_DC", "CURR_AC", "RES", "FREQ"]:
            if measurement_thread is None or not measurement_thread.is_alive():
                measurement_thread = threading.Thread(target=start_measurement, args=(command,))
                measurement_thread.start()
            else:
                print("Measurement is already running. Please stop it before starting a new one.")
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
            break
        else:
            print("Invalid command.")

except Exception as e:
    print(f"Could not communicate with the device: {e}")
finally:
    dmm.close()
