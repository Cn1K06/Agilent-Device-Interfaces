import telnetlib
from Agilent_53132A_commands import Counter_Commands
import time 

def main():
    ip_address = '172.17.2.21'
    port = 1234

    try:
        # Establish Telnet connection
        print(f"Connecting to {ip_address}:{port}...")
        tn = telnetlib.Telnet(ip_address, port)
        print("Connection established.")

        # Create an instance of MeasurementDevice using the Telnet connection
        device = Counter_Commands(ip_address, port)
        device.tn = tn  # Pass the Telnet connection to the class

        # Query device identity
        print("\nQuerying device identity...")
        identity = device.get_identity()
        print(f"Device Identity: {identity}")

        # Reset and clear the device status
        device.reset_instrument()
        print("\nResetting the device...")

        device.clear_status()
        print("Clearing the device status...")

        device.process_math()

        # User interaction loop
        while True:
            command = input(
                "\nEnter command (FREQ/PER/DCYC/PWID/NWID/RTIM/FTIM/RATIO/CONFIGURATE/LIST/MATH/EXIT): "
            ).strip().upper()
            if command == "EXIT":
                print("Exiting program...")
                break

            elif command in ["FREQ", "PER", "DCYC", "PWID", "NWID", "RTIM", "FTIM"]:
                channel = input("Enter the channel for measurement (1 or 2): ").strip()
                if channel not in ["1", "2"]:
                    print("Invalid channel. Please enter 1 or 2.")
                    continue
                mode = input("Measurement mode (SINGLE/CONTINUOUS): ").strip().upper()
                if mode == "SINGLE":
                    if command == "FREQ":
                        device.measure_frequency(channel)
                    elif command == "PER":
                        device.measure_period(channel)
                    elif command == "DCYC":
                        device.measure_duty_cycle(channel)
                    elif command == "PWID":
                        device.measure_pulse_width_pos(channel)
                    elif command == "NWID":
                        device.measure_pulse_width_neg(channel)
                    elif command == "RTIM":
                        device.measure_rise_time(channel)
                    elif command == "FTIM":
                        device.measure_fall_time(channel)
                elif mode == "CONTINUOUS":
                    try:
                        print("Press Ctrl+C to stop continuous measurement...")
                        while True:
                            if command == "FREQ":
                                device.measure_frequency(channel)
                            elif command == "PER":
                                device.measure_period(channel)
                            elif command == "DCYC":
                                device.measure_duty_cycle(channel)
                            elif command == "PWID":
                                device.measure_pulse_width_pos(channel)
                            elif command == "NWID":
                                device.measure_pulse_width_neg(channel)
                            elif command == "RTIM":
                                device.measure_rise_time(channel)
                            elif command == "FTIM":
                                device.measure_fall_time(channel)
                            time.sleep(1)  # Measuring interval (1 second)
                    except KeyboardInterrupt:
                        print("\nContinuous measurement stopped.")

                        stat_option = input("Do you want to retrieve statistics (MEAN/STDDEV/MIN/MAX)? (YES/NO): ").strip().upper()
                        if stat_option == "YES":
                            stat_type = input("Which statistic do you want to retrieve? (MEAN/STDDEV/MIN/MAX): ").strip().upper()
                            if stat_type in ["MEAN", "STDDEV", "MIN", "MAX"]:
                                stat_value = device.get_statistics(channel, stat_type)
                                print(f"{stat_type} value: {stat_value}")
                            else:
                                print("Invalid statistic type.")
                        else:
                            print("Statistics were not retrieved.")

                	    save_option = input("Do you want to save the measurements? (YES/NO): ").strip().upper()

                        if save_option == "YES":
                            memory_location = int(input("Enter memory location (1-20): ").strip())
                            save_result = device.save_measurements(memory_location)
                            if save_result is not None:
                                print(f"Measurements saved to memory location {memory_location}.")
                            else:
                                print("Failed to save measurements.")
                        else:
                            print("Measurements were not saved.")
                else:
                    print("Invalid mode. Please enter SINGLE or CONTINUOUS.")
            elif command == "RATIO":
                numerator_channel = input("Enter the numerator channel (1 or 2): ").strip()
                denominator_channel = input("Enter the denominator channel (1 or 2): ").strip()
                if numerator_channel in ["1", "2"] and denominator_channel in ["1", "2"]:
                    freq_num = device.measure_frequency(numerator_channel)
                    freq_den = device.measure_frequency(denominator_channel)
                    if freq_num and freq_den:
                        try:
                            ratio = float(freq_num) / float(freq_den)
                            print(f"Frequency Ratio (Channel {numerator_channel}/{denominator_channel}): {ratio}")
                        except ZeroDivisionError:
                            print("Error: Division by zero occurred.")
                    else:
                        print("Failed to measure frequencies.")
                else:
                    print("Invalid channel selection. Please enter 1 or 2.")

            elif command == "CONFIGURATE":
                channel = input("Enter the channel to configure (1 or 2): ").strip()
                if channel in ["1", "2"]:
                    device.configure_input(channel)
                else:
                    print("Invalid channel. Please enter 1 or 2.")

            elif command == "LIST":
                device.list_files_on_device()

            elif command == "MATH":
                device.enable_math()
                device.process_math()
                math_operation = input("Enter MATH Operation (SCALE/OFFSET/LIMIT/STATS): ").strip().upper()
                channel = input("Enter the channel (1 or 2): ").strip()

                if math_operation == "SCALE":
                    scale_factor = input("Enter scale factor: ").strip()
                    result = device.set_scale(channel, scale_factor)
                    print(f"Scale set: {result}")

                elif math_operation == "OFFSET":
                    offset_value = input("Enter offset value: ").strip()
                    result = device.set_offset(channel, offset_value)
                    print(f"Offset set: {result}")

                elif math_operation == "LIMIT":
                    limit_action = input("Enable or Disable Limit Test? (ENABLE/DISABLE): ").strip().upper()
                    if limit_action == "ENABLE":
                        upper_limit = input("Enter upper limit: ").strip()
                        lower_limit = input("Enter lower limit: ").strip()
                        result = device.enable_limit_test(channel, upper_limit, lower_limit)
                        print(f"Limit Test Enabled: {result}")
                    elif limit_action == "DISABLE":
                        result = device.disable_limit_test(channel)
                        print(f"Limit Test Disabled: {result}")

                elif math_operation == "STATS":
                    stat_type = input("Enter statistics type (MEAN/STDDEV/MIN/MAX): ").strip().upper()
                    result = device.get_statistics(channel, stat_type)
                    print(f"{stat_type} Result: {result}")

                else:
                    print("Invalid MATH operation.")
            else:
                print("Invalid command. Please try again.")

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Ensure proper disconnection
        if 'tn' in locals():
            tn.close()
            print("Telnet connection closed.")


if __name__ == "__main__":
    main()
