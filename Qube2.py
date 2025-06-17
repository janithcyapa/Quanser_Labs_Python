import quanser.hardware as qh
from array import array
import time
from quanser.hardware import Clock


board_type = "qube_servo2_usb"
board_identifier = "0"

# https://docs.quanser.com/quarc/documentation/qube_servo2_usb.html
# https://docs.quanser.com/quarc/documentation/python/
# 1 single-ended analog input channel, 12-bit, ±3A, channel 0 - Current to the motor in Amperes. 
# 2 single-ended encoder input channels, 5V 
    # 0 - Motor position
    # 1 - Encoder 1 position
# 3 digital inputs
    # 0 - Amplifier fault
    # 2 - Motor stall error
    # 1 - Motor stall detected
# Other inputs - Tachometer for motor 0

# 1 single-ended analog output, ±15V, channel 0 - Drives the motor voltage
# 1 digital output - Enable Motor
# 3 other outputs to drive RGB LED panel
    # 11000 - Red (0 ~ 1)
    # 11001 - Green (0 ~ 1)
    # 11002 - Blue (0 ~ 1)

try:
    card = qh.HIL()
    card.open(board_type, board_identifier)
    print("Card is connected.")

    version = qh.HIL.get_version()
    print(f"Version: {version.major}.{version.minor}.{version.release} (Build {version.build})")
    
    is_valid = card.is_valid()
    print(f"Card is valid: {is_valid}")

    motor_current_channel = array('I', [0])
    num_motor_current_channels = len(motor_current_channel)
    motor_current_buffer = array('d', [0.0] * num_motor_current_channels) 

    encoder_channels = array('I', [0, 1])
    num_encoder_channels = len(encoder_channels)
    encoder_buffer = array('i', [0] * num_encoder_channels)
        
    # Read tachometer (other input), channel 0
    other_channels = array('I', [0])
    num_other_channels = len(other_channels)
    other_buffer = array('d', [0.0] * num_other_channels)

    # Read three digital inputs: 0 - Amplifier fault, 1 - Motor stall detected, 2 - Motor stall error
    digital_channels = array('I', [0, 1, 2])
    num_digital_channels = len(digital_channels)
    digital_buffer = array('b', [0] * num_digital_channels)

    LED_channels = array('i', [11000, 11001, 11002])
    num_LED_channels = len(LED_channels)
    other_buffer = array('d', [1.0, 0.0, 0.0])

    card.read_analog(motor_current_channel, num_motor_current_channels, motor_current_buffer)
    print(f"Motor current (A): {motor_current_buffer[0]}")
    card.read_digital(digital_channels, num_digital_channels, digital_buffer)
    print(f"Digital inputs: Amplifier fault={digital_buffer[0]}, Motor stall detected={digital_buffer[1]}, Motor stall error={digital_buffer[2]}")
    card.read_analog(other_channels, num_other_channels, other_buffer)
    print(f"Tachometer value: {other_buffer[0]}")
    card.read_encoder(encoder_channels, num_encoder_channels, encoder_buffer)
    print(f"Encoder 0 value: {encoder_buffer[0]}, Encoder 1 value: {encoder_buffer[1]}")


    # Write a pattern to the RGB LED panel using 'other' channels (11000: Red, 11001: Green, 11002: Blue)

    card.write_other(LED_channels, num_LED_channels, other_buffer)
    print("LED pattern written: Red=1, Green=0, Blue=1")

    # Read the digital output state for Motor Enable (typically channel 0)
    motor_enable_channel = array('I', [0])
    motor_enable_buffer = array('b', [1])
    card.write_digital(motor_enable_channel, 1, motor_enable_buffer)
    print(f"Motor Enable state: {motor_enable_buffer[0]}")
    

    # Write a value to the motor (e.g., 5V), wait 3 seconds, then stop the motor
    motor_output_channel = array('I', [0])
    num_motor_output_channels = len(motor_output_channel)
    motor_output_buffer = array('d', [3.0])  # 5V to drive the motor
    card.write_analog(motor_output_channel, num_motor_output_channels, motor_output_buffer)
    print("Motor voltage set to 5V.")
    time.sleep(1)

    # Stop the motor by setting output to 0V
    
    # Task: Read and print analog and encoder values in a loop

    samples_in_buffer = 1
    frequency = 10  # Hz
    samples = 10
    samples_to_read = 1

    analog_channels = array('I', [0])  # Motor current channel
    num_analog_channels = len(analog_channels)
    voltages = array('d', [0.0] * num_analog_channels)

    encoder_channels = array('I', [0, 1])
    num_encoder_channels = len(encoder_channels)
    counts = array('i', [0] * num_encoder_channels)

    task = card.task_create_reader(
        samples_in_buffer,
        analog_channels, num_analog_channels,
        encoder_channels, num_encoder_channels,
        None, 0, None, 0
    )

    card.task_start(task, Clock.HARDWARE_CLOCK_0, frequency, samples)

    for index in range(samples):
        print(f"Reading sample {index + 1} of {samples}...")
        samples_read = card.task_read(task, samples_to_read, voltages, counts, None, None)
        for channel in range(num_analog_channels):
                print("ADC #%d: %6.3f" % (analog_channels[channel], voltages[channel]), end="  ")
        for channel in range(num_encoder_channels):
                print("ENC #%d: %5d" % (encoder_channels[channel], counts[channel]), end="  ")
        print("\n")

    card.task_stop(task)
    card.task_delete(task)
    print("Reader task stopped and deleted.")

    motor_output_buffer[0] = 0.0
    card.write_analog(motor_output_channel, num_motor_output_channels, motor_output_buffer)
    print("Motor stopped (voltage set to 0V).")


except qh.HILError as ex:
        print("ERROR:  %s" % ex.get_error_message())
finally:
    card.close()
