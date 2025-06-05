import quanser.hardware as qh
from array import array
import time


board_type = "qube_servo2_usb"
board_identifier = "0"

# https://docs.quanser.com/quarc/documentation/qube_servo2_usb.html

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
        

    # Read three digital inputs: 0 - Amplifier fault, 1 - Motor stall detected, 2 - Motor stall error
    digital_channels = array('I', [0, 1, 2])
    num_digital_channels = len(digital_channels)
    digital_buffer = array('b', [0] * num_digital_channels)
    card.read_digital(digital_channels, num_digital_channels, digital_buffer)
    print(f"Digital inputs: Amplifier fault={digital_buffer[0]}, Motor stall detected={digital_buffer[1]}, Motor stall error={digital_buffer[2]}")

    # Read tachometer (other input), channel 0
    other_channels = array('I', [0])
    num_other_channels = len(other_channels)
    other_buffer = array('d', [0.0] * num_other_channels)
    card.read_analog(other_channels, num_other_channels, other_buffer)
    print(f"Tachometer value: {other_buffer[0]}")

    card.read_analog(motor_current_channel, num_motor_current_channels, motor_current_buffer)
    print(f"Motor current (A): {motor_current_buffer[0]}")
        

    card.read_encoder(encoder_channels, num_encoder_channels, encoder_buffer)
    print(f"Encoder 0 value: {encoder_buffer[0]}, Encoder 1 value: {encoder_buffer[1]}")

finally:
    card.close()