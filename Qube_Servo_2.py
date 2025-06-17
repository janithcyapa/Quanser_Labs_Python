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

# Define the analog input channel for motor current (channel 0)
motor_current_channel = array('I', [0])
num_motor_current_channels = len(motor_current_channel)
motor_current = array('d', [0.0] * num_motor_current_channels) 
# To read motor current: card.read_analog(motor_current_channel, num_motor_current_channels, motor_current)

# Define encoder input channels (0: motor position, 1: encoder 1 position)
encoder_channels = array('I', [0, 1])
num_encoder_channels = len(encoder_channels)
encoder = array('i', [0] * num_encoder_channels)
# To read encoder positions: card.read_encoder(encoder_channels, num_encoder_channels, encoder)
#card.set_encoder_counts(encoder_channels, num_encoder_channels, array("i", [0] * num_encoder_channels))

# Define digital input channels (0: amplifier fault, 1: motor stall detected, 2: motor stall error)
digital_channels = array('I', [0, 1, 2])
num_digital_channels = len(digital_channels)
digital = array('b', [0] * num_digital_channels)
# To read digital inputs: card.read_digital(digital_channels, num_digital_channels, digital)

# Define other analog input channels (e.g., tachometer for motor 0)
tachometer_channels = array('I', [0])
num_tachometer_channels = len(tachometer_channels)
tachometer = array('d', [0.0] * num_tachometer_channels)
# To read other analog inputs: card.read_analog(tachometer_channels, num_tachometer_channels, tachometer)

# Define analog output channel for driving the motor voltage (channel 0)
motor_output_channel = array('I', [0])
num_motor_output_channels = len(motor_output_channel)
motor_output = array('d', [0]) 
# To write motor voltage: card.write_analog(motor_output_channel, num_motor_output_channels, motor_output)

# Define digital output channel for enabling the motor (channel 0)
motor_enable_channel = array('I', [0])
num_motor_enable_channel = len(motor_enable_channel)
motor_enable = array('b', [0])
# To enable the motor: card.write_digital(motor_enable_channel, 1, motor_enable)

# Define channels for controlling the RGB LED panel (11000: Red, 11001: Green, 11002: Blue)
LED_channels = array('i', [11000, 11001, 11002])
num_LED_channels = len(LED_channels)
other = array('d', [1.0, 0.0, 0.0])  # Example: Red ON, Green OFF, Blue OFF
# To set LED colors: card.write_other(LED_channels, num_LED_channels, other)


samples_in_buffer = 100
frequency = 10  # Hz
samples = 10 # For continuous reading, use qh.HIL.INFINITE
# samples = qh.HIL.INFINITE
samples_to_read = 1

try:
    version = qh.HIL.get_version()
    print(f"Version: {version.major}.{version.minor}.{version.release} (Build {version.build})")
    print('_____________________________________________')  

    card = qh.HIL()
    card.open(board_type, board_identifier)
    print("Card is connected.")
    is_valid = card.is_valid()
    print(f"Card is valid: {is_valid}")

    print('_____________________________________________')
   
    motor_enable = array('b', [1])
    card.write_digital(motor_enable_channel, 1, motor_enable)
    print(f"Motor Enable state: {motor_enable[0]}")
    time.sleep(1)

    task = card.task_create_reader(samples_in_buffer,
        None, 0,
        encoder_channels, num_encoder_channels,
        None, 0, 
        None, 0, 
    )
    card.task_start(task, Clock.HARDWARE_CLOCK_0, frequency, samples)
    print("Task started. Reading encoder and tachometer values...")
    
    # motor_output = array('d', [3.0])
    # card.write_analog(motor_output_channel, num_motor_output_channels, motor_output)        
    # time.sleep(1)

    for index in range(samples):
        print(flush=True)
        # card.set_encoder_counts(encoder_channels, num_encoder_channels, array("i", [0] * num_encoder_channels))
        print(f"Reading sample {index + 1} of {samples}...")
        card.task_read(task, samples, None, encoder, None, None)
        print(f"Encoder 0 value: {encoder[0]}, Encoder 1 value: {encoder[1]}")
        if index % 3 == 0:
            motor_output = array('d', [3.0])  # 5V to drive the motor
        elif index % 3 == 2:
            motor_output = array('d', [-3.0])  # 5V to drive the motor
        else:
            motor_output = array('d', [0])  # 5V to drive the motor
        card.write_analog(motor_output_channel, num_motor_output_channels, motor_output)        
        time.sleep(1)

    motor_output = array('d', [0.0])  # 5V to drive the motor
    card.write_analog(motor_output_channel, num_motor_output_channels, motor_output) 
    motor_enable_buffer = array('b', [0])
    card.write_digital(motor_enable_channel, 1, motor_enable)
    print(f"Motor Enable state: {motor_enable[0]}")

    card.task_stop(task)
    card.task_delete(task)
    print("Reader task stopped and deleted.")
except qh.HILError as ex:
        print("ERROR:  %s" % ex.get_error_message())
finally:
    card.close()