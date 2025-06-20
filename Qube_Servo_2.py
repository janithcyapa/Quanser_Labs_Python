import quanser.hardware as qh
from quanser.hardware import Clock
from array import array
import numpy as np
import time





class QubeServo2:
    def __init__(self,board_type ="qube_servo2_usb", board_identifier="0"):
        print('_____________________________________________')
        version = qh.HIL.get_version()
        print(f"Version: {version.major}.{version.minor}.{version.release} (Build {version.build})")
        print('_____________________________________________')

        self.card = qh.HIL()
        self.card.open(board_type, board_identifier)
        print("Card is connected.")
        self.is_valid = self.card.is_valid()
        print(f"Card is valid: {self.is_valid}")

        print('_____________________________________________')


        self.encoder_resolution = 2048 # I think :D maybe 512 IDK

        self.motor_current_channel = array('I', [0])
        self.num_motor_current_channels = len(self.motor_current_channel)
        self.motor_current = array('d', [0.0] * self.num_motor_current_channels) 

        self.tachometer_channels = array('I', [14000])
        self.num_tachometer_channels = len(self.tachometer_channels)
        self.tachometer = array('d', [0.0] * self.num_tachometer_channels)
        self.rpm = 0
        self.rad_per_sec = 0

        # Define encoder input channels (0: motor position, 1: encoder 1 position)
        self.encoder_channels = array('I', [0, 1])
        self.num_encoder_channels = len(self.encoder_channels)
        self.encoder = array('i', [0] * self.num_encoder_channels)

        # Read three digital inputs: 0 - Amplifier fault, 1 - Motor stall detected, 2 - Motor stall error
        self.fault_channels = array('I', [0, 1, 2])
        self.num_fault_channels = len(self.fault_channels)
        self.faults = array('b', [0] * self.num_fault_channels)

        # Write a value to the motor (e.g., 5V), wait 3 seconds, then stop the motor
        self.motor_output_channel = array('I', [0])
        self.num_motor_output_channels = len(self.motor_output_channel)
        self.motor_output = array('d', [0])  # 5V to drive the motor

        # Define digital output channel for enabling the motor (channel 0)
        self.motor_enable_channel = array('I', [0])
        self.num_motor_enable_channel = len(self.motor_enable_channel)
        self.motor_enable = array('b', [0])

        # Define channels for controlling the RGB LED panel (11000: Red, 11001: Green, 11002: Blue)
        self.LED_channels = array('i', [11000, 11001, 11002])
        self.num_LED_channels = len(self.LED_channels)
        self.LED = array('d', [0.0, 0.0, 0.0])  # Normalize to 0.0-1.0

        self.tasks = {}
        
    # INPUT FUNCTIONS
    def MotorCurrent(self):
        self.card.read_analog(self.motor_current_channel, self.num_motor_current_channels, self.motor_current)
        print(f"Motor current : {self.motor_current[0]} A")
        return self.motor_current[0]
    
    def MotorTachometer(self):
        # Read tachometer (other input), channel 14000
        self.card.read_other(self.tachometer_channels, self.num_tachometer_channels, self.tachometer)
        
        # Convert to RPM
        self.rpm = (self.tachometer[0] / self.encoder_resolution) * 60
        # Convert to rad/s
        self.rad_per_sec = (self.tachometer[0] / self.encoder_resolution) * 2 * np.pi
        
        print(f"Tachometer value: {self.tachometer[0]} counts/s | {self.rpm:.2f} RPM | {self.rad_per_sec:.2f} rad/s")
       
        return self.tachometer[0], self.rpm, self.rad_per_sec

    def Encoder(self):
        self.card.read_encoder(self.encoder_channels, self.num_encoder_channels, self.encoder)
        print(f"Encoder 0 value: {self.encoder[0]}, Encoder 1 value: {self.encoder[1]}")
        return self.encoder

    def ReadFaults(self):

        self.card.read_other(self.fault_channels, self.num_fault_channels, self.faults)
        print(f"fault inputs: Amplifier fault={self.faults[0]}, Motor stall detected={self.faults[1]}, Motor stall error={self.faults[2]}")
        return self.faults
    
    # OUTPUT FUNCTIONS 
    def MotorDrive(self,Voltage):
        RegulatedVoltage =  np.clip(Voltage, -5, 5)
        self.motor_output = array('d', [RegulatedVoltage])  # 5V to drive the motor
        self.card.write_analog(self.motor_output_channel, self.num_motor_output_channels, self.motor_output)
        print(f"Motor voltage : {self.motor_output[0]} V")

    def MotorEnable(self,enable=True):
        # Define digital output channel for enabling the motor (channel 0)
        self.motor_enable = array('b', [1 if enable else 0])
        
        # To enable the motor: card.write_digital(motor_enable_channel, 1, motor_enable)
        self.card.write_digital(self.motor_enable_channel, self.num_motor_enable_channel, self.motor_enable)
        print(f"Motor Enable state: {"Enabled" if self.motor_enable[0] == 1 else "Disabled"}")

    def LightUp(self, color="#FF0000"):
        # Convert hex color to RGB values (0-255) and then normalize to 0.0-1.0
        hex_color = color.lstrip('#')
        rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
        self.LED = array('d', [x/255.0 for x in rgb])  # Normalize to 0.0-1.0
        # Write the color to the RGB LED panel
        self.card.write_other(self.LED_channels, self.num_LED_channels, self.LED)
        print(f"LED color set to: {color} (RGB: {rgb})")

    # Exit 
    def Exit(self):
        self.card.close()
        exit()

    def ReaderTaskCreate(self, taskName="Reader", readCurrent=False, readEncoder=False, readFaults=False, readTachometer=False, samples_in_buffer=100):
        # Check if taskName already exists
        if taskName in self.tasks:
            raise ValueError(f"Task with name '{taskName}' already exists")

        # Create the task
        task = self.card.task_create_reader(
            samples_in_buffer,
            analog_channels=self.motor_current_channel if readCurrent else None,
            num_analog_channels=self.num_motor_current_channels if readCurrent else 0,
            encoder_channels=self.encoder_channels if readEncoder else None,
            num_encoder_channels=self.num_encoder_channels if readEncoder else 0,
            digital_channels=self.fault_channels if readFaults else None,
            num_digital_channels=self.num_fault_channels if readFaults else 0,
            other_channels=self.tachometer_channels if readTachometer else None,
            num_other_channels=self.num_tachometer_channels if readTachometer else 0,
        )

        # Store the task in the dictionary
        self.tasks[taskName] = task
        print(f"Task '{taskName}' Created")
        return task
 
    def WriterTaskCreate(self, taskName="Writer", writeMotor=False, writeMotorEnable=False, writeLED=False, samples_in_buffer=100):
        # Check if taskName already exists
        if taskName in self.tasks:
            raise ValueError(f"Task with name '{taskName}' already exists")

        # Create the writing task
        task = self.card.task_create_writer(
            samples_in_buffer,
            analog_channels=self.motor_output_channel if writeMotor else None,
            num_analog_channels=self.num_motor_output_channels if writeMotor else 0,
            pwm_channels= None,
            num_pwm_channels=0,
            digital_channels=self.motor_enable_channel if writeMotorEnable else None,
            num_digital_channels=self.num_motor_enable_channel if writeMotorEnable else 0,
            control_channels=self.LED_channels if writeLED else None,
            num_control_channels=self.num_LED_channels if writeLED else 0,
        )

        # Store the task in the dictionary
        self.tasks[taskName] = task
        print(f"Writing Task '{taskName}' Created")
        return task

    def StartReadTask(self, taskName, frequency=1000.0, samples=5000, loop_interval=0.001):
        """
        Start a Quanser HIL reader task by name and read data in a loop.

        Args:
            taskName (str): Name of the reader task
            frequency (float): Sampling frequency in Hz (default: 1000 Hz)
            samples (int): Number of samples to process (default: 5000)
            loop_interval (float): Time interval (seconds) between loop iterations (default: 1ms)

        Returns:
            list: List of read data buffers for each sample

        Raises:
            ValueError: If taskName doesn't exist or is not a reader task
            HILError: If task start or read operations fail
        """
        # Check if task exists
        if taskName not in self.tasks:
            raise ValueError(f"Task with name '{taskName}' does not exist")

        task = self.tasks[taskName]
        # Verify it's a reader task
        is_reader = any([
            self.motor_current_channel in task,
            self.encoder_channels in task,
            self.fault_channels in task,
            self.tachometer_channels in task
        ])
        if not is_reader:
            raise ValueError(f"Task '{taskName}' is not a reader task")

        # Initialize buffer for reading
        num_channels = (
            self.num_motor_current_channels +
            self.num_encoder_channels +
            self.num_fault_channels +
            self.num_tachometer_channels
        )
        buffer = array('d', [0.0] * num_channels)
        data = []  # Store read data

        try:
            # Start the task with hardware clock
            task.start(Clock.SYSTEM_CLOCK_1, frequency, samples)
            print(f"Reader Task '{taskName}' Started at {frequency} Hz for {samples} samples")

            # Main loop for reading
            samples_to_process = 1
            for _ in range(samples):
                task.read(buffer, samples_to_process)
                data.append(buffer[:])  # Store a copy of the buffer
                time.sleep(loop_interval)  # Prevent CPU overload

        except KeyboardInterrupt:
            task.stop()
            print(f"Reader Task '{taskName}' Stopped")
        except Exception as e:
            task.stop()
            print(f"Error in reader task '{taskName}': {str(e)}")
            raise
        finally:
            task.stop()
            task.delete()
            print(f"Reader Task '{taskName}' Deleted")

        return data


def main():
    # Simple Test
    Qube = QubeServo2()
    Qube.MotorEnable()
    Qube.LightUp()
    time.sleep(1)
    Qube.ReaderTaskCreate(False,True,False,False)
    Qube.MotorDrive(0)
    time.sleep(1)
    Qube.MotorEnable(enable=False)
    Qube.Exit()



if __name__ == "__main__":
    main()
