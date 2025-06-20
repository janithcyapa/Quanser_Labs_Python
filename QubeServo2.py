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
# Other inputs - Tachometer for motor 14000

# 1 single-ended analog output, ±15V, channel 0 - Drives the motor voltage
# 1 digital output - Enable Motor
# 3 other outputs to drive RGB LED panel
# 11000 - Red (0 ~ 1)
# 11001 - Green (0 ~ 1)
# 11002 - Blue (0 ~ 1)

import quanser.hardware as qh
from quanser.hardware import Clock
from array import array
import numpy as np
import time
import logging
from typing import Callable, Tuple, List, Optional

# Configure logging for debugging and monitoring
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class QubeServo2:
    """Class to interface with Quanser Qube-Servo 2 USB hardware."""

    def __init__(self, board_type="qube_servo2_usb", board_identifier="0"):
        """
        Initialize the Qube-Servo 2 hardware interface.

        Args:
            board_type (str): Type of the board (default: 'qube_servo2_usb').
            board_identifier (str): Board identifier (default: '0').

        Raises:
            RuntimeError: If board initialization fails.

        About Board:
            https://docs.quanser.com/quarc/documentation/qube_servo2_usb.html
            https://docs.quanser.com/quarc/documentation/python/
            1 single-ended analog input channel, 12-bit, ±3A, channel 0 - Current to the motor in Amperes.
            2 single-ended encoder input channels, 5V
                0 - Motor position
                1 - Encoder 1 position
            3 digital inputs
                0 - Amplifier fault
                2 - Motor stall error
                1 - Motor stall detected
            Other inputs - Tachometer for motor 14000

            1 single-ended analog output, ±15V, channel 0 - Drives the motor voltage
            1 digital output - Enable Motor
            3 other outputs to drive RGB LED panel
                11000 - Red (0 ~ 1)
                11001 - Green (0 ~ 1)
                11002 - Blue (0 ~ 1)
        """
        logger.info("Initializing QubeServo2...")
        try:
            # Display Quanser library version
            version = qh.HIL.get_version()
            logger.info(
                f"Quanser HIL Version: {version.major}.{version.minor}.{version.release} (Build {version.build})"
            )

            # Initialize hardware interface
            self.card = qh.HIL()
            self.card.open(board_type, board_identifier)
            logger.info("Hardware card connected successfully.")
            self.is_valid = self.card.is_valid()
            logger.info(f"Card validity: {self.is_valid}")

            if not self.is_valid:
                raise RuntimeError("Invalid hardware card detected.")

            # Hardware configuration constants
            self.encoder_resolution = 2048  # Encoder counts per revolution

            # Analog input for motor current (channel 0, ±3A, 12-bit)
            self.motor_current_channel = array("I", [0])
            self.num_motor_current_channels = len(self.motor_current_channel)
            self.motor_current = array("d", [0.0] * self.num_motor_current_channels)

            # Tachometer input (channel 14000)
            self.tachometer_channels = array("I", [14000])
            self.num_tachometer_channels = len(self.tachometer_channels)
            self.tachometer = array("d", [0.0] * self.num_tachometer_channels)
            self.rpm = 0.0
            self.rad_per_sec = 0.0

            # Encoder input channels (0: motor position, 1: encoder 1 position)
            self.encoder_channels = array("I", [0, 1])
            self.num_encoder_channels = len(self.encoder_channels)
            self.encoder = array("i", [0] * self.num_encoder_channels)

            # Digital input channels (0: amplifier fault, 1: motor stall detected, 2: motor stall error)
            self.fault_channels = array("I", [0, 1, 2])
            self.num_fault_channels = len(self.fault_channels)
            self.faults = array("b", [0] * self.num_fault_channels)

            # Analog output for motor voltage (channel 0, ±15V)
            self.motor_output_channel = array("I", [0])
            self.num_motor_output_channels = len(self.motor_output_channel)
            self.motor_output = array("d", [0.0])

            # Digital output for motor enable (channel 0)
            self.motor_enable_channel = array("I", [0])
            self.num_motor_enable_channel = len(self.motor_enable_channel)
            self.motor_enable = array("b", [0])

            # RGB LED panel control channels (11000: Red, 11001: Green, 11002: Blue)
            self.LED_channels = array("I", [11000, 11001, 11002])
            self.num_LED_channels = len(self.LED_channels)
            self.LED = array("d", [0.0, 0.0, 0.0])  # Normalized 0.0-1.0

            # Dictionary to store tasks
            self.tasks = {}

        except Exception as e:
            logger.error(f"Failed to initialize QubeServo2: {str(e)}")
            raise

    # INPUT FUNCTIONS
    def readMotorCurrent(self):
        """
        Read motor current from analog input channel.

        Returns:
            float: Motor current in Amperes.

        Raises:
            RuntimeError: If reading motor current fails.
        """
        try:
            self.card.read_analog(
                self.motor_current_channel,
                self.num_motor_current_channels,
                self.motor_current,
            )
            logger.info(f"Motor current: {self.motor_current[0]:.2f} A")
            return self.motor_current[0]
        except Exception as e:
            logger.error(f"Error reading motor current: {str(e)}")
            raise RuntimeError(f"Failed to read motor current: {str(e)}")

    def readTachometer(self):
        """
        Read tachometer data and convert to RPM and rad/s.

        Returns:
            tuple: (tachometer counts/s, RPM, rad/s).

        Raises:
            RuntimeError: If reading tachometer fails.
        """
        try:
            self.card.read_other(
                self.tachometer_channels, self.num_tachometer_channels, self.tachometer
            )
            self.rpm = (self.tachometer[0] / self.encoder_resolution) * 60
            self.rad_per_sec = (
                (self.tachometer[0] / self.encoder_resolution) * 2 * np.pi
            )
            logger.info(
                f"Tachometer: {self.tachometer[0]:.2f} counts/s | {self.rpm:.2f} RPM | {self.rad_per_sec:.2f} rad/s"
            )
            return self.tachometer[0], self.rpm, self.rad_per_sec
        except Exception as e:
            logger.error(f"Error reading tachometer: {str(e)}")
            raise RuntimeError(f"Failed to read tachometer: {str(e)}")

    def readEncoder(self):
        """
        Read encoder values for motor and pendulum positions.

        Returns:
            array: Encoder counts for channels 0 and 1.

        Raises:
            RuntimeError: If reading encoder fails.
        """
        try:
            self.card.read_encoder(
                self.encoder_channels, self.num_encoder_channels, self.encoder
            )
            logger.info(
                f"Encoder 0: {self.encoder[0]} counts, Encoder 1: {self.encoder[1]} counts"
            )
            return self.encoder
        except Exception as e:
            logger.error(f"Error reading encoder: {str(e)}")
            raise RuntimeError(f"Failed to read encoder: {str(e)}")

    def readFaults(self):
        """
        Read digital fault inputs.

        Returns:
            array: Fault states (amplifier fault, motor stall detected, motor stall error).

        Raises:
            RuntimeError: If reading faults fails.
        """
        try:
            self.card.read_other(
                self.fault_channels, self.num_fault_channels, self.faults
            )
            logger.info(
                f"Faults: Amplifier={self.faults[0]}, Stall Detected={self.faults[1]}, Stall Error={self.faults[2]}"
            )
            return self.faults
        except Exception as e:
            logger.error(f"Error reading faults: {str(e)}")
            raise RuntimeError(f"Failed to read faults: {str(e)}")

    # OUTPUT FUNCTIONS
    def driveMotor(self, voltage):
        """
        Drive the motor with a specified voltage, clipped to safe range.

        Args:
            voltage (float): Desired motor voltage (±15V max, clipped to ±5V for safety).

        Raises:
            RuntimeError: If writing motor voltage fails.
        """
        try:
            regulated_voltage = np.clip(voltage, -5.0, 5.0)
            self.motor_output = array("d", [regulated_voltage])
            self.card.write_analog(
                self.motor_output_channel,
                self.num_motor_output_channels,
                self.motor_output,
            )
            logger.info(f"Motor voltage set to: {self.motor_output[0]:.2f} V")
        except Exception as e:
            logger.error(f"Error driving motor: {str(e)}")
            raise RuntimeError(f"Failed to drive motor: {str(e)}")

    def enableMotor(self, enable=True):
        """
        Enable or disable the motor.

        Args:
            enable (bool): True to enable, False to disable.

        Raises:
            RuntimeError: If setting motor enable state fails.
        """
        try:
            self.motor_enable = array("b", [1 if enable else 0])
            self.card.write_digital(
                self.motor_enable_channel,
                self.num_motor_enable_channel,
                self.motor_enable,
            )
            state = "Enabled" if self.motor_enable[0] else "Disabled"
            logger.info(f"Motor state: {state}")
        except Exception as e:
            logger.error(f"Error setting motor enable state: {str(e)}")
            raise RuntimeError(f"Failed to set motor enable state: {str(e)}")

    def lightUp(self, color="#FF0000"):
        """
        Set the RGB LED panel color using a hex color code.

        Args:
            color (str): Hex color code (e.g., '#FF0000' for red).

        Raises:
            ValueError: If the color code is invalid.
            RuntimeError: If writing to LED channels fails.
        """
        try:
            # Validate hex color code
            if not color.startswith("#") or len(color) != 7:
                raise ValueError("Invalid hex color code. Use format '#RRGGBB'.")

            # Convert hex to normalized RGB values
            self.LED = self.HexToBinary(color)
            self.card.write_other(self.LED_channels, self.num_LED_channels, self.LED)
            logger.info(f"LED color set to: {color}")
        except ValueError as e:
            logger.error(f"Invalid color code: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Error setting LED color: {str(e)}")
            raise RuntimeError(f"Failed to set LED color: {str(e)}")

    def HexToBinary(self, color="#FF0000"):
        """
        Set the RGB LED panel color using a hex color code.

        Args:
            color (str): Hex color code (e.g., '#FF0000' for red).
                        Defaults to red ('#FF0000') if not specified.

        Returns:
            array.array: Array of 'd' type (double) with normalized RGB values (0.0-1.0).

        Raises:
            ValueError: If the color code is invalid.
            RuntimeError: If the color conversion fails.
        """
        try:
            # Remove '#' if present and validate length
            hex_color = color.lstrip("#")
            if len(hex_color) not in (3, 6):
                raise ValueError(
                    f"Invalid color code '{color}'. Must be 3 or 6 hex digits"
                )

            # Expand 3-digit shorthand to 6-digit form
            if len(hex_color) == 3:
                hex_color = "".join([c * 2 for c in hex_color])

            # Convert to RGB tuple
            try:
                rgb = tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))
            except ValueError as e:
                raise ValueError(f"Invalid hex value in color code '{color}'") from e

            # Normalize to 0.0-1.0 range and return as array
            return array("d", [x / 255.0 for x in rgb])

        except Exception as e:
            raise RuntimeError(
                f"Failed to convert color '{color}' to binary format: {str(e)}"
            ) from e

    def close(self):
        """
        Safely close the hardware connection.

        Raises:
            RuntimeError: If closing the hardware connection fails.
        """
        try:
            self.lightUp("#FF0000")
            self.driveMotor(0)
            time.sleep(1)
            self.enableMotor(0)
            self.lightUp("#000000")
            self.card.close()
            logger.info("Hardware connection closed successfully.")
        except Exception as e:
            logger.error(f"Error closing hardware connection: {str(e)}")
            raise RuntimeError(f"Failed to close hardware connection: {str(e)}")

    def create_reader_task(
        self,
        task_name="Reader",
        read_current=False,
        read_encoder=False,
        read_faults=False,
        read_tachometer=False,
        samples_in_buffer=100,
    ):
        """
        Create a reader task for asynchronous data acquisition.

        Args:
            task_name (str): Name of the task.
            read_current (bool): Read motor current if True.
            read_encoder (bool): Read encoder channels if True.
            read_faults (bool): Read fault channels if True.
            read_tachometer (bool): Read tachometer if True.
            samples_in_buffer (int): Number of samples in the buffer.

        Returns:
            task: Created reader task object.

        Raises:
            ValueError: If task name already exists or parameters are invalid.
            RuntimeError: If task creation fails.
        """
        if task_name in self.tasks:
            raise ValueError(f"Task '{task_name}' already exists.")

        try:

            task = self.card.task_create_reader(
                samples_in_buffer=samples_in_buffer,
                analog_channels=self.motor_current_channel if read_current else None,
                num_analog_channels=(
                    self.num_motor_current_channels if read_current else 0
                ),
                encoder_channels=self.encoder_channels if read_encoder else None,
                num_encoder_channels=self.num_encoder_channels if read_encoder else 0,
                digital_channels=self.fault_channels if read_faults else None,
                num_digital_channels=self.num_fault_channels if read_faults else 0,
                other_channels=self.tachometer_channels if read_tachometer else None,
                num_other_channels=(
                    self.num_tachometer_channels if read_tachometer else 0
                ),
            )

            self.tasks[task_name] = task
            logger.info(f"Reader task '{task_name}' created successfully.")
            return task
        except Exception as e:
            logger.error(f"Error creating reader task '{task_name}': {str(e)}")
            raise RuntimeError(f"Failed to create reader task: {str(e)}")

    def create_writer_task(
        self,
        task_name="Writer",
        write_motor=False,
        write_motor_enable=False,
        write_led=False,
        samples_in_buffer=100,
    ):
        """
        Create a writer task for asynchronous output control.

        Args:
            task_name (str): Name of the task.
            write_motor (bool): Write to motor voltage if True.
            write_motor_enable (bool): Write to motor enable if True.
            write_led (bool): Write to LED channels if True.
            samples_in_buffer (int): Number of samples in the buffer.

        Returns:
            task: Created writer task object.

        Raises:
            ValueError: If task name already exists or parameters are invalid.
            RuntimeError: If task creation fails.
        """
        if task_name in self.tasks:
            raise ValueError(f"Task '{task_name}' already exists.")

        try:
            task = self.card.task_create_writer(
                samples_in_buffer,
                analog_channels=self.motor_output_channel if write_motor else None,
                num_analog_channels=(
                    self.num_motor_output_channels if write_motor else 0
                ),
                pwm_channels=None,
                num_pwm_channels=0,
                digital_channels=(
                    self.motor_enable_channel if write_motor_enable else None
                ),
                num_digital_channels=(
                    self.num_motor_enable_channel if write_motor_enable else 0
                ),
                other_channels=self.LED_channels if write_led else None,
                num_other_channels=self.num_LED_channels if write_led else 0,
            )
            self.tasks[task_name] = task
            logger.info(f"Writer task '{task_name}' created successfully.")
            return task
        except Exception as e:
            logger.error(f"Error creating writer task '{task_name}': {str(e)}")
            raise RuntimeError(f"Failed to create writer task: {str(e)}")

    def start_reader_task(
        self, task_name, frequency=1000.0, samples=5000, loop_interval=0.001
    ):
        """
        Start a reader task and collect data in a loop.

        Args:
            task_name (str): Name of the reader task.
            frequency (float): Sampling frequency in Hz (default: 1000 Hz).
            samples (int): Number of samples to collect (default: 5000).
            loop_interval (float): Time interval between loop iterations (default: 1ms).

        Returns:
            list: List of read data buffers.

        Raises:
            ValueError: If task does not exist or is not a reader task.
            RuntimeError: If task execution fails.
        """
        if task_name not in self.tasks:
            raise ValueError(f"Task '{task_name}' does not exist.")

        task = self.tasks[task_name]
        is_reader = any(
            [
                self.motor_current_channel in task,
                self.encoder_channels in task,
                self.fault_channels in task,
                self.tachometer_channels in task,
            ]
        )
        if not is_reader:
            raise ValueError(f"Task '{task_name}' is not a reader task.")

        num_channels = (
            self.num_motor_current_channels
            + self.num_encoder_channels
            + self.num_fault_channels
            + self.num_tachometer_channels
        )
        buffer = array("d", [0.0] * num_channels)
        data = []

        try:
            task.start(Clock.SYSTEM_CLOCK_1, frequency, samples)
            logger.info(
                f"Reader task '{task_name}' started at {frequency} Hz for {samples} samples"
            )

            samples_to_process = 1
            for _ in range(samples):
                task.read(buffer, samples_to_process)
                data.append(buffer[:])
                time.sleep(loop_interval)
        except KeyboardInterrupt:
            logger.warning(f"Reader task '{task_name}' interrupted by user.")
        except Exception as e:
            logger.error(f"Error in reader task '{task_name}': {str(e)}")
            raise RuntimeError(f"Reader task execution failed: {str(e)}")
        finally:
            task.stop()
            task.delete()
            del self.tasks[task_name]
            logger.info(f"Reader task '{task_name}' stopped and deleted.")
        return data

    def master_control(
        self,
        control_function: Callable[
            [dict], Tuple[Optional[float], Optional[bool], Optional[str]]
        ],
        read_current: bool = False,
        read_encoder: bool = False,
        read_faults: bool = False,
        read_tachometer: bool = False,
        write_motor: bool = False,
        write_motor_enable: bool = False,
        write_led: bool = False,
        frequency: float = 500.0,
        samples: int = 5000,
        loop_interval: float = 0.001,
        reader_task_name: str = "MasterReader",
        writer_task_name: str = "MasterWriter",
    ) -> List[dict]:
        """
        Master control function to manage input reading and output writing for control systems.

        This function creates reader and writer tasks based on user-specified inputs and outputs,
        collects data, passes it to a user-defined control function, and applies the control outputs.

        Args:
            control_function: User-defined function that takes input data (dict) and returns
                             (motor_voltage, motor_enable, led_color).
                             motor_voltage (float, optional): Voltage to drive the motor (±5V).
                             motor_enable (bool, optional): True to enable motor, False to disable.
                             led_color (str, optional): Hex color code for LED (e.g., '#FF0000').
            read_current: Read motor current if True.
            read_encoder: Read encoder channels if True.
            read_faults: Read fault channels if True.
            read_tachometer: Read tachometer if True.
            write_motor: Write to motor voltage if True.
            write_motor_enable: Write to motor enable if True.
            write_led: Write to LED channels if True.
            frequency: Sampling frequency in Hz (default: 1000 Hz).
            samples: Number of samples to process (default: 5000).
            loop_interval: Time interval between loop iterations (default: 1ms).
            reader_task_name: Name of the reader task (default: 'MasterReader').
            writer_task_name: Name of the writer task (default: 'MasterWriter').
            samples_in_buffer: Number of samples in the buffer (default: 100).

        Returns:
            List[dict]: List of dictionaries containing input data for each sample.

        Raises:
            ValueError: If task names already exist, no inputs/outputs selected, or invalid parameters.
            RuntimeError: If task creation or execution fails.
        """

        samples_in_buffer = int(frequency)

        # Validate inputs and outputs
        if not any([read_current, read_encoder, read_faults, read_tachometer]):
            raise ValueError("At least one input must be selected for reading.")
        if not any([write_motor, write_motor_enable, write_led]):
            raise ValueError("At least one output must be selected for writing.")
        # Create reader and writer tasks
        try:
            reader_task = self.create_reader_task(
                task_name=reader_task_name,
                read_current=read_current,
                read_encoder=read_encoder,
                read_faults=read_faults,
                read_tachometer=read_tachometer,
                samples_in_buffer=samples_in_buffer,
            )
            # writer_task = self.create_writer_task(
            #     task_name=writer_task_name,
            #     write_motor=write_motor,
            #     write_motor_enable=write_motor_enable,
            #     write_led=write_led,
            #     samples_in_buffer=samples_in_buffer,
            # )
            logger.info(
                f"Master control started: Reader task '{reader_task_name}' and Writer task '{writer_task_name}' at {frequency} Hz"
            )
        except Exception as e:
            logger.error(f"Failed to create tasks: {str(e)}")
            raise RuntimeError(f"Task creation failed: {str(e)}")

        try:
            # Start both tasks
            self.card.task_start(
                reader_task, Clock.HARDWARE_CLOCK_0, frequency, samples
            )
            # self.card.task_start(
            #     writer_task, Clock.HARDWARE_CLOCK_0, frequency, samples
            # )
            logger.info(
                f"Master control started: Reader task '{reader_task_name}' and Writer task '{writer_task_name}' at {frequency} Hz"
            )

            # Main control loop
            samples_to_process = 1
            for index in range(samples):
                # Read data
                print(f"Sample = {index}")
                self.card.task_read(
                    task=reader_task,
                    num_samples=samples_to_process,
                    analog_buffer=self.motor_current if read_current else None,
                    encoder_buffer=self.encoder if read_encoder else None,
                    digital_buffer=self.faults if read_faults else None,
                    other_buffer=self.tachometer if read_tachometer else None,
                )

                # Call user-defined control function
                try:
                    motor_voltage, motor_enable, led_color = control_function(
                        self.motor_current[0],
                        self.encoder[0],
                        self.encoder[1],
                        self.tachometer[0],
                        self.faults,
                    )
                    if write_motor:
                        self.driveMotor(motor_voltage)
                    if write_motor_enable:
                        self.enableMotor(motor_enable)
                    if write_led:
                        self.lightUp(led_color)
                except Exception as e:
                    logger.error(f"Error in control function: {str(e)}")
                    raise RuntimeError(f"Control function failed: {str(e)}")

                time.sleep(loop_interval)

        except KeyboardInterrupt:
            logger.warning("Master control interrupted by user.")
        except Exception as e:
            logger.error(f"Error in master control: {str(e)}")
            raise RuntimeError(f"Master control execution failed: {str(e)}")
        finally:
            # Stop and delete tasks
            self.card.task_stop_all()
            self.card.task_delete_all()

        if reader_task_name in self.tasks:
            del self.tasks[reader_task_name]
        if writer_task_name in self.tasks:
            del self.tasks[writer_task_name]
        logger.info(
            f"Tasks '{reader_task_name}' and '{writer_task_name}' stopped and deleted."
        )
        return


def main():
    # check All are working
    try:
        qube = QubeServo2()
        qube.lightUp("#FF0000")
        time.sleep(2)
        qube.lightUp("#00FF00")
        qube.enableMotor(1)
        qube.driveMotor(1)
        time.sleep(3)
        qube.driveMotor(0)
        time.sleep(3)
        qube.driveMotor(-1)
        time.sleep(3)
        qube.lightUp("#0044FF")
        time.sleep(1)
        qube.close()
    except Exception as e:
        # qube.lightUp("#FF0000")
        logger.error(f"Error in main execution: {str(e)}")
        qube.close()
        raise


if __name__ == "__main__":
    main()
