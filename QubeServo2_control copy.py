from QubeServo2 import QubeServo2
import time
import logging
from quanser.hardware import HIL

# Configure logging for debugging and monitoring
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def PIDControl(
    motor_current=None,
    encoder1=None,
    encoder2=None,
    tachometer=None,
    faults=None,
):

    motor_enable = True
    led_color = "#00FF00"
    motor_voltage = 3
    return motor_voltage, motor_enable, led_color


def main():
    qube = QubeServo2()
    try:
        qube.lightUp("#FF0000")
        time.sleep(2)
        qube.lightUp("#00FF00")
        time.sleep(2)
        qube.enableMotor(1)
        qube.master_control(
            control_function=PIDControl,
            read_current=True,
            read_encoder=True,
            read_faults=False,
            read_faults=False,
            read_tachometer=False,
            write_motor=True,
            write_motor_enable=False,
            write_led=False,
            samples=HIL.INFINITE,
        )
        qube.lightUp("#0044FF")

    except Exception as e:
        # qube.lightUp("#FF0000")
        logger.error(f"Error in main execution: {str(e)}")
    finally:
        qube.close()


if __name__ == "__main__":
    main()
