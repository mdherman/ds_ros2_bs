import rclpy
from rclpy.node import Node

# Import msg
from std_msgs.msg import String
from px4_msgs.msg import TrajectorySetpoint
from ds_msgs.msg import DroneControl


# Create setpoint publisher class
class DroneControl(Node):

    def __init__(self):
        # Init publisher
        super().__init__('bs_droneControl')
        self.publisher_ = self.create_publisher(DroneControl, 'bs_use_control', 10)

        # Run prompt function
        self.prompt_user()


    # Prompt user for text file with setpoint data
    def prompt_user(self):

        # Create DroneControl message instance
        msg = DroneControl()

        # Keep prompt going
        while True:

            send_control_input = "not enter"
            send_control_input = input(">>")

            # Wait for user to hit enter
            if (send_control_input.upper() == "DISARM"):
                print("Disarm command sent..")
                msg.arm = False

            else if (send_control_input.upper() == "ARM":
                print("Arm command sent..")
                msg.arm = True

            else:
                msg.arm = False
                msg.offoard_control = False


            }

            # Publish message to 'bs_use_control'-topic
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bs_droneControl = DroneControl()

    rclpy.spin(bs_droneControl)

    bs_droneControl.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
