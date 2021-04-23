import rclpy
from rclpy.node import Node

# Import msg
from std_msgs.msg import String
from ds_ros2_msgs.msg import DroneControl


# Create setpoint publisher class
class DroneControlNode(Node):

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

        # Welcome message
        print("Droneswarm control center")

        # Keep prompt going
        while True:

            send_control_input = "not enter"
            send_control_input = input(">>")

            if (send_control_input == ""):
                send_control_input = "blank"

            if (send_control_input.upper() == "DISARM"):
                print("Disarm command sent..")
                msg.arm = False

            elif (send_control_input.upper() == "ARM"):
                print("Arm command sent..")
                msg.arm = True

            else:
                print("invalid command..")


            # Publish message to 'bs_use_control'-topic
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bs_droneControl = DroneControlNode()

    rclpy.spin(bs_droneControl)

    bs_droneControl.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
