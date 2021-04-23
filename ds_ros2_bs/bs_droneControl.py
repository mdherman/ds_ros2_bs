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
        print("#------------------------#")
        print("DRONESWARM CONTROL CENTER")
        print("#------------------------#")

        # Keep prompt going
        while True:

            send_control_input = "not enter"
            send_control_input = input(">>")
            
            # If input is blank, catch error
            if (send_control_input == ""):
                send_control_input = "blank"
            
            # Check command input
            if (send_control_input.upper() == "DISARM"):
                
                # Disarm
                print("Disarm command sent..")
                msg.arm = False
                msg.offboard_control = False
                self.publisher_.publish(msg)

            elif (send_control_input.upper() == "ARM"):
                
                # Disarm before arm
                msg.arm = False
                self.publisher_.publish(msg)
                
                # Arm and offboard control true
                print("Arm command sent..")
                msg.arm = True
                msg.offboard_control = True
                self.publisher_.publish(msg)

            else:
                print("invalid command..")


def main(args=None):
    rclpy.init(args=args)

    bs_droneControl = DroneControlNode()

    rclpy.spin(bs_droneControl)

    bs_droneControl.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
