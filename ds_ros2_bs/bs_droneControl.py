import rclpy
from rclpy.node import Node

# Import msg
from std_msgs.msg import String
from ds_ros2_msgs.msg import DroneControl
from ds_ros2_msgs.msg import TrajectorySetpoint


# Create setpoint publisher class
class DroneControlNode(Node):

    def __init__(self):
        # Init publisher
        super().__init__('bs_droneControl')

        # Create publishers
        self.control_publisher_ = self.create_publisher(DroneControl, 'bs_use_control', 10)
        self.setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, 'bs_use_setpoint', 10)

        # Variables
        self.launch = False

        # Run prompt function
        self.prompt_user()


    # Prompt user for text file with setpoint data
    def prompt_user(self):

        # Create DroneControl message instance
        control_msg = DroneControl()
        setpoint_msg = TrajectorySetpoint()

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
                control_msg.arm = False
                self.control_publisher_.publish(control_msg)

            elif (send_control_input.upper() == "ARM"):

                # Disarm before arm
                control_msg.arm = False
                self.control_publisher_.publish(control_msg)

                # Arm and offboard control true
                print("Arm command sent..")
                control_msg.arm = True
                self.control_publisher_.publish(control_msg)

            elif (send_control_input.upper() == "LAUNCH"):
                control_msg.launch = True
                self.control_publisher_.publish(control_msg)
                print("Launch command sent...")

            elif (send_control_input.upper() == "TURN PX ON"):
                control_msg.switch_px = True
                self.control_publisher_.publish(control_msg)
                print("Switched on pixhawk...")

            elif (send_control_input.upper() == "TURN PX OFF"):
                control_msg.switch_px = False
                self.control_publisher_.publish(control_msg)
                print("Switched off pixhawk...")

            elif (send_control_input.upper() == "LAND"):
                control_msg.launch = False
                self.control_publisher_.publish(control_msg)
                print("Land command sent...")

            elif (send_control_input.upper()[0] == "X"):
                print("Setpoint x sent...")
                setpoint_msg.x = float(send_control_input[1:])
                self.setpoint_publisher_.publish(setpoint_msg)

            elif (send_control_input.upper()[0] == "Y"):
                print("Setpoint y sent...")
                setpoint_msg.y = float(send_control_input[1:])
                self.setpoint_publisher_.publish(setpoint_msg)

            elif (send_control_input.upper()[0] == "Z"):
                print("Setpoint z sent...")
                setpoint_msg.z = float(send_control_input[1:])
                self.setpoint_publisher_.publish(setpoint_msg)

            elif (send_control_input.upper()[0:2] == "VZ"):
                print("Setpoint vz sent...")
                setpoint_msg.vz = float(send_control_input[2:])
                self.setpoint_publisher_.publish(setpoint_msg)

            elif (send_control_input.upper() == "VELOCITY"):
                setpoint_msg.x = float("NaN")
                setpoint_msg.y = float("NaN")
                setpoint_msg.z = float("NaN")
                setpoint_msg.yaw = float("NaN")
                setpoint_msg.yawspeed = float("NaN")
                setpoint_msg.vx = 0.0
                setpoint_msg.vy = 0.0
                setpoint_msg.vz = 0.0
                setpoint_msg.acceleration = [float("NaN"), float("NaN"), float("NaN")]
                setpoint_msg.jerk = [float("NaN"), float("NaN"), float("NaN")]
                setpoint_msg.thrust = [float("NaN"), float("NaN"), float("NaN")]

                control_msg.arm = False
                control_msg.launch = True

                print("Ready for velocity setpoints..")

                self.setpoint_publisher_.publish(setpoint_msg)
                self.control_publisher_.publish(control_msg)


            elif (send_control_input.upper() == "RESET"):
                setpoint_msg.x = 0.0
                setpoint_msg.y = 0.0
                setpoint_msg.z = 0.0
                setpoint_msg.yaw = 0.0
                setpoint_msg.yawspeed = 0.0
                setpoint_msg.vx = 0.0
                setpoint_msg.vy = 0.0
                setpoint_msg.vz = 0.0
                setpoint_msg.acceleration = [0.0, 0.0, 0.0]
                setpoint_msg.jerk = [0.0, 0.0, 0.0]
                setpoint_msg.thrust = [0.0, 0.0, 0.0]

                control_msg.arm = False
                control_msg.launch = True

                print("All reset..")

                self.setpoint_publisher_.publish(setpoint_msg)
                self.control_publisher_.publish(control_msg)

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
