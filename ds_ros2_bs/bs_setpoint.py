import rclpy
from rclpy.node import Node

# Import correct msg
from std_msgs.msg import String
from ds_msgs.msg import TrajectorySetpoint


# Create setpoint publisher class
class SetpointPublisher(Node):

    def __init__(self):
        # Init publisher
        super().__init__('bs_setpoint')
        self.publisher_ = self.create_publisher(TrajectorySetpoint, 'bs_use_setpoint', 10)

        # Member variables
        self.setpoints_ = {"x": 0.0,
                           "y": 0.0,
                           "z": 0.0,
                           "yaw": 0.0}

        # Run prompt function
        self.prompt_user()


    # Prompt user for text file with setpoint data
    def prompt_user(self):
        # Create TrajectorySetpoint message instance
        msg = TrajectorySetpoint()
        
        # Keep prompt going
        while True:

            # Read setpoint text-file and add values to setpoints_ dict
            setpoint_file_path = input("Setpoint file path: ")
            setpoint_file = open(setpoint_file_path, "r")

            for line in setpoint_file:
                key = ""
                for char in line:
                    if (char != ":"):
                        key += char
                    else:
                        if (key in self.setpoints_):
                            self.setpoints_[key] = round(float(line[line.find(":")+2:]), 5)

            # Match setpoints from setpoint file with message
            msg.x = self.setpoints_["x"]
            msg.y = self.setpoints_["y"]
            msg.z = self.setpoints_["z"]
            msg.yaw = self.setpoints_["yaw"]

            # Publish message to 'bs_use_setpoint'-topic
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bs_setpoint = SetpointPublisher()

    rclpy.spin(bs_setpoint)

    bs_setpoint.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
