import rclpy
from rclpy.node import Node
from interfaces.srv import GetTcpPose
from rtde_receive import RTDEReceiveInterface


class Receiver(Node):
    """
    ROS 2 node that exposes a service for retrieving the robot’s current TCP pose using RTDE.

    Connects to the robot via RTDEReceiveInterface and serves pose requests through a custom service interface.
    """
    def __init__(self):
        """
        Initializes the Receiver node by establishing a connection to the robot and setting up a TCP pose service.

        Reads the robot IP address from ROS parameters, configures the RTDE receive interface,
        and registers a service to return the current TCP pose.
        """
        super().__init__('receiver')
        self.declare_parameter('ip', '192.168.1.102')
        self.logger = self.get_logger()
        self.robot = RTDEReceiveInterface(self.get_parameter('ip').get_parameter_value().string_value, 500, [])
        self.get_tcp_service = self.create_service(GetTcpPose, 'get_tcp_pose', self.send_tcp_pose)
        self.logger.info('===== receiver initialized =====')

    def send_tcp_pose(self, request: GetTcpPose.Request, response: GetTcpPose.Response) -> GetTcpPose.Response:
        """
        Callback function for the get_tcp_pose service.

        Fetches the robot’s current TCP pose from the RTDE interface and includes it in the response.
        """
        response.tcp_pose = self.robot.getActualTCPPose()
        return response


def main(args=None) -> None:
    """
    Initializes and runs the Receiver node in a ROS 2 environment.

    Spins the node to handle incoming TCP pose requests and shuts down cleanly when terminated.
    """
    rclpy.init(args=args)
    node = Receiver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
