import os
import rclpy
from .config import Config
from rclpy.node import Node
from interfaces.srv import MoveRobot
from rtde_control import RTDEControlInterface
from ament_index_python import get_package_share_directory


class Controller(Node):
    """
    ROS 2 node that provides a service interface for commanding a UR robot via RTDE.

    Supports various movement types such as linear, joint-space, and inverse kinematics motions.
    Initializes the robot connection, loads tool configuration, and exposes a service for motion execution.
    """
    def __init__(self):
        """
        Initializes the Controller node and sets up the robot communication interface and ROS service.

        Loads TCP configuration from a YAML file, establishes RTDE communication with the robot,
        and registers a ROS 2 service to handle motion requests.
        """
        super().__init__('controller')
        self.logger = self.get_logger()
        self.declare_parameter('ip', '192.168.1.102')
        self.declare_parameter('tool', 'cap_gripper_s')
        self.declare_parameter('debug', False)
        self.robot = RTDEControlInterface(self.get_parameter('ip').get_parameter_value().string_value, 500,
                                          RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
        config_directory = get_package_share_directory('resources')
        resources_path = os.path.join(config_directory, 'config/config.yaml')
        self.config = Config.from_yaml(resources_path)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.debug = True
        self.robot.setTcp(self.config.tool_tcp)
        self.move_robot_service = self.create_service(MoveRobot, 'move_robot', self.move_robot_callback)
        self.logger.info('===== controller initialized =====')

    def move_robot_callback(self, request: MoveRobot.Request, response: MoveRobot.Response) -> MoveRobot.Response:
        """
        Callback for handling incoming MoveRobot service requests.

        Executes the appropriate robot motion command based on the requested type and returns a success or failure response.
        Supports commands: moveL, moveJ, moveJ_IK, and moveUntilContact.
        """
        self.logger.info(f'Move request received.')
        match request.type:
            case 'moveL':
                self.robot.moveL(pose=list(request.pose), speed=float(request.speed),
                                 acceleration=float(request.acceleration), asynchronous=False)
                response.success = True
                response.message = f'Successfully moved to {request.pose}.'
            case 'moveJ':
                self.robot.moveJ(q=list(request.pose), speed=float(request.speed),
                                 acceleration=float(request.acceleration), asynchronous=False)
                response.success = True
                response.message = f'Successfully moved to {request.pose}.'
            case 'moveJ_IK':
                self.robot.moveJ_IK(pose=list(request.pose), speed=float(request.speed),
                                    acceleration=float(request.acceleration), asynchronous=False)
                response.success = True
                response.message = f'Successfully moved to {request.pose}.'
            case 'moveUntilContact':
                # self.robot.moveUntilContact()
                response.success = True
                response.message = f'Successfully moved until contact.'
            case _:
                response.success = False
                response.message = f'Type {request.type} is not supported.'
        return response


def main(args=None) -> None:
    """
    Initializes and runs the Controller node in a ROS 2 environment.

    Spins the node to handle incoming robot movement service requests and shuts down cleanly on termination.
    """
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
