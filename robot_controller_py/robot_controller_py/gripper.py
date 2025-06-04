

import rclpy
import socket
import time
from interfaces.srv import MoveGripper
from rclpy.node import Node


class GripperController(Node):
    """
    ROS 2 node for controlling a Robotiq gripper via TCP socket communication.

    Initializes the socket connection, configures the gripper, and provides a ROS 2 service interface
    for executing commands such as open, close, position control, and activation.
    """
    def __init__(self):
        """
        Initializes the GripperController node, establishes a socket connection with the gripper,
        sends initial configuration commands, and sets up the move_gripper service.

        Reads gripper IP, port, and force level from ROS parameters and prepares the controller for operation.
        """
        super().__init__('robot_controller')
        self.declare_parameter('gripper_ip', '192.168.1.102')
        self.declare_parameter('gripper_port', 63352)
        self.declare_parameter('force', 50)
        self.logger = self.get_logger()
        gripper_ip = self.get_parameter('gripper_ip').value
        gripper_port = self.get_parameter('gripper_port').value
        try:
            self.socket= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((gripper_ip, gripper_port))
        except socket.error as e:
            self.get_logger().error(str(e))
        force = int(round((self.get_parameter('force').get_parameter_value().integer_value * 255) / 100))
        self.socket.sendall(b'SET FOR %i\n' % force*255)
        self.gripper_service = self.create_service(MoveGripper, 'move_gripper', self.gripper_callback)
        self.socket.sendall(b'SET ACT 1\n')
        self.logger.info('===== gripper_controller initialized =====')

    def gripper_callback(self, request: MoveGripper.Request, response: MoveGripper.Response) -> MoveGripper.Response:
        """
        Callback function for processing incoming gripper commands from the move_gripper service.

        Routes the request based on its type and delegates to the corresponding internal method.
        Supported types include 'open', 'close', 'position', and 'activate'.
        """
        self.logger.info(f'Gripper request received.')
        match request.type:
            case 'open':
                response = self.open_gripper()
            case 'close':
                response = self.close_gripper()
            case 'position':
                response = self.set_position(int(request.position))
            case 'activate':
                response = self.activate_gripper()
            case _:
                response.success = False
                response.message = f'Type {request.type} is not supported.'
        return response

    def execute_gripper_command(self, primary_command: bytes, success_message: str, failure_message: str, delay: float = 3.0) -> MoveGripper.Response:
        """
        Sends a gripper command over the socket and interprets the response.

        Used internally to standardize command execution logic with optional delay and success/failure parsing.
        """
        self.socket.sendall(primary_command)
        self.socket.sendall(b'SET GTO 1\n')
        time.sleep(delay)
        gripper_response = self.socket.recv(2 ** 10).decode('utf-8').strip()
        print(gripper_response)
        response = MoveGripper.Response()
        if 'n' in gripper_response:
            response.success = False
            response.message = failure_message
        else:
            response.success = True
            response.message = success_message
        return response

    def open_gripper(self) -> MoveGripper.Response:
        """
        Sends a command to open the gripper fully.

        Returns a service response indicating whether the operation succeeded.
        """
        return self.execute_gripper_command(
            b'SET POS 0\n','Open successful.','Open failed.')

    def close_gripper(self) -> MoveGripper.Response:
        """
        Sends a command to close the gripper fully.

        Returns a service response indicating whether the operation succeeded.
        """
        return self.execute_gripper_command(
            b'SET POS 255\n','Close successful.','Close failed.')

    def set_position(self, position: int) -> MoveGripper.Response:
        """
        Sends a command to move the gripper to a specified position (0–255).

        Returns a service response indicating whether the operation succeeded.
        """
        command = f'SET POS {position}\n'.encode()
        return self.execute_gripper_command(
            command,f'Set position to {position} successful.',f'Set position to {position} failed.')

    def activate_gripper(self) -> MoveGripper.Response:
        """
        Sends a command to activate the gripper.

        Returns a service response indicating whether activation was successful.
        """
        return self.execute_gripper_command(
            b'SET ACT 1\n','Activation successful.','Activation failed.')


def main(args=None) -> None:
    """
    Initializes and runs the Gripper node in a ROS 2 environment.

    Spins the node to handle incoming gripper service requests and shuts down cleanly on termination.
    """
    rclpy.init(args=args)
    node = GripperController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
