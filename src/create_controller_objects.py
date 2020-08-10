import numpy as np

from common.Controller import Controller
from common.Command import Command
from common.State import State
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics


def create_controller_objects(yaw_rate = 0.0):
    default_velocity=np.array([0.2, 0.0])    # np.zeros(2)

    # Create config
    config = Configuration()
    config.z_clearance = 0.02

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics,)
    state = State()
    command = Command()

    # Emulate the joystick inputs required to activate the robot
    command.activate_event = 1
    controller.run(state, command)
    command.activate_event = 0
    command.trot_event = 1
    controller.run(state, command)
    command = Command()  # zero it out

    # Apply a constant command. # TODO Add support for user input or an external commander
    command.horizontal_velocity = default_velocity
    command.yaw_rate = yaw_rate

    return controller, state, command
