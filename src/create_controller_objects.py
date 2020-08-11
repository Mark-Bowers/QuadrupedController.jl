from Controller import Controller
from common.State import State
from Config import Configuration
from Kinematics import four_legs_inverse_kinematics


def create_controller_objects():
    # Create config
    config = Configuration()

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics,)
    state = State()

    return controller, state
