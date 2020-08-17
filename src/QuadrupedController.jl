__precompile__() # this module is safe to precompile

"""
A Julia wrapper for the code controller for Stanford Pupper, a Raspberry Pi-based quadruped robot that can trot, walk, and jump.
"""
module QuadrupedController
export Configuration, Command
export Robot, run!, behavior_state, behavior_state_string
export toggle_activate, toggle_trot, toggle_hop

using DocStringExtensions
@template (FUNCTIONS, METHODS, MACROS) =
    """
    $(SIGNATURES)
    $(DOCSTRING)
    """

@template (TYPES,) =
    """
    $(TYPEDEF)
    $(DOCSTRING)
    """

#using Conda
using PyCall

# https://github.com/JuliaPy/PyCall.jl#using-pycall-from-julia-modules
const Configuration = PyNULL()
const four_legs_inverse_kinematics = PyNULL()
const Controller = PyNULL()
const State = PyNULL()

function __init__()
    # Install required packages, if not yet installed
    """
    Conda.add_channel("conda-forge")
    if !haskey(Conda._installed_packages_dict(), "numpy")
        Conda.add("numpy")
    end
    if !haskey(Conda._installed_packages_dict(), "transforms3d")
        Conda.add("transforms3d")
    end
    """

    pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__)    # in order to load a Python module from the current directory

    copy!(Configuration, pyimport("Config").Configuration)
    copy!(four_legs_inverse_kinematics, pyimport("Kinematics").four_legs_inverse_kinematics)
    copy!(Controller, pyimport("Controller").Controller)
    copy!(State, pyimport("State").State)
end

"""
    $(SIGNATURES)

Returns the behavior_state as an integer

# Examples
```julia-repl
julia> behavior_state(state)
1
```
"""
behavior_state(state) = parse(Int, string(state.behavior_state)[end-2:end-1])

"""
    $(SIGNATURES)

Returns the behavior_state as a string

# Examples
```julia-repl
julia> behavior_state_string(state)
TROT
```
"""
behavior_state_string(state) = string(state.behavior_state)[25:end-1]

mutable struct Command # Stores movement commands
    horizontal_velocity::Array{Float64,1}
    yaw_rate::Float64
    height::Float64
    pitch::Float64
    roll::Float64

    activation::Int                     # doesn't appear to be used anywhere

    hop_event::Bool
    trot_event::Bool
    activate_event::Bool

    Command(
        horizontal_velocity::Array{Float64,1} = [0.0, 0.0],
        yaw_rate::Float64 = 0.0,
        height::Float64 = -0.16,
        pitch::Float64 = 0.0,
        roll::Float64 = 0.0,

        activation::Int = 0,            # doesn't appear to be used anywhere

        hop_event::Bool = false,
        trot_event::Bool = false,
        activate_event::Bool = false
    ) = new(horizontal_velocity, yaw_rate, height, pitch, roll, activation, hop_event, trot_event, activate_event)
end

struct Robot
    controller
    state
    command::Command
end

"""
    $(SIGNATURES)

Robot object contructor
"""
function Robot(config = Configuration(), command::Command = Command())
    controller = Controller(config, four_legs_inverse_kinematics,)
    state = State()   # holds the runtime state of the simulator
    Robot(controller, state, command)
end

# associated functions:
# [sim]step - run
# joystick commands
# reading sensors/environment/state

"""
    $(SIGNATURES)

Steps the controller
"""
run!(robot) = robot.controller.run(robot.state, robot.command)

"""
    $(SIGNATURES)

Toggles the robot state from DEACTIVATED to REST, and from any state other than DEACTIVATED to DEACTIVATED

# Examples
```julia-repl
julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_activate(robot)
Toggling from REST: 0 to DEACTIVATED: -1
"""
function toggle_activate(robot)
    print("Toggling from ", behavior_state_string(robot.state))
    robot.command.activate_event = 1
    run!(robot)
    robot.command.activate_event = 0
    println(" to ", behavior_state_string(robot.state))
end

"""
    $(SIGNATURES)

Toggles the robot state between REST or a HOP state to TROT, and from TROT to REST

# Examples
```julia-repl
julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_trot(robot)
Toggling from REST: 0 to TROT: 1
"""
function toggle_trot(robot)
    print("Toggling from ", behavior_state_string(robot.state))
    robot.state.behavior_state = robot.controller.trot_transition_mapping[robot.state.behavior_state]
    run!(robot)
    println(" to ", behavior_state_string(robot.state))
end

"""
    $(SIGNATURES)

Toggles the robot state between REST or TROT to HOP, from HOP to FINISHHOP, and from FINISHHOP to REST

# Examples
```julia-repl
julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_hop(robot)
Toggling from REST: 0 to HOP: 2
"""
function toggle_hop(robot)
    print("Toggling from ", behavior_state_string(robot.state))
    robot.state.behavior_state = robot.controller.hop_transition_mapping[robot.state.behavior_state]
    run!(robot)
    println(" to ", behavior_state_string(robot.state))
end

end
