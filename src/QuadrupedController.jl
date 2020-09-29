__precompile__() # this module is safe to precompile

"""
A Julia wrapper for the code controller for Stanford Pupper, a Raspberry Pi-based quadruped robot that can trot, walk, and jump.
"""
module QuadrupedController
export Configuration, Command
export Robot, run!, behavior_state, behavior_state_string
export toggle_activate, toggle_trot, toggle_hop, turn_left, turn_right, end_turn
export set_height!, set_pitch!, set_roll!, adjust_height!, adjust_roll!

@time using Conda
@time using PyCall

# https://github.com/JuliaPy/PyCall.jl#using-pycall-from-julia-modules
const Configuration = PyNULL()
const four_legs_inverse_kinematics = PyNULL()
const Controller = PyNULL()
const State = PyNULL()

function __init__()
    # Install required packages, if not yet installed
    @static if !Sys.iswindows()
        Conda.add_channel("conda-forge")
    end
    if !haskey(Conda._installed_packages_dict(), "numpy")
        Conda.add("numpy")
    end
    if !haskey(Conda._installed_packages_dict(), "transforms3d")
        Conda.add("transforms3d")
    end

    pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__)    # in order to load a Python module from the current directory

    copy!(Configuration, pyimport("Config").Configuration)
    copy!(four_legs_inverse_kinematics, pyimport("Kinematics").four_legs_inverse_kinematics)
    copy!(Controller, pyimport("Controller").Controller)
    copy!(State, pyimport("State").State)
end

"""
    behavior_state(state)

Returns the behavior_state as an integer

# Examples
```julia-repl
julia> behavior_state(state)
1
```
"""
behavior_state(state) = parse(Int, string(state.behavior_state)[end-2:end-1])

"""
    behavior_state_string(state)

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

    command_interval::Float64

    hop_event::Bool
    trot_event::Bool
    activate_event::Bool

    Command(
        horizontal_velocity::Array{Float64,1} = [0.0, 0.0],
        yaw_rate::Float64 = 0.0,
        height::Float64 = -0.16,
        pitch::Float64 = 0.0,
        roll::Float64 = 0.0,

        hop_event::Bool = false,
        trot_event::Bool = false,
        activate_event::Bool = false;

        command_interval::Float64 = 1.0 / 30
    ) = new(horizontal_velocity, yaw_rate, height, pitch, roll, command_interval, hop_event, trot_event, activate_event)
end

struct Robot
    controller              # Defined in Controller.py
    state                   # Defined in State.py
    command::Command
end

"""
    Robot(config = Configuration(), command::Command = Command())

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
    run!(robot)

Steps the controller
"""
run!(robot) = robot.controller.run(robot.state, robot.command)

"""
    toggle_activate(robot)

Toggles the robot state from DEACTIVATED to REST, and from any state other than DEACTIVATED to DEACTIVATED

# Examples
```julia-repl
julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_activate(robot)
Toggling from REST: 0 to DEACTIVATED: -1
```
"""
function toggle_activate(robot)
    print("Toggling from ", behavior_state_string(robot.state))
    try
        robot.command.activate_event = 1
    catch
        println("\nError: Illegal state transition in toggle_activate!")
        return
    end
    run!(robot)
    robot.command.activate_event = 0
    println(" to ", behavior_state_string(robot.state))
end

"""
    toggle_trot(robot)

Toggles the robot state between REST or a HOP state to TROT, and from TROT to REST

# Examples
```julia-repl
julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_trot(robot)
Toggling from REST: 0 to TROT: 1
```
"""
function toggle_trot(robot)
    print("Toggling from ", behavior_state_string(robot.state))
    try
        robot.state.behavior_state = robot.controller.trot_transition_mapping[robot.state.behavior_state]
    catch
        println("\nError: Illegal state transition in toggle_trot!")
        return
    end
    run!(robot)
    println(" to ", behavior_state_string(robot.state))
end

"""
    toggle_hop(robot)

Toggles the robot state between REST or TROT to HOP, from HOP to FINISHHOP, and from FINISHHOP to REST

# Examples
```julia-repl
julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_hop(robot)
Toggling from REST: 0 to HOP: 2
```
"""
function toggle_hop(robot)
    print("Toggling from ", behavior_state_string(robot.state))
    try
        robot.state.behavior_state = robot.controller.hop_transition_mapping[robot.state.behavior_state]
    catch
        println("\nError: Illegal state transition in toggle_hop!")
        return
    end
    run!(robot)
    println(" to ", behavior_state_string(robot.state))
end

const turn_yaw_rate = 0.4
const turn_velocity = [0.001, 0]

"""
    turn_left(robot)

Turns the robot roughly 90 degrees to the left by slowing down horizontal velocity
to near 0 and setting a yaw_rate to a positive constant

# Examples
```julia-repl
julia> turn_left(robot)
```
"""
function turn_left(robot)
    robot.command.horizontal_velocity = turn_velocity
    robot.command.yaw_rate = turn_yaw_rate
    #println("Pupper marching in place: ", robot.command.horizontal_velocity)
    #println("Pupper turning in place: ",robot.command.yaw_rate)
end

"""
    turn_right(robot)

Turns the robot roughly 90 degrees to the right by slowing down horizontal velocity
to near 0 and setting a yaw_rate to a negative constant

# Examples
```julia-repl
julia> turn_right(robot)
Pupper marching in place:
```
"""
function turn_right(robot)
    robot.command.horizontal_velocity = turn_velocity
    robot.command.yaw_rate = -turn_yaw_rate
    #println("Pupper marching in place: ", robot.command.horizontal_velocity)
    #println("Pupper turning in place: ",robot.command.yaw_rate)
end

"""
    end_turn(robot)

Ends the turn and proceeds on a forward heading at the pre-turn velocity

# Examples
```julia-repl
julia> end_turn(robot)
```
"""
function end_turn(robot)
    robot.command.horizontal_velocity = [0.2, 0]
    robot.command.yaw_rate = 0.0
end

# Functions ported over from JoystickInterface.py
function deadband(value, band_radius)
    return max(value - band_radius, 0) + min(value + band_radius, 0)
end

function clipped_first_order_filter(input, target, max_rate, tau)
    rate = (target - input) / tau
    return clamp(rate, -max_rate, max_rate)
end

# set commands
set_height!(command::Command, height::Real) = (command.height = clamp(height, -0.25, -0.025))

function set_pitch!(robot::Robot, pitch)
    config  = robot.controller.config

    pitch *= -config.max_pitch
    deadbanded_pitch = deadband(pitch, config.pitch_deadband)
    pitch_rate = clipped_first_order_filter(
        robot.state.pitch,
        deadbanded_pitch,
        config.max_pitch_rate,
        config.pitch_time_constant
    )
    robot.command.pitch = robot.state.pitch + robot.command.command_interval * pitch_rate
end

set_roll!(command::Command, roll::Real) = (command.roll = clamp(roll, -1.0, 1.0))

# adjust commands
function adjust_height!(robot::Robot, adjust_by::Int)
    height = robot.state.height - robot.command.command_interval * robot.controller.config.z_speed * adjust_by
    set_height!(robot.command, height)
end

function adjust_roll!(robot::Robot, adjust_by::Int)
    roll = robot.state.roll + robot.command.command_interval * -robot.controller.config.roll_speed * adjust_by
    set_roll!(robot.command, roll)
end

end
