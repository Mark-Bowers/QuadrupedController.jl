__precompile__() # this module is safe to precompile

module QuadrupedController
using Conda
using PyCall

export Configuration, four_legs_inverse_kinematics, Controller, State, Command, create_controller_objects

# https://github.com/JuliaPy/PyCall.jl#using-pycall-from-julia-modules
const Configuration = PyNULL()
const four_legs_inverse_kinematics = PyNULL()
const Controller = PyNULL()
const State = PyNULL()
const Command = PyNULL()
const create_controller_objects = PyNULL()

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
    copy!(Command, pyimport("Command").Command)
    copy!(create_controller_objects, pyimport("create_controller_objects").create_controller_objects)
end

end
