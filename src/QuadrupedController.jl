__precompile__() # this module is safe to precompile
module QuadrupedController
using PyCall

export Configuration, four_legs_inverse_kinematics

const Configuration = PyNULL()
const four_legs_inverse_kinematics = PyNULL()

function __init__()
    pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__)    # in order to load a Python module from the current directory

    copy!(Configuration, pyimport("Config").Configuration)
    copy!(four_legs_inverse_kinematics, pyimport("Kinematics").four_legs_inverse_kinematics)
end

end
