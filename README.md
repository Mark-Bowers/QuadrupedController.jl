# QuadrupedController

## Overview
This repository hosts a Julia wrapper for the code controller for Stanford Pupper and Stanford Woofer, Raspberry Pi-based quadruped robots that can trot, walk, and jump.

Video of pupper in action: https://youtu.be/NIjodHA78UE

Project page: https://stanfordstudentrobotics.org/pupper

## Installation
From the Julia REPL use ']' key to access the package manager then add https://github.com/Mark-Bowers/QuadrupedController.jl.git.

```julia-repl
julia> ]

(@v1.5) pkg> add https://github.com/Mark-Bowers/QuadrupedController.jl.git
   Updating git-repo `https://github.com/Mark-Bowers/QuadrupedController.jl.git`
   ...
```
## Using QuadrupedController
```julia-repl
julia> using QuadrupedController
[ Info: Precompiling QuadrupedController [0a0771de-1099-46b6-8518-8474b76bc44f]

julia> config = Configuration();

julia> config.z_clearance = 0.01;   # height to pick up each foot during trot

julia> # Initialize a command struct

julia> # Pupper walks in circles with yaw set to non-zero value

julia> const crouch_height = -0.08;

julia> const normal_height = -0.16;

julia> command = Command([0.4, 0.0], 0.5, crouch_height, 0.1)
Command([0.4, 0.0], 0.5, -0.08, 0.1, 0.0, 0, false, false, false)

julia> robot.state.joint_angles
3×4 Array{Float64,2}:
 0.0  0.0  0.0  0.0
 0.0  0.0  0.0  0.0
 0.0  0.0  0.0  0.0

julia> toggle_activate(robot)
Toggling from DEACTIVATED: -1 to REST: 0

julia> robot.state.joint_angles
3×4 Array{Float64,2}:
 -0.221357   0.20312  -0.252215   0.278054
  1.1608     1.14737   1.23489    1.20753
 -1.12729   -1.15074  -1.2256    -1.24252

julia> toggle_trot(robot)
Toggling from REST: 0 to TROT: 1

julia> robot.state.joint_angles
3×4 Array{Float64,2}:
 -0.218254   0.20806  -0.260656   0.272427
  1.1977     1.19121   1.27731    1.26334
 -1.09238   -1.10386  -1.17945   -1.18832

julia> run!(robot)

julia> robot.state.joint_angles
3×4 Array{Float64,2}:
 -0.224001   0.203941  -0.2557    0.279818
  1.24024    1.22791    1.33272   1.30435
 -1.04501   -1.06778   -1.12335  -1.14214

```