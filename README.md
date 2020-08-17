# QuadrupedController

## Overview
This repository hosts a Julia wrapper for the code controller for Stanford Pupper and Stanford Woofer, Raspberry Pi-based quadruped robots that can trot, walk, and jump.

Video of pupper in action: https://youtu.be/NIjodHA78UE

Project page: https://stanfordstudentrobotics.org/pupper

## Installation
From the Julia REPL use '[' key to access the package manager then add https://github.com/Mark-Bowers/QuadrupedController.jl.git.

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

julia> robot = Robot(config);

julia> toggle_activate(robot, command)
Toggling from DEACTIVATED: -1 to REST: 0

julia> toggle_trot(robot, command)
Toggling from REST: 0 to TROT: 1
```