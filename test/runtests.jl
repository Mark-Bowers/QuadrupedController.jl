using QuadrupedController
using Test

using PyCall

@testset "QuadrupedController.jl" begin
    @test QuadrupedController != PyNULL()
    @test size(names(QuadrupedController)) == (10,)
    @test names(QuadrupedController)[1] == :Command
    @test names(QuadrupedController)[2] == :Configuration
    @test names(QuadrupedController)[3] == :QuadrupedController
    @test names(QuadrupedController)[4] == :Robot
    @test names(QuadrupedController)[5] == :behavior_state
    @test names(QuadrupedController)[6] == :behavior_state_string
    @test names(QuadrupedController)[7] == :run!
    @test names(QuadrupedController)[8] == :toggle_activate
    @test names(QuadrupedController)[9] == :toggle_hop
    @test names(QuadrupedController)[10] == :toggle_trot
end

@testset "Command" begin
    command = Command()
    @test string(command) == "Command([0.0, 0.0], 0.0, -0.16, 0.0, 0.0, 0, false, false, false)"
end

@testset "Configuration" begin
    @test Configuration != PyNULL()
    @test Configuration.__module__ == "Config"
    @test string(Configuration) == "PyObject <class 'Config.Configuration'>"

    config = Configuration()
    @test config.ABDUCTION_OFFSET == 0.03
end

@testset "Robot" begin
    config = Configuration()
    @test config.z_clearance == 0.07
    config.z_clearance = 0.01
    command = Command([0.4, 0.0], 0.5, -0.08, 0.1)
    @test string(command) == "Command([0.4, 0.0], 0.5, -0.08, 0.1, 0.0, 0, false, false, false)"
    robot = Robot(config, command)
    @test behavior_state(robot.state) == -1 # DEACTIVATED
    @test behavior_state_string(robot.state) == "DEACTIVATED: -1"
    toggle_activate(robot)
    @test behavior_state(robot.state) == 0  # REST
    @test behavior_state_string(robot.state) == "REST: 0"
    toggle_trot(robot)
    @test behavior_state(robot.state) == 1  # TROT
    @test behavior_state_string(robot.state) == "TROT: 1"
end
