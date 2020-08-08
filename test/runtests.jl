using QuadrupedController
using Test

using PyCall

@testset "QuadrupedController.jl" begin
    @test QuadrupedController != PyNULL()
    @test size(names(QuadrupedController)) == (4,)
    @test names(QuadrupedController)[1] == :Configuration
    @test names(QuadrupedController)[2] == :Controller
    @test names(QuadrupedController)[3] == :QuadrupedController
    @test names(QuadrupedController)[4] == :four_legs_inverse_kinematics
end

@testset "Configuration" begin
    @test Configuration != PyNULL()
    @test Configuration.__module__ == "Config"
    @test string(Configuration) == "PyObject <class 'Config.Configuration'>"

    config = Configuration()
    @test config.ABDUCTION_OFFSET == 0.03
end

@testset "four_legs_inverse_kinematics" begin
    @test four_legs_inverse_kinematics != PyNULL()
    @test four_legs_inverse_kinematics.__module__ == "Kinematics"
    @test string(four_legs_inverse_kinematics)[1:47] == "PyObject <function four_legs_inverse_kinematics"
    @test typeof(methods(four_legs_inverse_kinematics).ms[1]) == Method

    np = pyimport("numpy")
    @test four_legs_inverse_kinematics(np.zeros((3,4)), Configuration())[2,4] ≈ -0.2627961442139497
end

@testset "Controller" begin
    @test Controller != PyNULL()
    @test Controller.__module__ == "Controller"
    @test string(Controller) == "PyObject <class 'Controller.Controller'>"

    controller = Controller(Configuration(), four_legs_inverse_kinematics,)
    @test controller.config.ABDUCTION_OFFSET == 0.03
    @test controller.gait_controller.subphase_ticks(49) == 14
    @test controller.gait_controller.subphase_ticks(50) == 0
end
