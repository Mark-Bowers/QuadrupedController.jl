using QuadrupedController
using Test

using PyCall

@testset "QuadrupedController.jl" begin
    @test QuadrupedController != PyNULL()
    @test size(names(QuadrupedController)) == (3,)
    @test names(QuadrupedController)[1] == :Configuration
    @test names(QuadrupedController)[3] == :four_legs_inverse_kinematics
end

@testset "Configuration" begin
    @test QuadrupedController.Configuration != PyNULL()
    @test QuadrupedController.Configuration.__module__ == "Config"
    @test string(QuadrupedController.Configuration) == "PyObject <class 'Config.Configuration'>"

    config = QuadrupedController.Configuration()
    @test config.ABDUCTION_OFFSET == 0.03
end

@testset "four_legs_inverse_kinematics" begin
    @test QuadrupedController.four_legs_inverse_kinematics != PyNULL()
    @test QuadrupedController.four_legs_inverse_kinematics.__module__ == "Kinematics"
    @test string(QuadrupedController.four_legs_inverse_kinematics)[1:47] == "PyObject <function four_legs_inverse_kinematics"
    @test typeof(methods(QuadrupedController.four_legs_inverse_kinematics).ms[1]) == Method

    np = pyimport("numpy")
    @test QuadrupedController.four_legs_inverse_kinematics(np.zeros((3,4)), QuadrupedController.Configuration())[2,4] ≈ -0.2627961442139497
end
