module QuadrupedController

using StaticArrays

struct Controller
    swingparams
    stanceparams
    gaitparams
    mvref
    jointangles::SMatrix{3, 4, Float64, 12}
    function Controller()

        new(nothing, nothing, nothing, nothing, zeros(SMatrix{3, 4, Float64}))
     end
end

end
