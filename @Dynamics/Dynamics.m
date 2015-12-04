classdef Dynamics
    properties (Access = public)
        x_ 
        x_prev_
    end
    
    properties (Access = protected)
        f_ 
    end    
    
    methods
        function dynamics = Dynamics(f)
            import casadi.*;
            dynamics.f_=f;
        end
        
        function dynamics = set.f_(dynamics, f)
        %TODO: VALIDATE f HERE!!!!
            dynamics.f_=f;
        end 
        
    end
end