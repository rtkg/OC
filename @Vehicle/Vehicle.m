classdef Vehicle
    
    properties (Access = protected)
        pose_
        footprint_
        geometries_
    end
    
    methods
        function vehicle = Vehicle()
            import casadi.*;
            % footprint_=Geometry;
            % geometry_=Geometry;
            disp('Constructed Vehicle');
        end

        function print(obj)
            disp('schas');
        end
    end
    
end    