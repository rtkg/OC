classdef Vehicle 
    
    properties (Access = protected)
        footprint_
        geometries_
        ax_
		trans_
    end
    
    properties (Abstract)
        s_ %state for visualization
		f_ %dynamics
    end
    
    methods (Abstract)
        init(vehicle)
        computePosture(vehicle)
    end
    
    methods
		function vehicle = Vehicle(ax)
            if nargin > 0
                vehicle.ax_=ax;
            else
                vehicle.ax_= axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]);
            end

            vehicle.trans_=hgtransform('Parent', vehicle.ax_);
            
        end

    end
end    