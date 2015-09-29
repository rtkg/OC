classdef CitiTruck < Vehicle
    
    properties 
        x_
    end

    properties (Access = private)
        l_ %axle distance
    end
    
    methods
        function cititruck = CitiTruck(x,ax)
            if nargin > 0      
                args{1}=x; %x=[x,y,theta, phi] (position_x,y, heading, steering angle)
            else
                args{1}=zeros(4,1);
            end
            
            if nargin > 1
                super_arg=ax;
            else
                super_arg= axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]);
            end
            
            cititruck = cititruck@Vehicle(super_arg);
            cititruck.x_=args{1};
            
            cititruck = cititruck.init;
               cititruck = cititruck.computePosture;  
        end
        
        function cititruck = set.x_(cititruck, x)
            assert(isvector(x));
            assert(numel(x) == 4);
            assert((x(3) > -pi) && (x(3) < pi));
            assert((x(4) > -pi) && (x(4) < pi)); 
            cititruck.x_=x; 
        end 
        
        cititruck = init(cititruck)
        cititruck = computePosture(cititruck)
        
    end
    
end    