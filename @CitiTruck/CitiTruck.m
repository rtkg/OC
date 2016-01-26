classdef CitiTruck < Vehicle
    
    properties 
        s_
		f_
		x_
		y_
		th_
		ph_
		L_
		v_
		w_
    end

    properties (Access = private)
        l_ %axle distance
	end
	

    methods
        function cititruck = CitiTruck(s,ax)
            if nargin > 0      
                args{1}=s; %s=[x,y,theta, phi] (position_x,position_y, heading, steering angle)
            else
                args{1}=zeros(4,1);
            end
            
            if nargin > 1
                super_arg=ax;
            else
                super_arg= axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]);
            end
            
            cititruck = cititruck@Vehicle(super_arg);
			cititruck = cititruck.init;
			cititruck.s_=args{1};
            %cititruck = cititruck.computePosture;  
			
        end
        
        function cititruck = set.s_(cititruck, s)
            assert(isvector(s));
            assert(numel(s) == 4);
            assert((s(3) > -pi) && (s(3) < pi));
            assert((s(4) > -pi) && (s(4) < pi)); 
			cititruck.s_=s; 
			cititruck = cititruck.computePosture;  
        end 
        
        cititruck = init(cititruck)
        cititruck = computePosture(cititruck)
        
    end
    
end    