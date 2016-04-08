classdef Segway
	
	properties (Access = protected)
		trans_
		ax_
	end
	
	properties  (Access = public)
		
		g_  %gravity
		m_  %segway pendulum mass
		M_  %segway base mass
		l_  %segway pendulum length
		x_  %segway state [x; dx; phi; dphi]
		u_  %control
		dt_ %sampling rate [s]
	end
	
	methods (Access = protected)
		function dx=dynamics(segway, t,x)
			
			g=segway.g_;
			m=segway.m_;
			M=segway.M_;
			l=segway.l_;
			u=segway.u_;
			
			dx=zeros(4,1);
			dx(1)=x(2);
			dx(2)=1/(M+m*sin(x(3))^2)*(u+m*g*cos(x(3))*sin(x(3))-m*l*sin(x(3))*x(4)^2);
			dx(3)=x(4);
			dx(4)=1/l/(M+sin(x(3))^2)*(g*(M+m)*sin(x(3))+cos(x(3))*(u-m*l*sin(x(3))*x(4)^2));
		end
		
		segway = init(segway);
	end
	
	methods
		
		function segway = Segway(ax)
			if nargin == 0
				segway.ax_ = axes;
			else
				segway.ax_=ax;
			end
			
			segway = segway.init;
		end
		
		function segway = set.x_(segway, x)
			assert(isvector(x));
			assert(numel(x) == 4);
			segway.x_=x;
			T = makehgtform('zrotate',x(3));
			T(1:3,4)=[x(1); 0; 0];
			set(segway.trans_,'Matrix',T);
			drawnow;
		end
		
		x_new=step(segway);
		
	end
	
end
