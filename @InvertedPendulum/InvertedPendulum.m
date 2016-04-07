classdef InvertedPendulum
	
	properties (Access = protected)
		g_  %gravity
		m_  %pendulum mass
		l_  %pendulum length
		c_  %input gain
		ax_
		trans_
	end
	
	properties  (Access = public)
		x_  %pendulum state [phi; dphi]
		u_  %control
		dt_ %sampling rate [s]
	end
	
	methods (Access = protected)
		function dx=dynamics(inverted_pendulum, t,x)
			dx=[x(2); -inverted_pendulum.g_/inverted_pendulum.l_*sin(x(1))+inverted_pendulum.c_*inverted_pendulum.u_];
		end
		
		inverted_pendulum = init(inverted_pendulum);
	end
	
	methods
		
		function inverted_pendulum = InvertedPendulum(ax)
			if nargin == 0
				inverted_pendulum.ax_ = axes;
			else
				inverted_pendulum.ax_=ax;
			end
			
			
			inverted_pendulum = inverted_pendulum.init;
		end
		
		function inverted_pendulum = set.x_(inverted_pendulum, x)
			assert(isvector(x));
			assert(numel(x) == 2);
			assert((x(1) > -pi) && (x(1) <= pi));
			inverted_pendulum.x_=x;
			Rz = makehgtform('zrotate',x(1));
			set(inverted_pendulum.trans_,'Matrix',Rz);
			drawnow;
		end
		
		x_new=step(inverted_pendulum);
		
	end
	
end
