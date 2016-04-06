classdef InvertedPendulum 
    
    properties (Access = protected)
        geometries_
        ax_
	trans_
        g_ %gravity
    end
    
    properties (Access = public)
        x_  %state
        u_  %control
        dt_ %time step 
        m_  %pendulum mass
        l_  %pendulum length
    end
    
    methods 
        init(InvertedPendulum)
        step() %updates the state via integrating the dynamics according to x_{k+1}=f(x_{k},u_{k},dt)
  
		function inverted_pendulum = InvertedPendulum(ax)
            if nargin > 0
                inverted_pendulum.ax_=ax;
            else
                inverted_pendulum.ax_= axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]);
            end












            inverted_pendulum.trans_=hgtransform('Parent', inverted_pendulum.ax_);
            
        end

    end
end    
