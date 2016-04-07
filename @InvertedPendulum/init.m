function inverted_pendulum = init(inverted_pendulum)


inverted_pendulum.g_=9.81;
inverted_pendulum.m_=1;
inverted_pendulum.l_=1;
inverted_pendulum.c_=1;
inverted_pendulum.x_=[0; 0];
inverted_pendulum.u_=0;
inverted_pendulum.dt_=1e-2;

h(1)=plot(inverted_pendulum.ax_, [0; 0], [0; inverted_pendulum.l_],'k','LineWidth',4); hold on;
h(2)=plot(inverted_pendulum.ax_, 0, inverted_pendulum.l_,'b.','MarkerSize',80);

inverted_pendulum.trans_=hgtransform('Parent', inverted_pendulum.ax_);
set(h,'Parent',inverted_pendulum.trans_);

