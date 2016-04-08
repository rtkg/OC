function segway = init(segway)

segway.g_=9.81;
segway.l_=1;
segway.m_=1;
segway.M_=1;
segway.x_=[0; 0; 0; 0];
segway.u_=0;
segway.dt_=1e-2;

h(1)=rectangle(segway.ax_,'Position',[-0.02 0 0.04 segway.l_],'FaceColor','k','EdgeColor','k');
h(2)=rectangle(segway.ax_,'Position',[-0.2 -0.2 0.4 0.4],'Curvature',[1 1],'FaceColor',[.8 .8 .8],'EdgeColor','k','LineWidth',4);
h(3)=rectangle(segway.ax_,'Position',[-0.035 -0.035 0.07 0.07],'Curvature',[1 1],'FaceColor','k','EdgeColor','k');
h(4)=rectangle(segway.ax_,'Position',[-0.08 segway.l_-0.08 0.16 0.16],'Curvature',[1 1],'FaceColor','b','EdgeColor','b');

segway.trans_=hgtransform('Parent', segway.ax_);
set(h,'Parent',segway.trans_);

