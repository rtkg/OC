function cititruck = init(cititruck)
cititruck.l_=1.7; %axle distance

%define the geometries and footprint

%%%%%%%%%%%%%%%%% vehicle frame %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
geometries{1}=Frame(1,'V',cititruck.ax_);

%%%%%%%%%%%%%%%%% steered wheel %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vertices=[-0.15 -0.05 0;
	-0.15  0.05 0;
	0.15  0.05 0;
	0.15 -0.05 0];
geometries{2}=Polygon([1 2 3 4],vertices,cititruck.ax_,'k',1,'k',1);

%%%%%%%%%%%%%%%%% passive wheels %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vertices=[-0.1 -0.05 0;
	-0.1  0.05 0;
	0.1  0.05 0;
	0.1 -0.05 0];
geometries{3}=Polygon([1 2 3 4],vertices,cititruck.ax_,'k',1,'k',1);
geometries{4}=Polygon([1 2 3 4],vertices,cititruck.ax_,'k',1,'k',1);

%the wheel geometries expressed in the vehicle frame
set(geometries{3}.trans_,'Matrix',makehgtform('translate',[cititruck.l_ 0.425 0]));
set(geometries{4}.trans_,'Matrix',makehgtform('translate',[cititruck.l_ -0.425 0]));
%%%%%%%%%%%%%%%%% CitiTruck body %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vertices=[0    0.5  0;
	1.6  0.5  0;
	1.6  0.35 0;
	0.4  0.35 0;
	0.4 -0.35 0;
	1.6 -0.35 0;
	1.6 -0.5  0;
	0   -0.5  0];
geometries{5}=Polygon(1:8,vertices,cititruck.ax_,'k',1,'k',0.2);
%the body geometry expressed in the vehicle frame
set(geometries{5}.trans_,'Matrix',makehgtform('translate',[0.3 0 0]));

%%%%%%%%%%%%%%%%% Footprint %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vertices=[-1.125  0.6 0;
	1.125  0.6 0;
	1.125 -0.6 0;
	-1.125 -0.6 0];
footprint=Polygon(1:4,vertices,cititruck.ax_,'m',0.5,'m',0);
set(footprint.trans_,'Matrix',makehgtform('translate',[0.875 0 0]));

set(footprint.trans_,'Parent',cititruck.trans_);
for i=1:length(geometries)
	set(geometries{i}.trans_,'Parent',cititruck.trans_);
end

cititruck.footprint_=footprint;
cititruck.geometries_=geometries;

%%%%%%%%%%%%%%%%%%%%%%% Dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DifferentialState x;
DifferentialState y;
DifferentialState th;
DifferentialState ph;
DifferentialState L;

Control v;
Control w;

f=acado.DifferentialEquation;
f.add(dot(x) == v*cos(th));
f.add(dot(y) == v*sin(th));
f.add(dot(th) == v/cititruck.l_*tan(ph));
f.add(dot(ph) == w);
f.add(dot(L) == v*v+w*w); % Dummy equation to integrate used power


cititruck.x_=x;
cititruck.y_=y;
cititruck.th_= th;
cititruck.ph_= ph;
cititruck.L_= L;
cititruck.v_=v;
cititruck.w_=w;
cititruck.f_=f;
end