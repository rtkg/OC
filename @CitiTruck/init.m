function cititruck = init(cititruck)
%define the geometries and footprint

%%%%%%%%%%%%%%%%% vehicle frame %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 geometries{1}=Frame(1,'V',cititruck.ax_);
 %%%%%%%%%%%%%%%%% steered wheel %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vertices=[-0.15 -0.05 0;
          -0.15  0.05 0;
           0.15  0.05 0;
           0.15 -0.05 0];    
geometries{2}=Polygon([1 2 3 4],vertices,cititruck.ax_,'k',1,'k',1);
 %the wheel geometry expressed in the vehicle frame
  set(geometries{2}.trans_,'Matrix',makehgtform('translate',[-0.7 0 0])); 
%%%%%%%%%%%%%%%%% CitiTruck body %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
vertices=[0    0.5  0;
          1.5  0.5  0;
          1.5  0.35 0;
          0.4  0.35 0;
          0.4 -0.35 0;
          1.5 -0.35 0;
          1.5 -0.5  0;
          0   -0.5  0];
geometries{3}=Polygon(1:8,vertices,cititruck.ax_,'k',1,'k',0.2);
%the body geometry expressed in the vehicle frame
 set(geometries{3}.trans_,'Matrix',makehgtform('translate',[-0.5 0 0]));
drawnow;


%    cititruck.footprint_=footprint;
    cititruck.geometries_=geometries;
    
    
    for i=1:length(geometries)
       set(geometries{i}.trans_,'Parent',cititruck.trans_);
    end
end