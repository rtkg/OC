function cititruck = computePosture(cititruck)
    
%set the steering wheel angle
set(cititruck.geometries_{2}.trans_,'Matrix',makehgtform('zrotate',cititruck.s_(4))); 
   
%transformation expressing the vehicle in the world frame (V_W_T) 
V_W_T = makehgtform('zrotate',cititruck.s_(3)); V_W_T(1:2,4)=cititruck.s_(1:2);
set(cititruck.trans_,'Matrix',V_W_T);

drawnow;
end