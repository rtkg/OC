clear all; close all; clc;
import casadi.*;

% Params
p_ =MX.sym('p',2,1);

% Control
u_ = MX.sym('u');

% State
x_ = MX.sym('x');
dx_ = p_(1)*x_+p_(2)*u_;



f = MXFunction({x_,p_,u_},{dx_});
f.setOption('name','f');
f.init();


%  Integrate with Explicit Euler over 0.2 seconds
dt = 0.01; %Time step
xj_ = x_;

for j = 1:2
    [fj_] = f.call({xj_,p_,u_});
    xj_ = xj_+dt*fj_{:}';
end

%  Discrete time dynamics function
F = MXFunction({x_,p_,u_},{xj_});
F.setOption('name','F');
F.init();

%  Number of control segments
nu=50;

% Control for all segments
 U_ = MX.sym('U',nu);
 
% Initial conditions
X0_ = MX(1);
X_=X0_;
for k =1:nu
  X = F.call({X_,p_,U_(k)});
  X_=X{:};
end