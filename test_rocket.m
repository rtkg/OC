clear all; close all; clc;
import casadi.*;


% Control
u = MX.sym('u');

% State
x = MX.sym('x',3);
s = x(0); % position
v = x(1); % speed
m = x(2); % mass

% ODE right hand side
sdot = v;
vdot = (u - 0.05 * v*v)/m;
mdot = -0.1*u*u;
xdot = vertcat([sdot,vdot,mdot]);

% ODE right hand side function
f = MXFunction({x,u},{xdot});
f.setOption('name','f');
f.init();

%  Integrate with Explicit Euler over 0.2 seconds
 dt = 0.01; %Time step
 xj = x;

 for j = 1:2
   [fj] = f.call({xj,u});
   xj = xj+dt*fj{:}';
 end

 
%  Discrete time dynamics function
 F = MXFunction({x,u},{xj});
 F.init();

%  Number of control segments
 nu = 50;

% Control for all segments
 U = MX.sym('U',nu);
 
% Initial conditions
X0 = MX([0,0,1]);

% Integrate over all intervals
X=X0;
for k =1:nu
  X_ = F.call({X,U(k)});
  X=X_{:};
end

%  Objective function and constraints
J = U'*U; % u'*u in Matlab
 G = X(1:2); % x(1:2) in Matlab (0:2)?

% NLP
 nlp = MXFunction(nlpIn('x',U),nlpOut('f',J,'g',G));
  
% Allocate an NLP solver
 solver = NlpSolver('ipopt', nlp);
 solver.setOption('tol',1e-10);
 solver.setOption('expand',true);
 solver.init();

% Bounds on u and initial condition
solver.setInput(-0.5, 'lbx');
solver.setInput( 0.5, 'ubx');
solver.setInput( 0.4, 'x0');

% Bounds on g
solver.setInput([10,0],'lbg');
solver.setInput([10,0],'ubg');

%  Solve the problem
solver.evaluate();

% Get the solution
 plot(full(solver.getOutput('x'))); hold on; grid on;
 plot(full(solver.getOutput('lam_x')));
