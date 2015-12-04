clear all; close all; clc;
import casadi.*;

nk = 20;   
tf = 10.0; 

% Declare variables (use scalar graph)
u  = SX.sym('u');    % control
 x  = SX.sym('x',2); % states

% # ODE right hand side and quadratures
% xdot = vertcat( [(1 - x[1]*x[1])*x[0] - x[1] + u, x[0]] )
% qdot = x[0]*x[0] + x[1]*x[1] + u*u

% # DAE residual function
% dae = SXFunction(daeIn(x=x,p=u),daeOut(ode=xdot, quad=qdot))

% # Create an integrator
% integrator = Integrator("cvodes", dae)
% integrator.setOption("tf",tf/nk) # final time
% integrator.init()

% # All controls (use matrix graph)
% x = MX.sym("x",nk) # nk-by-1 symbolic variable
% U = vertsplit(x) # cheaper than x[0], x[1], ...

% # The initial state (x_0=0, x_1=1)
% X  = MX([0,1])

% # Objective function
% f = 0

% # Build a graph of integrator calls
% for k in range(nk):
%   X,QF = integratorOut(integrator(integratorIn(x0=X,p=U[k])),"xf","qf")
%   f += QF

% # Terminal constraints: x_0(T)=x_1(T)=0
% g = X

% # Allocate an NLP solver
% nlp = MXFunction(nlpIn(x=x),nlpOut(f=f,g=g))
% solver = NlpSolver("ipopt", nlp)
% solver.init()

% # Set bounds and initial guess
% solver.setInput(-0.75, "lbx")
% solver.setInput( 1.,   "ubx")
% solver.setInput( 0.,   "x0")
% solver.setInput( 0.,   "lbg")
% solver.setInput( 0.,   "ubg")

% # Solve the problem
% solver.evaluate()

% # Retrieve the solution
% u_opt = NP.array(solver.getOutput("x"))

% # Time grid
% tgrid_x = NP.linspace(0,10,nk+1)
% tgrid_u = NP.linspace(0,10,nk)

% # Plot the results
% plt.figure(1)
% plt.clf()
% plt.plot(tgrid_u,u_opt,'-.')
% plt.title("Van der Pol optimization - single shooting")
% plt.xlabel('time')
% plt.legend(['u trajectory'])
% plt.grid()
% plt.show()