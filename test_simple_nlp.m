clear all; close all; clc;
import casadi.*;

% Declare variables
x = SX.sym('x',2);

f = x(0)^2 + x(1)^2; % objective
g = x(0)+x(1)-10;     % constraint

% Form the NLP
nlp = SXFunction(nlpIn('x',x),nlpOut('f',f,'g',g));

%Pick an NLP solver
MySolver = 'ipopt';
%MySolver = 'sqpmethod';
% MySolver = 'worhp';(I.call(integratorIn('x0',X,'p',U(k)))


% Allocate a solver
solver = NlpSolver(MySolver, nlp);
if (strcmp(MySolver,'sqpmethod'))
    solver.setOption('qp_solver','qpoases');
    solver.setOption('qp_solver_options',{'printLevel', 'none'});
end

solver.init()

% Set constraint bounds
solver.setInput(0.,'lbg');

% Solve the NLP
solver.evaluate();

% Print solution
disp('-----');
disp(['objective at solution = ', num2str(solver.getOutput('f').getValue)]);
disp('primal solution:');
disp(solver.getOutput('x'));
disp('dual solution (x)');
disp(solver.getOutput('lam_x'));
disp('dual solution (g)');
disp(solver.getOutput('lam_g'));

