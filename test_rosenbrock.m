clear all; close all; clc;
import casadi.*;


% Create NLP: Solve the Rosenbrock problem:
%     minimize    x^2 + 100*z^2
%     subject to  z + (1-x)^2 - y == 0
x = SX.sym('x');
y = SX.sym('y');
z = SX.sym('z');
v = [x;y;z]
f = x^2 + 100*z^2;
g = z + (1-x)^2 - y;
nlp = SXFunction(nlpIn('x',v),nlpOut('f',f','g',g));

% Create IPOPT solver object
solver = NlpSolver('ipopt', nlp);
solver.init();

solver.setInput([2.5,3.0,0.75],'x0');
solver.setInput(0,'ubg');
solver.setInput(0,'lbg');
solver.evaluate();
 
% Print the solution
disp('-----');
disp(['objective at solution = ', num2str(solver.getOutput('f').getValue)]);
disp('primal solution:');
disp(full(solver.getOutput('x')));
disp('dual solution (x)');
disp(full(solver.getOutput('lam_x')));
disp('dual solution (g)');
disp(full(solver.getOutput('lam_g')));