clear all; close all;
addpath(genpath('../'));

tend=10;
intv=100;
s0=[-1;-1;0];


BEGIN_ACADO
acadoSet('problemname', 'od_ocp_test');

DifferentialState x;
DifferentialState y;
DifferentialState th;
DifferentialState L;
Control v;
Control w;

f = acado.DifferentialEquation();       % Set the differential equation object
f.add(dot(x) == v*cos(th));
f.add(dot(y) == v*sin(th));
f.add(dot(th) == w);
f.add(dot(L) == w*w);

% Optimal Control Problem
ocp = acado.OCP(0.0, tend);

ocp.minimizeMayerTerm(L);               % Minimize the consumed energy
ocp.subjectTo( f );                     % Optimize with respect to the differential equation
ocp.subjectTo( 'AT_START', x ==  s0(1) );
ocp.subjectTo( 'AT_START', y ==  s0(2) );
ocp.subjectTo( 'AT_START', th ==  s0(3) );
ocp.subjectTo( 'AT_END', x ==  0.0 );
ocp.subjectTo( 'AT_END', y ==  0.0 );

% Optimization Algorithm
algo =acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm, link it to your OCP
%algo.set( 'KKT_TOLERANCE', 1e-6 );      % Set a custom KKT tolerance

END_ACADO

out = od_ocp_test_RUN();
