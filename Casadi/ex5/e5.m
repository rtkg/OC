clear all; close all; clc;
import casadi.*;

T = 10; %End time
N = 20; %Number of control intervals
NK = 20; %Number of Runge-Kutta 4 steps per interval and step size
DT = T/(N*NK);
NU = 101; %Number of discrete control values
NX = 101; %# Number of discrete state values