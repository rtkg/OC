from quadcopter import *
from casadi.tools import *
from pylab import *

show_3d = False

# This file solves a time-optimal problem
pA = np.array([0,pi/2,0])
pB = np.array([0,3*pi/2,0])

ps = [pA,pB]
rs = [0.8,0.8]

# Import the tracking solution (for initialisation)
import pickle
sol_tracking = pickle.load(file('tracking.pkl','r'))

# Construct the quadcopter model
model = Quadcopter()

# Construct the explicit ODE
R = jacobian(model.r,model.dx)
rhs = - casadi.solve(R, substitute(model.r, model.dx, 0) ) 
f = SXFunction( [ model.x, model.u ], [ rhs ] )
f.init()

N = 200


# This constructs an object that behaves like an MX,
# but has convenient accessors
W = struct_symMX([
    (
     entry("X",struct=model.x,repeat=N+1),
     entry("Z",struct=model.x,repeat=N),
     entry("U",struct=model.u,repeat=N)
    ),
    entry("T")
])  


T = W["T"]
DT = T/N


ts = [0]
t = 0

# Build up the list of constraints
g = []
for i in range(N):
  slope = (W["Z",i]-W["X",i])/(0.5*DT)
  [xdot] = model.f([ W["Z",i], W["U",i] ])
  
  g.append(xdot-slope)
  xpred = W["X",i] + slope*DT
  g.append(xpred-W["X",i+1])
  
  t = t + DT
  ts.append(t)


def norm22(x):
  return mul(x.T,x)

# This time, we have both equalities and inequalities.
# To avoid messy indexing of lbg and ubg,
# we use a structure for conveniece
g = struct_MX([
      entry("collocation",expr=g),
      entry("obstacleA",expr=[ norm22(p[:2]-pA[:2]) for p in W["X",:,"p"] ]), 
      entry("obstacleB",expr=[ norm22(p[:2]-pB[:2]) for p in W["X",:,"p"] ]) 
    ])

# Construct the objective

# Reference quaternion orientation: upright position
qref = array([0,0,0,1])

# To avoid the quadcopter getting lost in spinning or upside down trajectories,
# we add some extra regularisation
R2 = vertcat(W["U",:]+W["X",:,"w"]+[ q-qref for q in W["X",:,"q"] ])

alpha = 0.025

# Time optimality
f = T + (T/N) * alpha*mul(R2.T, R2)

# Create the NLP
nlp = MXFunction( nlpIn(x=W), nlpOut(f=f, g=g) )
nlp.init()

# Create an IPOPT NLP solver
solver = NlpSolver("ipopt",nlp)
solver.setOption("linear_solver","mumps") # todo: remove
# If we need more than 100 iterations, something is wrong
solver.setOption("max_iter",200)
solver.init()

lbg = g(0)
ubg = g(0)

# Set the bounds on the obstacle constraints
lbg["obstacleA"] = rs[0]**2
lbg["obstacleB"] = rs[1]**2

ubg["obstacleA"] = inf
ubg["obstacleB"] = inf

solver.setInput(lbg, "lbg")
solver.setInput(ubg, "ubg")

# Construct and populate the vectors with
# upper and lower bounds simple bounds
lbx = W(-inf)
ubx = W(inf)

lbx["U",:] = 0
ubx["U",:] =  0.5

#  p_0 = [0,0,0]^T
lbx["X",0,"p"] = 0.0
ubx["X",0,"p"] = 0.0

#  v_0 = [0,0,0]^T
lbx["X",0,"v"] = 0.0
ubx["X",0,"v"] = 0.0

# p_N = [0,2*pi,0]^T
lbx["X",-1,"p"] = [0,2*pi,0]
ubx["X",-1,"p"] = [0,2*pi,0]

# v^0_y=0, v^0_z=0
lbx["X",-1,"v"] = [-inf,0,0]
ubx["X",-1,"v"] = [inf,0,0]

x0 = W(0)

# Initialize the horizon length
x0["T"] = 3.0
x0["X",:] = sol_tracking["X",:]
x0["Z",:] = sol_tracking["Z",:]
x0["U",:] = sol_tracking["U",:]

solver.setInput(x0,  "x0")
solver.setInput(lbx, "lbx")
solver.setInput(ubx, "ubx")

# Solve the NLP
solver.evaluate()

# Cast the result vector in a form
# that we can easily access
sol = W(solver.getOutput("x"))

X = sol["X",:,"p","x"]
Y = sol["X",:,"p","y"]
Z = sol["X",:,"p","z"]


# 2D plots
ts = linspace(0,sol["T"],N+1)

figure()
plot(ts,X,label="p_x")
plot(ts,Y,label="p_y")
plot(ts,Z,label="p_z")
xlabel("Time [s]")
legend(loc="upper left")

title("State trajectories")

figure()
plot(X,Y,'.',label="optimized")

for p, r in zip(ps,rs):
  gca().add_patch(Circle(p[:2],r,color='red'))
  gca().add_patch(Circle(p[:2],r,color='red'))

legend()

title("Top down trajectory view")
xlabel("x [m]")
xlabel("y [m]")
axis('equal')

figure()
step(ts,horzcat(sol["U",:]+[ sol["U",-1] ]).T,where='post')
xlabel("Time [s]")
title("Control trajectories")
ylim([-0.1,0.6])

if show_3d:
  # 3D plots
  from mpl_toolkits.mplot3d import Axes3D
  figure()
  ax = gca(projection='3d')

  ax.plot(array(X),array(Y),array(Z),"b.",label="optimized")

  # Plot the rotors
  circle = array([ [cos(t),sin(t),0] for t in linspace(0,2*pi) ]).T*0.05

  for p, q in zip(sol["X",::30,"p"],sol["X",::30,"q"]):
    for offset in [ array([[1,0,0]]),array([[0,1,0]]),array([[-1,0,0]]),array([[0,-1,0]]) ]:
      circle_offset = circle + 0.1*offset.T
      circle_3D = mul(quat(*q), circle_offset)
      ax.plot(
        array(p[0]+circle_3D[0,:]).squeeze(),
        array(p[1]+circle_3D[1,:]).squeeze(),
        array(p[2]+circle_3D[2,:]).squeeze(),
        'k'
      )
      
  # plot the obstacles
  for p, r in zip(ps,rs):
    x=linspace(-1,1,40)
    z=linspace(-3,3,40)

    Xc, Zc= meshgrid(x,z)
    Yc=sqrt(1-Xc**2)*r

    ax.plot_wireframe(p[0]+Xc*r,p[1]+Yc,Zc,color='red')
    ax.plot_wireframe(p[0]+Xc*r,p[1]-Yc,Zc,color='red')
    
  ax.set_xlim([-pi,pi])
  ax.set_ylim([0,2*pi])
  ax.set_zlim([-pi,pi])

show()

