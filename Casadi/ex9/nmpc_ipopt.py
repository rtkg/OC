from casadi import *
from casadi.tools import *
from plotter import *
from pylab import *

"""
       NOTE: if you use spyder,
           make sure you open a Python interpreter
                 instead of an IPython interpreter
           otherwise you wont see any plots
"""

T = 10.0    # End time
N = 20      # Control discretization (20)
M = 10      # RK discretization (10)

# Declare variables (use scalar graph)
u  = SX.sym("u")    # control
x  = SX.sym("x",2)  # states

# System dynamics
xdot = vertcat( [(1 - x[1]**2)*x[0] - x[1] + u, x[0]] )
f = SXFunction([x,u],[xdot])
f.setOption("name","f")
f.init()

# RK4 with M steps
# also outputs contributions to Gauss-Newton objective
U = MX.sym("U")
X0 = MX.sym("X0",2)
DT = T/(N*M)
XF = X0
QF = 0
R_terms = [] # Terms in the Gauss-Newton objective
for j in range(M):
    [k1] = f([XF,             U])
    [k2] = f([XF + DT/2 * k1, U])
    [k3] = f([XF + DT/2 * k2, U])
    [k4] = f([XF + DT   * k3, U])
    XF += DT/6*(k1   + 2*k2   + 2*k3   + k4)
    R_terms.append(XF)
    R_terms.append(U)
R_terms = vertcat(R_terms) # Concatenate terms
F = MXFunction([X0,U],[XF,R_terms])
F.setOption("name","F")
F.init()

# Define NLP variables
W = struct_symMX([
      (
        entry("X",shape=(2,1),repeat=N+1),
        entry("U",shape=(1,1),repeat=N)
      )
])


# NLP constraints
g = []

# Terms in the Gauss-Newton objective
R = []

# Build up a graph of integrator calls
for k in range(N):
    # Call the integrator
    [x_next_k, R_terms] = F([ W["X",k], W["U",k] ])

    # Append continuity constraints
    g.append(x_next_k - W["X",k+1])

    # Append Gauss-Newton objective terms
    R.append(R_terms)

# Concatenate constraints
g = vertcat(g)

# Concatenate terms in Gauss-Newton objective
R = vertcat(R)



# Objective function
obj = mul(R.T,R)/2

# Create an NLP solver object
nlp = MXFunction(nlpIn(x=W),nlpOut(f=obj,g=g))
nlp_solver = NlpSolver("ipopt", nlp)
nlp_solver.setOption("linear_solver", "mumps")
nlp_solver.init()

# All constraints are equality constraints in this case
nlp_solver.setInput(0, "lbg")
nlp_solver.setInput(0, "ubg")

# Construct and populate the vectors with
# upper and lower simple bounds
w_min = W(-inf)
w_max = W( inf)

# Control bounds
w_min["U",:] = -1
w_max["U",:] = 1

w_k = W(0)

ts = linspace(0,T,N+1)
plotter = Plotter(ts)
t = 0
x_current = array([1,0])
while True:
  w_min["X",0] = x_current
  w_max["X",0] = x_current

  # Pass data to NLP solver
  nlp_solver.setInput(w_k,"x0")
  nlp_solver.setInput(w_min,"lbx")
  nlp_solver.setInput(w_max,"ubx")
   
  # Solve the OCP
  nlp_solver.evaluate()
    
  # Extract from the solution the first control
  sol = W(nlp_solver.getOutput("x"))

  print "solx-1"
  print sol["X",-1]
  print "solu-1"
  print sol["U",-1]
  exit()

  # Plot the solution
  plotter.show(t,x_current,sol)


  import sys
  sys.stdout.write('Waiting for your input (<enter>, "quit|clip|clear", or numbers ):')
  wait = raw_input()
  if "quit" in wait:
    break
  if "clear" in wait:
    plotter.clear()
  if "clip" in wait:
    plotter.toggleClipping()
  try:  # Easier to Ask Forgiveness than Permission
    x_current[:] = array(map(float,wait.split(" ")))
  except:
    pass
    
  # Simulate the system with this control
  F.setInput( x_current,0)
  F.setInput( u_nmpc ,1)
  F.evaluate()
  
  # Update the current state
  x_current = F.getOutput(0)
  
  t += T/N
  # Shift the time to have a better initial guess
  # For the next time horizon
  w_k["X",:-1] = sol["X",1:]
  w_k["U",:-1] = sol["U",1:]
  w_k["X",-1] = sol["X",-1]
  w_k["U",-1] = sol["U",-1]
  

