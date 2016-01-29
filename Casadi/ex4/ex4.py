#import casadi
import numpy
from casadi import *

px0 = 0.0
py0 = 1.5

pxF = 20.0
pyF = 0.0

T = 3.0
LINEAR = False
#LINEAR = True

CVODES = False
#CVODES = True

def dxdt(x,cat=veccat):
    px = x[0]
    vx = x[1]
    py = x[2]
    vy = x[3]

    v = sqrt(vx*vx + vy*vy)
    if LINEAR:
        beta = 0.02
        return cat([vx,
                    -beta*vx,
                    vy,
                    -beta*vy - 9.81])
    else:
        alpha = 0.02
        return cat([vx,
                    -alpha*v*vx,
                    vy,
                    -alpha*v*vy - 9.81])

# rk4
N = 20
h = T/float(N)

x = numpy.array([px0, 5.0, py0, 5.0])
#x = numpy.array([px0, 9.07829, py0, 16.1732])

for k in range(N):
    k1 = h*dxdt(x         , numpy.array)
    k2 = h*dxdt(x + 0.5*k1, numpy.array)
    k3 = h*dxdt(x + 0.5*k2, numpy.array)
    k4 = h*dxdt(x +     k3, numpy.array)
    x = x + (k1 + 2*k2 + 2*k3 + k4)/6.0
print "rk4: " + str(x)

# IPOPT
x = SX.sym('x',4)
dx = dxdt(x,cat=veccat)
dae = SXFunction( [x], [dx] )
dae.setOption("name","dae")
dae.init()

v0 = MX.sym("v0",2)
x = vertcat((px0, v0[0], py0, v0[1]))

x0 = SX.sym('x',4)
integrator = Integrator("cvodes",SXFunction(daeIn(x=x0), daeOut(ode=dxdt(x0))))
integrator.setOption('name','integrator')
integrator.init()

for k in range(N):
    if CVODES:
        x = integratorOut(integrator( integratorIn(x0=x) ), "xf")[0]
    else:
        k1 = h*dae([x         ])[0]
        k2 = h*dae([x + 0.5*k1])[0]
        k3 = h*dae([x + 0.5*k2])[0]
        k4 = h*dae([x +     k3])[0]
        x = x + (k1 + 2*k2 + 2*k3 + k4)/6.0

pf = vertcat((x[0],x[2]))
F = MXFunction([v0],[pf])
F.init()

F.setInput([5,5])
F.evaluate()
print "MXFunction: " + str(F.getOutput())

nlp = MXFunction( nlpIn(x=v0), nlpOut(f=0, g=pf - veccat([pxF,pyF])) )
solver = NlpSolver("ipopt",nlp)
solver.setOption("max_iter",30)
solver.init()

solver.setInput([5,5], "x0")
solver.setInput(0, "lbg")
solver.setInput(0, "ubg")

solver.evaluate()
print "ipopt:         " + str(solver.getOutput("x"))


# newton scheme
F = MXFunction([v0],[pf - veccat([pxF,pyF])])
F.init()


Fjac = F.jacobian()
Fjac.init()

v0 = numpy.array([5.0, 5.0])
tol = 1.0e-7
for k in range(1000):
    Fjac.setInput(v0)
    Fjac.evaluate()
    J = Fjac.getOutput(0)
    f = Fjac.getOutput(1)
    dv0 = - numpy.linalg.solve(J, f)[:,0]

    norm_step = numpy.linalg.norm(dv0)
    print "newton step " + str(k) + ": " + str(DMatrix(v0)) + ", step: " + str(norm_step)
    if norm_step <= tol:
        kf = k
        break
    v0 += dv0
#    v0 -= 
print "newton step converged in " + str(k) + " iterations (tolerance: " + str(tol) + ")"

#dae = casadi.SXFunction(casadi.daeIn(x=x), casadi.daeOut(ode=casadi.veccat([x[1],-x[0]])))
#integrator = casadi.Integrator("cvodes", dae)
#integrator.setOption("tf",Tf)
#integrator.init()
#
#integrator.setInput([10.0, 0.0], "x0")
#integrator.evaluate()
#cvodes_x10 = integrator.getOutput("xf")
#print "cvodes x(10): " + str(cvodes_x10)
#
#plt.show()
