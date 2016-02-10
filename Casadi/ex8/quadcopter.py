from casadi import *
from casadi.tools import *

import numpy
from numpy import cos,sin, vstack, hstack, multiply

class Quadcopter:
  """
   
   Quadcopter model
   
   ::
   
     states = [
               p_x     \\
               p_y      (p: centre of mass position)
               p_z     /
               v_x     \\
               v_y      (v: centre of mass velocity)
               v_z     /
               q_0     \\
               q_1      (q: orientation quaternions)
               q_2      /
               q_3     /              
               w_0     \\
               w_1      (w: angular velocity)
               w_2     /
               r_0     \\
               r_1      (r: rotor speeds)
               r_2      /
               r_3     /
              ]
               
     controls =  [
                  CR_0  \\
                  CR_1   (CR: motor torques)
                  CR_2   /
                  CR_3  /
                 ]
   
   Main usage:
   
   -----------
   from model import *
   model = QuadcopterModel()
   -----------
  """
  
  def __init__(self):
    self._model = QuadcopterModel()
    
    self.x  = self._model.states    
    self.dx = self._model.dstates
    self.u  = self._model.controls
    
    self.r = substitute([self._model.res_],
               [self.dx,self.x,self.u],
               [self._model.scaling_states*SX(self.dx),
                self._model.scaling_states*SX(self.x),
                self._model.scaling_controls*SX(self.u)
                ])[0]
    
    self.r0 = 403.60380987049399/self._model.scaling_states["r",0]
    self.u0 = 0.00261746256/self._model.scaling_controls["CR",0]
    
    self.x0 = self.x(0)
    self.x0["p","z"] = 1
    self.x0["q",3] = 1
    self.x0["r",:] = self.r0

    R = jacobian(self.r,self.dx)
    rhs = - casadi.solve(R, substitute(self.r, self.dx, 0) ) 
    self.f = SXFunction( [ self.x, self.u ], [ rhs ] )
    self.f.init()

class QuadcopterModel:
  """
   
   Quadcopter model
   
   ::
   
     states = [
               p_x     \\
               p_y      (p: centre of mass position)
               p_z     /
               v_x     \\
               v_y      (v: centre of mass velocity)
               v_z     /
               q_0     \\
               q_1      (q: orientation quaternions)
               q_2      /
               q_3     /              
               w_0     \\
               w_1      (w: angular velocity)
               w_2     /
               r_0     \\
               r_1      (r: rotor speeds)
               r_2      /
               r_3     /
              ]
               
     controls =  [
                  CR_0  \\
                  CR_1   (CR: motor torques)
                  CR_2   /
                  CR_3  /
                 ]
   
   Main usage:
   
   -----------
   from model import *
   model = QuadcopterModel()
   
   x  = model.x   # states
   dx = model.dx  # state derivatives
   u  = model.u   # controls
   -----------
   
  """
  def __init__(self,debug=False,quatnorm=False,qstab=False):
    """
    Keyword arguments:
      debug -- wether to print out debug info
      quatnorm -- add the quaternion norm to the DAE rhs
    """

    # ----------- syistem states and their derivatives ----
    pos = struct_symSX(["x","y","z"])     # rigid body centre of mass position [m]   {0}   
    v   = struct_symSX(["x","y","z"])  # rigid body centre of mass position velocity [m/s] {0}

    NR = 4                               # Number of rotors
    
    states = struct_symSX([
      entry("p",struct=pos),
      entry("v",struct=v),
      entry("q",shape=4),                # quaternions  {0} -> {1}
      entry("w",shape=3),                # rigid body angular velocity w_101 [rad/s] {1}
      entry("r",shape=NR)                # spin speed of rotor, wrt to platform. [rad/s] Should be positive!
                                         # The signs are such that positive means lift generating, regardless of spin direction.
      
    ])
    
    pos, v, q, w, r = states[...]

    # ------------------------------------------------

    dist = struct_symSX([
      entry("Faer",shape=3),             # Disturbance on aerodynamic forcing [N]
      entry("Caer",shape=3)             # Disturbance on aerodynamic torques [Nm]
    ])


    # ----------------- Controls ---------------------
    controls = struct_symSX([
      entry("CR",shape=NR)              # [Nm]
          # Torques of the motors that drive the rotors, acting from platform on propeller
          # The torque signs are always positive when putting energy in the propellor,
          # regardless of spin direction.
          # 
    ])
    
    CR = controls["CR"]
    
    # ------------------------------------------------


    # ----------------  Temporary symbols --------------
    F = SX.sym("F",3)          # Forces acting on the platform in {1} [N]
    C = SX.sym("C",3)          # Torques acting on the platform in {1} [Nm]

    rotors_Faer = [SX.sym("Faer_%d" %i,3,1) for i in range(NR)] # Placeholder for aerodynamic force acting on propeller {1} [N]
    rotors_Caer = [SX.sym("Caer_%d" %i,3,1) for i in range(NR)] # Placeholder for aerodynamic torques acting on propeller {1} [Nm]

    # ---------------------------------------------------


    # ----------------- Parameters ---------------------
    
    rotor_model = struct_symSX([
         "c",        # c          Cord length [m]
         "R",        # R          Radius of propeller [m]
         "CL_alpha", # CL_alpha   Lift coefficient [-]
         "alpha_0",  # alpha_0
         "CD_alpha", # CD_alpha   Drag coefficient [-]
         "CD_i",     # CD_i       Induced drag coefficient [-]  
    ])
    
    p = struct_symSX([
      entry("rotors_model",repeat=NR,struct=rotor_model),    # Parameters that describe the rotor model
      entry("rotors_I",repeat=NR,shape=Sparsity.diag(3)),  # Inertias of rotors [kg.m^2]
      entry("rotors_spin",repeat=NR),    # Direction of spin from each rotor. 1 means rotation around positive z.
      entry("rotors_p",repeat=NR,shape=3),  # position of rotors in {1} [m],
      entry("I",sym=diag(SX.sym("[Ix,Iy,Iz]"))), # Inertia of rigid body [kg.m^2]
      "m",       # Mass of the whole system [kg]
      "g",       # gravity [m/s^2]
      "rho",     # Air density [kg/m^3]
    ])
    
    I,m,g,rho = p[["I","m","g","rho"]]
 
    # --------------------------------------------------

   
    # ----------------- Parameters fillin's ---------------------

    p_ = p()
    p_["rotors_spin"] = [1,-1,1,-1]

    p_["rotors_model",:,{}] =  { "c": 0.01, "R" : 0.127, "CL_alpha": 6.0, "alpha_0": 0.15, "CD_alpha": 0.02, "CD_i": 0.05} # c          Cord length [m]

    p_["m"] = 0.5      # [kg]
    p_["g"] = 9.81     # [N/kg]
    p_["rho"] = 1.225  # [kg/m^3]

    L = 0.25
    
    I_max = p_["m"] * L**2 # Inertia of a point mass at a distance L
    I_ref = I_max/5   
    #print "q I_max", I_max
    #print "q I_ref", I_ref
    #print "q I_ref/2", I_ref/2
    p_["I"] = diag([I_ref/2,I_ref/2,I_ref]) # [N.m^2]
    

    p_["rotors_p",0] = DMatrix([L,0,0])
    p_["rotors_p",1] = DMatrix([0,L,0])
    p_["rotors_p",2] = DMatrix([-L,0,0])
    p_["rotors_p",3] = DMatrix([0,-L,0])

    for i in range(NR):
        R_ = p_["rotors_model",i,"R"] #  Radius of propeller [m]
        m_ = 0.01 # Mass of a propeller [kg]
        I_max = m_ * R_**2 # Inertia of a point mass
        #print "rotor I_max", I_max
        I_ref = I_max/5 
        #print "rotor I_ref", I_ref
        #print "rotor I_ref/2", I_ref/2
        p_["rotors_I",i] = diag([I_ref/2,I_ref/2,I_ref])


    if debug:
        print p.vecNZcat()
        
    dist_ = dist(0)
        
    # ----------------- Scaling ---------------------
    
    scaling_states   = states(1)
    scaling_controls = controls(1)
    
    scaling_states["r"] = 500
    scaling_controls["CR"] = 0.005
    
    scaling_dist = dist()
    
    scaling_dist["Faer"] = float(p_["m"]*p_["g"]/NR)
    scaling_dist["Caer"] = 0.0026

    # ----------- Frames ------------------
    T_10 = mul(tr(*pos),Tquat(*q))
    T_01 = kin_inv(T_10)
    R_10 = T2R(T_10)
    R_01 = R_10.T
    # -------------------------------------

    dstates = struct_symSX(states)
    
    dp,dv,dq,dw,dr = dstates[...]
    
    res = struct_SX(states) # DAE residual hand side
    # ----------- Dynamics of the body ----
    res["p"] = v - dp
    # Newton, but transform the force F from {1} to {0}
    res["v"] = mul(R_10,F) - m*dv
    # Kinematics of the quaterion.
    res["q"] = mul(quatDynamics(*q),w)-dq
    
    #print mul(quatDynamics(*q),w)
    #Skew =  jacobian(mul(quatDynamics(*q),w),q)
    #print "skew=",Skew
    #print "skew2=",mul(jacobian(quatDynamics(*q),q),w).reshape((4,4))
    
    #d = mul([q.T,Skew,q])
    
    #a = DMatrix([1,2,0.3,0.7])
    #a=a/sqrt(sumAll(a**2))
    #print jacobian(substitute(d,q,a),w)
    
    
    # This is a trick by Sebastien Gros to stabilize the quaternion evolution equation
    if qstab:
      res["q"] += -q*(sumAll(q**2)-1)
    # Agular impulse H_1011
    H = mul(p["I"],w)    # Due to platform
    for i in range(NR):
      H+= mul(p["rotors_I",i], w + vertcat([0,0,p["rotors_spin",i]*r[i]])) # Due to rotor i

    self.H = H
    dH = mul(jacobian(H,w),dw) + mul(jacobian(H,q),dq) + mul(jacobian(H,r),dr) + cross(w,H)
    
    # dH = mul(jacobian(H,vertcat([w,q,r])),vertcat([dw,dq,dr])) + cross(w,H)  #same
    
    
    # More expensive..
    #temp = SXFunction([vertcat([w,q,r])],[H])
    #temp.init()
    
    #d = temp.derivative(1,0)
    #d.init()
    
    #dH = d([vertcat([w,q,r]),vertcat([dw,dq,dr])])[1]+ cross(w,H)
    
    #dotdraw(dH)

    res["w"] = C - dH

    for i in range(NR):
      res["r",i] = CR[i] - rotors_Caer[i][2] - p["rotors_I",i][2,2]*(dr[i]+p["rotors_spin",i]*dw[2]) # Dynamics of rotor i
    
    # ---------------------------------

    # Make a vector of f ?
    #if quatnorm:
    #    f = vertcat(f+[sumAll(q**2)-1])
    #else:
    #    f = vertcat(f)  

    # ------------ Force model ------------

    Fg = mul(R_01,vertcat([0,0,-g*m]))

    F_total = Fg + sum(rotors_Faer)  + dist["Faer"]    # Total force acting on the platform
    C_total = SX([0,0,0]) + dist["Caer"]               # Total torque acting on the platform

    for i in range(NR):
       C_total[:2] -= p["rotors_spin",i]*rotors_Caer[i][:2] # The x and y components propagate
       # There are no components..
       
       # The rotor inertia arising from I*dw[2] is already present in the platform dynamic balance
       #C_total[2] -= p["rotors_spin",i]*CR[i]-p["rotors_I",i][2,2]*dw[2]      # the z compent moves through a serparate system
       C_total[2]-= p["rotors_spin",i]*rotors_Caer[i][2]
       C_total += cross(p["rotors_p",i],rotors_Faer[i]) # Torques due to thrust

    
    res = substitute(res,F,F_total)
    res = substitute(res,C,C_total)
    
    subs_before = []
    subs_after  = []
    
    v_global = mul(R_01,v)
    u_z = SX([0,0,1])
    
    # Now fill in the aerodynamic forces
    for i in range(NR):
        c,R,CL_alpha,alpha_0, CD_alpha, CD_i = p["rotors_model",i,...]
        #Bristeau P-jean, Martin P, Salaun E, Petit N. The role of propeller aerodynamics in the model of a quadrotor UAV. In: Proceedings of the European Control Conference 2009.; 2009:683-688.
        v_local = v_global + (cross(w,p["rotors_p",i])) # Velocity at rotor i
        rotors_Faer_physics =  (rho*c*R**3*r[i]**2*CL_alpha*(alpha_0/3.0-v_local[2]/(2.0*R*r[i]))) * u_z
        subs_before.append(rotors_Faer[i])
        subs_after.append(rotors_Faer_physics)
        rotors_Caer_physics = rho*c*R**4*r[i]**2*(CD_alpha/4.0+CD_i*alpha_0**2*(alpha_0/4.0-2.0*v_local[2]/(3.0*r[i]*R))-CL_alpha*v_local[2]/(r[i]*R)*(alpha_0/3.0-v_local[2]/(2.0*r[i]*R))) * u_z
        subs_before.append(rotors_Caer[i])
        subs_after.append(rotors_Caer_physics )
    

    
    res = substitute(res,veccat(subs_before),veccat(subs_after))
    
    # Make an explicit ode
    rhs = - solve(jacobian(res,dstates),substitute(res,dstates,0))
    
    # --------------------------------------

    self.res_w = res
    self.res = substitute(res,dist,dist_)
    self.res_ = substitute(self.res,p,p_)
    
    resf = SXFunction([dstates, states, controls ],[self.res_])
    resf.setOption("name","resf")
    resf.init()
    self.resf = resf
    
    self.rhs_w = rhs
    
    self.rhs = substitute(rhs,dist,dist_)

    self.rhs_ = substitute(self.rhs,p,p_)
    
    self.H =  substitute(substitute(H,p,p_),dist,dist_)

    t = SX.sym("t")
    # We end up with a DAE that captures the system dynamics
    dae = SXFunction(daeIn(t=t,x=states,p=controls),daeOut(ode=self.rhs_))
    dae.setOption("name","dae")
    dae.init()
    
    self.dae = dae
    
    cdae = SXFunction(controldaeIn(t=t, x=states, u= controls,p=p),daeOut(ode=self.rhs))
    cdae.setOption("name","cdae")
    cdae.init()
    self.cdae = cdae

    self.states  = states
    self.dstates = dstates
    self.p = p
    self.p_ = p_
    self.controls = controls
    self.NR = NR
    self.w = dist
    self.w_ = dist_
    self.t = t
    
    self.states_  = states()
    self.dstates_ = states()
    self.controls_ = controls()
    
    self.scaling_states = scaling_states
    self.scaling_controls = scaling_controls
    self.scaling_dist = scaling_dist
    
    

casadiAvailable = False
casadiTypes = set()
try:
  import casadi as c
  casadiAvailable = True
  casadiTypes = set([type(c.SXElement()),type(c.SX())])
except ImportError:
  pass
  
def TRx(a):
  constr = numpy.matrix
  if casadiAvailable and type(a) in casadiTypes:
    constr = c.blockcat
  return  constr([[1,0,0,0],[0,cos(a),-sin(a),0],[0,sin(a),cos(a),0],[0,0,0,1]])

def TRy(a):
  constr = numpy.matrix
  if casadiAvailable and type(a) in casadiTypes:
    constr = c.blockcat
  return  constr([[cos(a),0,sin(a),0],[0,1,0,0],[-sin(a),0,cos(a),0],[0,0,0,1]])

def TRz(a):
  constr = numpy.matrix
  if casadiAvailable and type(a) in casadiTypes:
    constr = c.blockcat
  return  constr([[cos(a),-sin(a),0,0],[sin(a),cos(a),0,0],[0,0,1,0],[0,0,0,1]])

def tr(x,y,z):
  return  c.blockcat([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]])
  
def scale(a):
  return  numpy.matrix([[a,0,0],[0,a,0],[0,0,a]])
  
def Tscale(a):
  return  R2T(scale(a))
  
def Tquat(q0,q1,q2,q3):
  return R2T(quat(q0,q1,q2,q3))
  
def quat(q0,q1,q2,q3):
  """
  From Jeroen's presentation. q = [e*sin(theta/2); cos(theta/2)]
  """
  constr = c.blockcat
  types =  set([type(q) for q in [q0,q1,q2,q3]])
  #if not(types.isdisjoint(casadiTypes)):
  #  constr = c.blockcat

  rho = constr([[q0],[q1],[q2]])
  rho_skew = skew(rho)
  I_3 = constr([[1.0,0,0],[0,1.0,0],[0,0,1.0]])

  #A = multiply(I_3,(numpy.dot(rho.T,-rho)+q3*q3))+numpy.dot(rho,rho.T)*2.0-q3*rho_skew*2.0
  
  b = q0
  c_ = q1
  d = q2
  a = q3
  
  a2 = a**2
  b2 = b**2
  c2 = c_**2
  d2 = d**2

  am2 = -a2
  bm2 = -b2
  cm2 = -c2
  dm2 = -d2
  
  bb = 2*b
  aa = 2*a
  
  bc2 = bb*c_
  bd2 = bb*d
  ac2 = aa*c_
  ab2 = aa*b
  ad2 = aa*d
  cd2 = 2*c_*d
  
  A = constr([[a2+b2+cm2+dm2,  bc2 - ad2,  bd2  + ac2],[bc2 + ad2, a2+bm2+c2+dm2, cd2 - ab2], [ bd2 -ac2, cd2 + ab2, a2+bm2+cm2+d2]]).T
  
  return A.T

def quatOld(q0,q1,q2,q3):
  """
  From Shabana AA. Dynamics of multibody systems. Cambridge Univ Pr; 2005.
  defined as [ cos(theta/2) e*sin(theta/2) ]
  """
  constr = numpy.matrix
  types =  set([type(q) for q in [q0,q1,q2,q3]])
  #if not(types.isdisjoint(casadiTypes)):
  #  constr = c.blockcat
    
  E  = constr([[-q1, q0, -q3, q2],[-q2, q3, q0, -q1],[-q3,-q2,q1,q0]])
  Eb = constr([[-q1, q0, q3, -q2],[-q2, -q3, q0, q1],[-q3,q2,-q1,q0]])
  
  
  if not(types.isdisjoint(casadiTypes)):
    constr = c.blockcat
    
  return constr(numpy.dot(E,Eb.T))

def fullR(R_0_0,R_1_0,R_2_0, R_0_1, R_1_1, R_2_1, R_0_2, R_1_2, R_2_2):
  constr = numpy.matrix
  types =  set([type(q) for q in [R_0_0,R_1_0,R_2_0, R_0_1, R_1_1, R_2_1, R_0_2, R_1_2, R_2_2]])
  if not(types.isdisjoint(casadiTypes)):
    constr = c.blockcat
  return constr([[R_0_0,  R_0_1,  R_0_2],[R_1_0,  R_1_1,  R_1_2 ],[R_2_0,  R_2_1,  R_2_2 ]])
  
def TfullR(R_0_0,R_1_0,R_2_0, R_0_1, R_1_1, R_2_1, R_0_2, R_1_2, R_2_2):
  return R2T(fullR(R_0_0,R_1_0,R_2_0, R_0_1, R_1_1, R_2_1, R_0_2, R_1_2, R_2_2))
  

def origin() :
  return tr(0,0,0)
  
  
def trp(T):
  return numpy.matrix(T)[:3,3]
  
def kin_inv(T):
  R=numpy.matrix(T2R(T).T)
  constr = numpy.matrix
  if type(T) in casadiTypes:
    constr = c.blockcat
  return constr(vstack((hstack((R,-numpy.dot(R,trp(T)))),numpy.matrix([0,0,0,1]))))


def vectorize(vec):
  """
  Make sure the result is something you can index with single index
  """
  if hasattr(vec,"shape"):
    if vec.shape[0] > 1 and vec.shape[1] > 1:
      raise Exception("vectorize: got real matrix instead of vector like thing: %s" % str(vec))
    if vec.shape[1] > 1:
      vec = vec.T
    if hasattr(vec,"tolist"):
      vec = [ i[0] for i in vec.tolist()]
  return vec

def skew(vec):
  myvec = vectorize(vec)

  x = myvec[0]
  y = myvec[1]
  z = myvec[2]

  constr = numpy.matrix
  types =  set([type(q) for q in [x,y,z]])
  if not(types.isdisjoint(casadiTypes)):
    constr = c.blockcat

  return constr([[0,-z,y],[z,0,-x],[-y,x,0]])
  
def invskew(S):
  return c.SX([S[2,1],S[0,2],S[1,0]])
  
def cross(a,b):
  return c.mul(skew(a),b)

def T2R(T):
  """
   Rotational part of transformation matrix 
   
  """
  return T[0:3,0:3]
  
def R2T(R):
  """
   Pack a rotational matrix in a homogenous form
   
  """
  T  = c.SX([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1.0]])
  T[:3,:3] = R
  return T
  
def T2w():
  """
   skew(w_100) = T2w(T_10)
   
  """
  
def T2W(T,p,dp):
  """
   w_101 = T2W(T_10,p,dp)
   
  """
  R = T2R(T)
  dR = c.reshape(c.mul(c.jacobian(R,p),dp),(3,3))
  return invskew(c.mul(R.T,dR))

def quatDynamics(q0,q1,q2,q3):
  """
   dot(q) = quatDynamics(q)*w_101
   
  """
  constr = numpy.matrix
  if type(q0) in casadiTypes:
    constr = c.blockcat
  B = constr([[q3,-q2,q1],[q2,q3,-q0],[-q1,q0,q3],[-q0,-q1,-q2]])*0.5
  return B

def T2WJ(T,p):
  """
   w_101 = T2WJ(T_10,p).diff(p,t)
   
  """
  R = T2R(T)
  RT = R.T
  
  temp = []
  for i,k in [(2,1),(0,2),(1,0)]:
     #temp.append(c.mul(c.jacobian(R[:,k],p).T,R[:,i]).T)
     temp.append(c.mul(RT[i,:],c.jacobian(R[:,k],p)))

  return c.vertcat(temp)
  
def evalModel(model):
  states   = model.states
  dstates  = model.dstates
  
  controls = model.controls
  p        = model.p
  
  states_   = model.states_
  dstates_  = model.dstates_
  
  controls_ = model.controls_
  p_        = model.p_
  
  rhs      = model.rhs
  res      = model.res
  
  # We end up with a DAE that captures the system dynamics
  dyn = SXFunction([dstates, states, controls ],[res])
  dyn.setOption("name","dyn")
  dyn.init()
  
  NR       = model.NR


  resd = dyn([SX(dstates_), SX(states_), SX(controls_)])[0]

  res  = dyn([SX(dstates_), SX(states_), SX(controls_)])[0]

  res_ = substitute(res,p,p_)

  for v in zip(states.cat,resd,res,res_):
      print v[0]
      for i in v[1:]:
          print "  ", i
          
  return (res,res_)
    
def unittests():
  model    = QuadcopterModel()
  
  states   = model.states
  dstates  = model.dstates
  
  controls = model.controls
  p        = model.p
  
  states_   = model.states_
  dstates_  = model.dstates_
  
  controls_ = model.controls_
  p_        = model.p_
  
  NR       = model.NR


  #  ----------------- do quick checks on the model -----

  print "zero input -> gravity"

  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [0,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i]
      if i == states.f["v"][2]:
          print res_[i].toScalar()
          assert(res_[i].toScalar().getValue()==-p_["g"]*p_["m"])
      else:
          assert(res_[i].toScalar().getValue()==0)

  print "platform rotating around +z"

  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [0,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,1]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i]
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()==-p_["g"]*p_["m"])
      elif i == states.f["q"][2]:
          assert(res[i].toScalar().isEqual(0.5)) # The quaternion is changing
      else:
          assert(res_[i].toScalar().getValue()==0)

  print "platform accelerates around +z"

  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [0,0,0,0]
  dstates_["r"] = [-1,1,-1,1]  # The rotors keep steady in the inertial frame

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,1]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i]
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()==-p_["g"]*p_["m"])
      elif i in list(states.f["r"]):
          assert(res_[i].toScalar().getValue()==0) # since the rotors are mounted in vertical bearing
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)
      else:
          assert(res_[i].toScalar().getValue()==0)
          
  print "One rotor works"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print states.cat[i],res[i], res_[i].toScalar()
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()>-p_["g"])  # gravity pulls down
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(res_[i].toScalar().getValue()<0)  # rotor slows down
      else:
          assert(res_[i].toScalar().isEqual(0))
  thrust = res_[states.f["v"][2]].toScalar().getValue()+p_["g"]
  angular = res_[states.f["w"][1]].toScalar().getValue()
  slowdown = res_[states.f["r"][0]].toScalar().getValue()
  

  print "One rotor works while platform moves upwards"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0.01]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print states.cat[i], res[i], res_[i].toScalar()
      if i == states.f["p"][2]:
          assert(res_[i].toScalar().getValue()==states_["v"][2])  # dx = v
      elif i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()+p_["g"]<thrust)  # Translation speed makes for decreased propeller lift
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(abs(res_[i].toScalar().getValue())<abs(slowdown))  # rotor slows down less than without the upward movement, because lift is smaller
      else:
          assert(res_[i].toScalar().isEqual(0))
  print "One rotor works while platform moves sideways"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [1,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i], res_[i].toScalar(),abs(slowdown)
      if i == states.f["p"][0]:
          assert(res_[i].toScalar().getValue()==states_["v"][0])  # dx = v
      elif i == states.f["v"][2]:
          print "debug:" , res_[i].toScalar().getValue()+p_["g"], thrust
          assert(res_[i].toScalar().getValue()+p_["g"]<=thrust)  # Translation speed makes for decreased propeller lift
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(abs(res_[i].toScalar().getValue())==abs(slowdown)) # rotor slows down less, because lift is smaller
      else:
          assert(res_[i].toScalar().isEqual(0))
  
          
  print "One rotor works while platform rotates around positive local x-axis"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0.05,0,0,0]
  states_["w"]  = [0.1,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i], res_[i].toScalar(),abs(thrust)
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()+p_["g"]==thrust)  # No change
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y   + some gyro effects
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(res_[i].toScalar().getValue()==slowdown)  # No change
      else:
          assert(res_[i].toScalar().isEqual(0))

             
  print "One rotor works while platform rotates around axis through rotor center and parallel to y-axis (positive)"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0.25*0.1] #L*omega
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0.05,0,0]
  states_["w"]  = [0,0.1,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i], res_[i].toScalar()
      if i == states.f["p"][2]:
          assert(res_[i].toScalar().getValue()==states_["v"][2])
      elif i == states.f["w"][0]:
          assert(res_[i].toScalar().getValue()<0) # Gyroscopic effect: accelerarion around -x
      elif i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()+p_["g"]==thrust)  # No change
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)        # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(res_[i].toScalar().getValue()==slowdown) # rotor slows down less, because lift is smaller
      else:
          assert(res_[i].toScalar().isEqual(0))
          
  
  print "One rotor works while platform is tilted 45 degrees along -y"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,-sin(pi/8),0,cos(pi/8)]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print states.cat[i], res[i], res_[i].toScalar()
      if i == states.f["v"][0]:
          assert(res_[i].toScalar().getValue()<0)  # Thrust in -x direction
      elif i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()+p_["g"]<thrust)  # Inclination makes for smaller upward force
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(res_[i].toScalar().getValue()<0)  # rotor slows down
      else:
          assert(res_[i].toScalar().isEqual(0))

  print "One rotor works while platform is tilted 45 degrees along +y"
  # @TODO:  make this unittest  
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,sin(pi/8),0,cos(pi/8)]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      if i == states.f["v"][0]:
          assert(res_[i].toScalar().getValue()>0)  # Thrust in -x direction
      elif i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()+p_["g"]<thrust)  # Inclination makes for smaller upward force
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(res_[i].toScalar().getValue()<0)  # rotor slows down
      else:
          assert(res_[i].toScalar().isEqual(0))



  print "platform rotating around +x"

  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [0,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0.5,0,0,0]
  states_["w"]  = [1,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i]
      if i == states.f["v"][2]:
          assert(res[i].toScalar().isEqual((-p["g"]*p["m"]).at(0),2))
      else:
          assert(res[i].toScalar().isEqual(0))
          

  print "platform rotating around -y with one rotor spinning"

  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,-0.05,0,0] # The quaternion is changing
  states_["w"]  = [0,-0.1,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i], res_[i].toScalar()
      if i == states.f["v"][2]:
         assert(res_[i].toScalar()+p_["g"]*p_["m"]<thrust)  # Decreased lift due to local velocity at the rotor
      elif i == states.f["q"][1]:
          assert(res_[i].toScalar().getValue()==0)
      elif i == states.f["w"][0]:
          assert(res_[i].toScalar().getValue()>0) # Gyroscopic effect: accelerarion around +x
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0) # Acceleration around - y due to rotor thrust
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          assert(res_[i].toScalar().getValue()<0)  # rotor slows down
      else:
          assert(res[i].toScalar().isEqual(0))

  print "platform rotating around -y with all rotors spinning"

  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,100,100,100]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,-0.005,0,0] # The quaternion is changing
  states_["w"]  = [0,-0.01,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      print res[i], res_[i].toScalar().getValue(), angular
      if i == states.f["v"][2]:
          assert(res_[i].toScalar()>-p_["g"]*p_["m"])
      elif i == states.f["w"][0]:
          assert(res_[i].toScalar().getValue()==0) # Gyroscopic effects cancel out 
      #elif i == states.f["w"][1]:
      #    assert(res_[i].toScalar().getValue()==0) #  Thrusts cancel out   # improved aero: thrusts do not cancel exactly
      elif i == states.f["w"][1]:
          assert(abs(res_[i].toScalar().getValue())<=0.2*abs(angular)) #   improved aero: thrusts do not cancel exactly, let's say they must cancel up to 20%
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z: torques do not cancel out completely
      elif i == states.f["q"][1]:
          assert(res_[i].toScalar().getValue()==0)
      elif i in states.f["r"]:
          assert(res_[i].toScalar().getValue()<0)  # rotor slows down
      else:
          assert(res[i].toScalar().isEqual(0))   
          
  print "One rotor with positive spin accelerates"
  controls_["CR"]= [1,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,0,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()>-p_["g"])  # gravity pulls down
      elif i == states.f["w"][1]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -y
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()<0)  # platform rotates over -z
      elif i == states.f["r"][0]:
          print p_["rotors_I",0,2,2]
          assert(res_[i].toScalar().getValue()<controls_["CR"][0]/p_["rotors_I",0,2,2])
                                                   # Rotor accelerates, but less than in vacuum
      else:
          assert(res_[i].toScalar().isEqual(0))
 
  print "One rotor with negative spin accelerates"
  controls_["CR"]= [0,1,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [0,100,0,0]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()>-p_["g"])  # gravity pulls down
      elif i == states.f["w"][0]:
          assert(res_[i].toScalar().getValue()>0)  # platform rotates over +x
      elif i == states.f["w"][2]:
          assert(res_[i].toScalar().getValue()>0)  # platform rotates over +z
      elif i == states.f["r"][1]:
          assert(res_[i].toScalar().getValue()<controls_["CR"][1]/p_["rotors_I",1,2,2])
                                                   # Rotor accelerates, but less than in vacuum
      else:
          assert(res_[i].toScalar().isEqual(0))   

  print "All rotors rotate constant"
  controls_["CR"]= [0,0,0,0]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,100,100,100]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()>-p_["g"])  # gravity pulls down
      elif i in states.f["r"]:
          assert(res_[i].toScalar().getValue()<0)      # all rotors slow down
      else:
          assert(res_[i].toScalar().isEqual(0))   

  print "All rotors accelerate"
  controls_["CR"]= [1,1,1,1]
  states_["p"]  = [0,0,0]
  dstates_["p"] = [0,0,0]
  states_["v"]  = [0,0,0]
  dstates_["v"] = [0,0,0]
  states_["r"]  = [100,100,100,100]
  dstates_["r"] = [0,0,0,0]

  states_["q"]  = [0,0,0,1]
  dstates_["q"] = [0,0,0,0]
  states_["w"]  = [0,0,0]
  dstates_["w"] = [0,0,0]

  [res, res_] = evalModel(model)

  for i in range(res.size()):
      if i == states.f["v"][2]:
          assert(res_[i].toScalar().getValue()>-p_["g"])  # gravity pulls down
      elif i in states.f["r"]:
          assert(res_[i].toScalar().getValue()<controls_["CR",int(i-states.f["r"][0])]/p_["rotors_I",int(i-states.f["r"][0]),2,2])
                                                   # Rotor accelerates, but less than in vacuum
      else:
          assert(res_[i].toScalar().isEqual(0))   

if __name__ == '__main__':
    unittests()
