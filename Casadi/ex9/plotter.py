import matplotlib
matplotlib.interactive(True)

from pylab import *

ion()

print "backend", get_backend()

class Plotter:
  def __init__(self,t_local):
    self.fig = figure()
    title("NMPC running")
    self.firstrun = True
    self.t_local = t_local
    self.x1_history = []
    self.x2_history = []
    self.u_history = []
    self.t_global = []
    self.clipping = False
    #self.toggleClipping = []
    
  def toggleClipping(self):
    self.clipping = not self.clipping

  def clear(self):
    self.t_global = []
    self.x1_history = []
    self.x2_history = []
    self.u_history = []
       
  def show(self,t,x_current,sol):
    self.x1_history.append(x_current[0])
    self.x2_history.append(x_current[1])
    self.u_history.append(sol["U",0])
    self.t_global.append(t)
    if self.firstrun:
      subplot(1,2,2)
      gca().set_yticklabels([])
      ##################################################################
      [self.p_control] = step(self.t_local,sol["U",:] + [ sol["U",-1] ],'-.',where="pre",label="u")
      ##################################################################
      [self.p_states1] = plot(self.t_local,sol["X",:,0],'o--',label="x1")
      [self.p_states2] = plot(self.t_local,sol["X",:,1],'o-',label="x2")
      title("The future")
      legend()
      grid(True)
      xlabel('Prediction horizon [s]')
      
      subplot(1,2,1)
      [self.p_control_history] = step(self.t_global,self.u_history,'-.',where="pre",label="u")
      [self.p_states1_history] = plot(array(self.t_global)-10,self.x1_history,'o--',label="x1")
      [self.p_states2_history] = plot(array(self.t_global)-10,self.x2_history,'o-',label="x2")
      ##################################################################
      xlim([-10,0])
      ##################################################################
      title("The past")
      grid(True)
      xlabel('Past trajectories [s]')
      subplots_adjust(wspace=0, hspace=0)
      self.firstrun = False
      pause(0.1)
    else:
      self.p_control.set_ydata(sol["U",:] + [ sol["U",-1] ])
      self.p_states1.set_ydata(sol["X",:,0])
      self.p_states2.set_ydata(sol["X",:,1])
      ##################################################################
      self.p_control.set_xdata(array(self.t_local)+self.t_global[-1])
      self.p_states1.set_xdata(array(self.t_local)+self.t_global[-1])
      self.p_states2.set_xdata(array(self.t_local)+self.t_global[-1])
      subplot(1,2,2).set_xlim([self.t_global[-1],self.t_global[-1]+self.t_local[-1]])
      
      self.p_control_history.set_ydata(self.u_history)
      self.p_states1_history.set_ydata(self.x1_history)
      self.p_states2_history.set_ydata(self.x2_history)
      self.p_control_history.set_xdata(self.t_global)
      self.p_states1_history.set_xdata(self.t_global)
      self.p_states2_history.set_xdata(self.t_global)
      subplot(1,2,1).set_xlim([self.t_global[-1]-10,self.t_global[-1]])
      ##################################################################
      
#      subplot(1,2,1).set_xlim([self.t_global[0],self.t_global[-1]])

      draw()
      pause(0.1)

