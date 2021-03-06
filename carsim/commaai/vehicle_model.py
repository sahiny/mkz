#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
# import pdb
# from selfdrive.car.honda.interface import CarInterface

## dynamic bycicle model from "The Science of Vehicle Dynamics (2014), M. Guiggiani"##

# Xdot = A*X + B*U
# where X = [v, r], with v and r lateral speed and rotational speed, respectively
# and U is the steering angle (controller input)
# 
# A depends on longitudinal speed, u, and vehicle parameters CP

class CP(object):
  def __init__(self):

    # kg of standard extra cargo to count for drive, gas, etc...
    std_cargo = 136

    # FIXME: hardcoding honda civic 2016 touring wight so it can be used to 
    # scale unknown params for other cars
    m_civic = 2923./2.205 + std_cargo
    l_civic = 2.70
    aF_civic = l_civic * 0.4
    aR_civic = l_civic - aF_civic
    j_civic = 2500
    cF_civic = 85400
    cR_civic = 90000

    self.m = m_civic
    self.l = l_civic
    self.aF = aF_civic
    self.sR = 13.0
    self.steerKp, self.steerKi = 0.8, 0.24

    self.aR = self.l - self.aF
    # TODO: get actual value, for now starting with reasonable value for 
    # civic and scaling by mass and wheelbase
    self.j = j_civic * self.m * self.l**2 / (m_civic * l_civic**2)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position... all cars will have approximately similar dyn behaviors
    self.cF = cF_civic * self.m / m_civic * (self.aR / self.l) / (aR_civic / l_civic)
    self.cR = cR_civic * self.m / m_civic * (self.aF / self.l) / (aF_civic / l_civic)

    # no rear steering, at least on the listed cars above
    self.chi = 0.


def create_dyn_state_matrices(u, CP):
  A = np.zeros((2,2))
  B = np.zeros((2,1))
  A[0,0] = - (CP.cF + CP.cR)/(CP.m*u)
  A[0,1] = - (CP.cF*CP.aF - CP.cR*CP.aR) / (CP.m*u) - u
  A[1,0] = - (CP.cF*CP.aF - CP.cR*CP.aR) / (CP.j*u)
  A[1,1] = - (CP.cF*CP.aF**2 + CP.cR*CP.aR**2) / (CP.j*u)
  B[0,0] = (CP.cF + CP.chi*CP.cR) / CP.m / CP.sR
  B[1,0] = (CP.cF*CP.aF - CP.chi*CP.cR*CP.aR) / CP.j / CP.sR
  return A, B


def kin_ss_sol(sa, u, CP):
  # kinematic solution, useful when speed ~ 0
  K = np.zeros((2,1))
  K[0,0] = CP.aR / CP.sR / CP.l * u
  K[1,0] = 1. / CP.sR / CP.l * u
  return K * sa


def dyn_ss_sol(sa, u, CP):
  # Dynamic solution, useful when speed > 0
  A, B = create_dyn_state_matrices(u, CP)
  return - np.matmul(inv(A), B) * sa

def calc_slip_factor(CP):
  # the slip factor is a measure of how the curvature changes with speed
  # it's positive for Oversteering vehicle, negative (usual case) otherwise
  return CP.m * (CP.cF * CP.aF - CP.cR * CP.aR) / (CP.l**2 * CP.cF * CP.cR)

class VehicleModel(object):
  def __init__(self, init_state=np.asarray([[0.],[0.]])):
    self.dt = 0.1
    lookahead = 2.    # s
    self.steps = int(lookahead / self.dt)
    self.update_state(init_state)
    self.state_pred = np.zeros((self.steps, self.state.shape[0]))
    self.CP = CP()

  def update_state(self, state):
    self.state = state

  def steady_state_sol(self, sa, u):
    # if the speed is too small we can't use the dynamic model 
    # (tire slip is undefined), we then use the kinematic model
    if u > 0.1:
      return dyn_ss_sol(sa, u, self.CP)
    else:
      return kin_ss_sol(sa, u, self.CP)

  def calc_curvature(self, sa, u):
    # this formula can be derived from state equations in steady state conditions
    sf = calc_slip_factor(self.CP)
    return (1. - self.CP.chi)/(1. - sf * u**2) * sa / self.CP.sR / self.CP.l

  def get_steer_from_curvature(self, curv, u):
    # this function is the exact inverse of calc_curvature, returning steer angle given curvature
    sf = calc_slip_factor(self.CP)
    # print(sf)
    a = self.CP.l * self.CP.sR * (1. - sf * u**2) / ((1. - self.CP.chi)* u) * curv
    # print(a)
    return a

  def state_prediction(self, sa, u):
    # U is the matrix of the controls
    # u is the long speed
    A, B = create_dyn_state_matrices(u, self.CP)
    return np.matmul((A * self.dt + np.identity(2)), self.state) + B * sa * self.dt


# if __name__ == '__main__':
#   # load car params
#   CP = CarInterface.get_params("HONDA CIVIC 2016 TOURING", {})
#   print CP
#   VM = VehicleModel(CP)
#   print VM.steady_state_sol(.1, 0.15)
  
