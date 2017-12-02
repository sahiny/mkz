import sys
import numpy as np

import longcontrol as LoC
import adaptivecruise as AC
import pathplanner as PP
import latcontrol as LaC
import vehicle_model as VM
from drive_helpers import learn_angle_offset

class Car(object):

  def __init__(self, vLead, dRel, vRel, aLeadK):
    self.vLead = vLead
    self.dRel = dRel
    self.vRel = vRel
    self.aLeadK = aLeadK

class FixedRoadModel(object):
  def __init__(self, v_init):
    self.u0 = v_init
    self.m = (2923./2.205) + 136.
    self.Iz = 2500
    self.a = 1.08
    self.b = 1.62
    self.l = 2.7
    self.g = 9.82
    self.alpha_ass = 0.32
    self.alpha_road = 0.3
    self.Caf = 85400.
    self.Car = 90000.
    self.F_yfmax = 1500.
    self.rmin = self.u0**2/(0.9*self.alpha_road*self.g)
    # total_t1 = np.zeros(1000)
    # total_t2 = (np.linspace(10,10.99,100) - 10)*self.u0/self.rmin
    # total_t3 = (self.u0/self.rmin)*np.ones(900)
    # total_t4 = ((np.linspace(20,21.99,200)-20)/2) * self.u0/(-self.rmin) + (1-((np.linspace(20,21.99,200)-20)/2)) * self.u0/(self.rmin)
    # total_t5 = (self.u0/self.rmin)*np.ones(800)
    # self.total_t = np.hstack([total_t1, total_t2, total_t3, total_t4, total_t5])
    self.total_t = np.zeros(3000)
  def query_rd(self, t):
    # total: 30 seconds * 100 samples/second = 3000 samples
    return self.total_t[int(100*t):int(100*t)+50]


class RoadModel(object):

  def __init__(self):
    # path, left_lane, and right_lane all have to be a 50x1 matrix
    self.path = np.zeros(50)
    self.leftLane = np.zeros(50)
    self.rightLane = np.zeros(50)
    self.leftProb = 0.5
    self.rightProb = 0.5

  def update(self, path, left_lane, right_lane, left_prob, right_prob):
    # temp_path = np.array(path._data.tolist())
    # self.path = temp_path.reshape(path.size).transpose()
    # temp_leftlane = np.array(left_lane._data.tolist())
    # self.leftLane = temp_leftlane.reshape(left_lane.size).transpose()
    # temp_rightlane = np.array(right_lane._data.tolist())
    # self.rightLane = temp_rightlane.reshape(right_lane.size).transpose()
    # self.leftProb = left_prob
    # self.rightProb = right_prob
    self.path = path
    self.left_lane = left_lane
    self.right_lane = right_lane
    self.left_prob = left_prob
    self.right_prob = right_prob


class Controller(object):

  def __init__(self, v_ego_des):
    self.acc = AC.AdaptiveCruise()
    self.pid_c = LoC.LongControl()
    self.v_ego_des_kph = v_ego_des * 3.6;

  def update(self, enabled, vEgo, vLead, dRel, aLeadK):
    vRel = vLead - vEgo
    l1 = Car(vLead, dRel, vRel, aLeadK)
    self.acc.update(vEgo, vEgo, l1)
    gas, brake = self.pid_c.update(enabled, vEgo, self.v_ego_des_kph,
                                   self.acc.v_target_lead, self.acc.a_target, 
                                   self.acc.jerk_factor, True)
    return gas, brake

class RunLatController(object):

  def __init__(self, v_init):
    self.PP = PP.PathPlanner()
    self.LaC = LaC.LatControl()
    self.VM = VM.VehicleModel()
    # self.model = FixedRoadModel(v_init)
    self.offset = 0.
    self._path_pinv = self.compute_path_pinv()
  
  def compute_path_pinv(self):
    deg = 3
    x = np.arange(50.0)
    X = np.vstack(tuple(x**n for n in range(deg, -1, -1))).T
    pinv = np.linalg.pinv(X)
    return pinv

  def model_polyfit(self, points, path_pinv):
    return np.dot(path_pinv, map(float, points))

  def update(self, v_ego, theta_ego, observed_path):
    # model: 
    # left_lane = np.sin(np.arange(50) + 1.8)
    # path = np.sin(np.arange(50))
    # right_lane = np.sin(np.arange(50) - 1.8)
    # left_prob = np.random.uniform()
    # right_prob = 1 - left_prob
    # left_lane = 0.9*np.ones(50)
    # path = np.zeros(50)
    # right_lane = -0.9*np.ones(50)
    # left_prob = 0.5
    # right_prob = 0.5

    # self.model.update(path, left_lane, right_lane, left_prob, right_prob)
    # self.offset = learn_angle_offset(True, v_ego, self.offset, self.PP.c_poly, \
    #   self.PP.c_prob, self.LaC.y_des, False)
    # self.PP.update(v_ego, self.model)
    observed_path = np.array(observed_path)
    observed_path.reshape((50,1))
    # print(observed_path)
    d_poly = self.model_polyfit(observed_path, self._path_pinv)

    steer, sat_flag, d_lookahead, y_des, angle_steers_des = self.LaC.update(True, v_ego, theta_ego, False, \
      d_poly, 0., self.VM)
    return steer, float(sat_flag), d_lookahead, y_des, angle_steers_des, d_poly[0], d_poly[1], d_poly[2], d_poly[3]
    # return steer