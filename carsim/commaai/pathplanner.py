import math
import numpy as np

from numpy import interp


def compute_path_pinv():
  deg = 3
  x = np.arange(50.0)
  X = np.vstack(tuple(x**n for n in range(deg, -1, -1))).T
  pinv = np.linalg.pinv(X)
  return pinv

def model_polyfit(points, path_pinv):
  return np.dot(path_pinv, map(float, points))

# lane width http://safety.fhwa.dot.gov/geometric/pubs/mitigationstrategies/chapter3/3_lanewidth.cfm
_LANE_WIDTH_V = [3., 3.8]

# break points of speed
_LANE_WIDTH_BP = [0., 31.]

def calc_desired_path(l_poly, r_poly, p_poly, l_prob, r_prob, p_prob, speed):
  #*** this function computes the poly for the center of the lane, averaging left and right polys
  lane_width = interp(speed, _LANE_WIDTH_BP, _LANE_WIDTH_V)

  # lanes in US are ~3.6m wide
  half_lane_poly = np.array([0., 0., 0., lane_width / 2.])
  c_poly = ((l_poly - half_lane_poly) * l_prob +
            (r_poly + half_lane_poly) * r_prob) / (l_prob + r_prob)
  c_prob = math.sqrt((l_prob**2 + r_prob**2) / 2.)

  p_weight = 1. # predicted path weight relatively to the center of the lane
  d_poly =  list((c_poly*c_prob + p_poly*p_prob*p_weight ) / (c_prob + p_prob*p_weight))
  return d_poly, c_poly, c_prob

class PathPlanner(object):
  def __init__(self):
    self.dead = True
    self.d_poly = [0., 0., 0., 0.]
    self.c_poly = [0., 0., 0., 0.]
    self.c_prob = 0.
    # self.lead_dist, self.lead_prob, self.lead_var = 0, 0, 1
    self._path_pinv = compute_path_pinv()

  def update(self, v_ego, md):
    p_poly = model_polyfit(md.path, self._path_pinv)       # predicted path
    # print(p_poly)
    l_poly = model_polyfit(md.leftLane, self._path_pinv)   # left line
    # print(l_poly)
    r_poly = model_polyfit(md.rightLane, self._path_pinv)  # right line
    # print(r_poly)

    p_prob = 1.                       # model does not tell this probability yet, so set to 1 for now
    l_prob = md.leftProb   # left line prob
    r_prob = md.rightProb  # right line prob

    # compute target path
    self.d_poly, self.c_poly, self.c_prob = calc_desired_path(l_poly, r_poly, p_poly, l_prob, r_prob, p_prob, v_ego)
    # print(self.d_poly)
    # print(self.c_poly)
    # print(self.c_prob)

