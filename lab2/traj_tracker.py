import time
import math
import random
from traj_planner_utils import *

TIME_STEP_SIZE = 0.05 #s
LOOK_AHEAD_TIME = 1.0 #s
MIN_DIST_TO_POINT = 0.1 #m
MIN_ANG_TO_POINT = 0.10 #rad


def wrap_to_pi(angle):
  """Wrap angle data in radians to [-pi, pi]

  Parameters:
  angle (float)   -- unwrapped angle

  Returns:
  angle (float)   -- wrapped angle
  """
  while angle >= math.pi:
    angle -= 2 * math.pi

  while angle <= -math.pi:
    angle += 2 * math.pi
  return angle

class TrajectoryTracker():
  """ A class to hold the functionality for tracking trajectories.
      Arguments:
        traj (list of lists): A list of traj points Time, X, Y, Theta (s, m, m, rad).
  """
  current_point_to_track = 0
  traj_tracked = False
  traj = []

  def __init__(self, traj):
    self.current_point_to_track = 0
    self.traj = traj
    self.traj_tracked = False
      
  def get_traj_point_to_track(self, current_state):
    """ Determine which point of the traj should be tracked at the current time.
        Arguments:
          current_state (list of floats): The current Time, X, Y, Theta (s, m, m, rad).
        Returns:
          desired_state (list of floats: The desired state to track - Time, X, Y, Theta (s, m, m, rad).
    """

    # if self.traj_tracked
    # self.current_point_to_track = 0

    return self.traj[self.current_point_to_track]
  
  def print_traj(self):
    """ Print the trajectory points.
    """
    print("Traj:")
    for i in range(len(self.traj)):
        print(i,self.traj[i])
          
  def is_traj_tracked(self, controller):
    """ Return true if the traj is tracked.
        Returns:
          traj_tracked (boolean): True if traj has been tracked.
    """
    # MODULAR: for point tracking
    # print(controller.poiPointTrackernt_tracked, self.current_point_to_track, len(self.traj))
    if controller.point_tracked and self.current_point_to_track == len(self.traj) - 1:
      self.traj_tracked = True
    elif controller.point_tracked:
      self.current_point_to_track += 1
      controller.point_tracked = False

    return self.traj_tracked
    
class PointTracker():
  """ A class to determine actions (motor control signals) for driving a robot to a position.
  """
  def __init__(self):
    self.point_tracked = False

  def get_dummy_action(self, x_des, x):
    """ Return a dummy action for now
    """
    action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    return action

  def point_tracking_control(self, desired_state, current_state):
    """ Return the motor control signals as actions to drive a robot to a desired configuration
        Arguments:
          desired_state (list of floats): The desired Time, X, Y, Theta (s, m, m, rad).
          current_state (lis5t of floats): The current Time, X, Y, Theta (s, m, m, rad).
    """
    L = 0.2 # distance between wheels / 2

    # k_p = 10  # > 0, 5, 50
    # k_a = 11  # > k_p, 15, 60
    # k_b = -5  # < 0

    k_p = 30  # > 0, 5, 50
    k_a = 40  # > k_p, 15, 60
    k_b = -10 # < 0


    # various variables...
    del_x = desired_state[1] - current_state[1]
    del_y = desired_state[2] - current_state[2]
    rho = math.sqrt(del_x ** 2 + del_y ** 2)
    alpha = wrap_to_pi(-current_state[3] + math.atan2(del_y, del_x))
    beta = wrap_to_pi(-current_state[3] - alpha)

    # check for whether it makes sense to go forward or backward
    if (abs(alpha) < (math.pi / 2)):
      beta = wrap_to_pi(beta + desired_state[3])
      vel = k_p * rho
      omega = k_a * alpha + k_b * beta

    else:
      alpha = wrap_to_pi(-current_state[3] + math.atan2(-del_y, -del_x))
      beta = wrap_to_pi(-current_state[3] - alpha - desired_state[3])
      vel = - k_p * rho
      omega = k_a * alpha + k_b * beta

    # calculate left/right omegas
    omega_r = 1 / 2 * (vel / L + omega)
    omega_l = 1 / 2 * (omega - vel / L)

    # calculate left/right velocities
    vel_r = omega_r * 2 * L
    vel_l = -omega_l * 2 * L

    # set the control vector
    action = [vel_r, vel_l, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # set self.point_tracked=True  when the  robots is within tolerance of the point being tracked
    # use the variables MIN_DIST_TO_POINT and MIN_ANG_TO_POINT
    if (abs(rho) < MIN_DIST_TO_POINT and abs(beta) < MIN_ANG_TO_POINT):
      self.point_tracked = True

    return action
