from __future__ import division
import time
import logging

import constants
from lib import util
from lib import parameter_store
from lib.parameter_store import Priority

ps = parameter_store.ParameterStore(__name__, prefix="controller/temp")
logger = logging.getLogger(__name__)


class ProcessTempController(object):
  """A PID controller for the process temperature.
  This controller uses PID and a deadband. The derivative term is calculated
  using a running average of the derivative. The duration of the running
  average can be specified. The integral is calculated using an accumulator
  with a specified duration.
  """

  def __init__(self):
    """Set internal state.
    New bays with brushless pumps have no set rate.  They are either on or off.
    """
    # Set topic defaults from config
    config = constants.TEMP_CONTROLLER
    for key, value in config["defaults"].iteritems():
      ps.priority_set(key, value, Priority.DEFAULT)
    self.control_period = config["control_period"]

    self.pump_on = 0
    self.heating_mode = None
    self.cooling_mode = None
    self.temp_control = 0
    self.latest_temps = []
    self.latest_timestamps = []
    self.p_term = 0
    self.i_term = 0
    self.d_term = 0
    # For bath temp mode, we need the following states
    self.bath_temp_setpoint = 22
	self.bath_temp_base = ps.get("process_temp_setpoint", block=True) + 1.415 #the 1.415 replaces the integral term for now

  def update(self):
    """Update internal state."""
    # Get required external data
    enabled = ps.get("~/enabled", block=True)
    bath_temp = ps.get("bath_temp/running_average/30", block=True)
    temp_setpoint = ps.get("process_temp_setpoint", block=True)
    latest_temp = ps.get("process_temp", block=True)

    # Get config
    deadband = ps.get("~/deadband")
    kp = ps.get("~/kp")
    ki = ps.get("~/ki")
    kd = ps.get("~/kd")
    integral_duration = ps.get("~/integral_duration")
    derivative_duration = ps.get("~/derivative_duration")
    bath_deadband = ps.get("~/bath_deadband")
    mode = ps.get("~/mode")
    min_bath_temp_setpoint = ps.get("~/min_bath_temp_setpoint")
    max_bath_temp_setpoint = ps.get("~/max_bath_temp_setpoint")

    # Check that the system is enabled.
    if not enabled:
      self.shutdown()
      #self.heating_mode = False
      #self.cooling_mode = False
      #self.pump_on = 0
      return

    # When enabled, turn on bath pump.
    ps.priority_set("bath_pump_setpoint", 1, Priority.UPDATE)

    # Append latest temp
    self.latest_temps.append(latest_temp)
    self.latest_timestamps.append(time.time())

    # Truncate the full array of temp data just to prevent the array from
    # growing excessively.
    max_duration = max([derivative_duration, integral_duration])
    max_array_length = int(max_duration / self.control_period)
    self.latest_temps = self.latest_temps[-max_array_length:]
    self.latest_timestamps = self.latest_timestamps[-max_array_length:]

    # Compute the error integral.
    integral_array_length = int(integral_duration / self.control_period)
    integral = sum([
      temp_setpoint - t for t in self.latest_temps[-integral_array_length:]
    ]) * (self.latest_timestamps[-1] - self.latest_timestamps[-2]) #need to multiply by time interval

    # Compute the derivative of the temp by looking at a window of recent data.
    derivative_array_length = int(derivative_duration / self.control_period)
    derivative, _ = util.linear_regression(
      self.latest_timestamps[-derivative_array_length:],
      self.latest_temps[-derivative_array_length:],
    )

    # Calculate the temp control.
    # Track each internal PID term for debugging.
    self.p_term = kp * (temp_setpoint - latest_temp)
    self.i_term = ki * integral
    self.d_term = kd * derivative
    self.temp_control = sum((self.p_term, self.i_term, -1 * self.d_term))

    if mode == "process_temp":
      if abs(latest_temp - temp_setpoint) <= deadband:
        # If we're in the deadband, heater/cooler should both be off.
        self.apply_thermal_control(heating=False, cooling=False)
      elif latest_temp > temp_setpoint:
        self.apply_thermal_control(heating=False, cooling=True)
      else:
        self.apply_thermal_control(heating=True, cooling=False)

    elif mode == "bath_temp":
      # If we're in the process temp deadband, we should not update the
      # bath temp setpoint.
      if abs(latest_temp - temp_setpoint) <= deadband:
        return
      # Add the temp control signal to the bath temp setpoint.
      new_bath_temp_setpoint = self.bath_temp_base + self.temp_control
      # Apply the min/max bath temp setpoints
      self.bath_temp_setpoint = max(
        min_bath_temp_setpoint,
        min(new_bath_temp_setpoint, max_bath_temp_setpoint),
      )
      # If we're within the deadband of the water bath temp, do nothing
      # If not, heat or cool accordingly
      if (abs(self.bath_temp_setpoint - bath_temp) <= bath_deadband):
        self.apply_thermal_control(heating=False, cooling=False)
      elif ((self.bath_temp_setpoint - bath_temp) > bath_deadband):
        self.apply_thermal_control(heating=True, cooling=False)
      else:
        self.apply_thermal_control(heating=False, cooling=True)

      # Publish debugging topics
      ps.set("bath_temp_setpoint", self.bath_temp_setpoint)
      if constants.BAY_CONFIG['verbose_logging']:
        ps.set("~/p_term", self.p_term)
        ps.set("~/i_term", self.i_term)
        ps.set("~/d_term", self.d_term)

  def apply_thermal_control(self, heating, cooling):
    assert not (heating and cooling)
    block_temp = ps.get("block_temp", block=True)
    max_block_temp = ps.get("~/max_block_temp")
    if heating and block_temp > max_block_temp:
      logger.warning(
        'block temp exceeds max of {}C, turning off heating block'.
        format(max_block_temp)
      )
      ps.priority_set("heating_block_setpoint", 0, Priority.UPDATE)
    else:
      ps.priority_set("heating_block_setpoint", int(heating), Priority.UPDATE)
    ps.priority_set("cooling_peltier_setpoint", int(cooling), Priority.UPDATE)

  def shutdown(self):
    ps.priority_set("heating_block_setpoint", 0, Priority.UPDATE)
    ps.priority_set("cooling_peltier_setpoint", 0, Priority.UPDATE)
    ps.priority_set("bath_pump_setpoint", 0, Priority.UPDATE)