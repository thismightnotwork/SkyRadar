
#
# This file is part of the ATC-Pie project,
# an air traffic control simulation program.
# 
# Copyright (C) 2015  Michael Filhol <mickybadia@gmail.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
#

from base.util import some

from session.config import settings
from session.env import env


# ---------- Constants ----------

default_XPDR_mode = 'C'

# -------------------------------

class Status:
	def __init__(self, airborne):
		self.snapped_GND = not airborne # fixed wing on GND or helo at air taxi height
		self.ready_for_DEP = False
		self.snapped_lined_up = False # lined up for DEP (until lift-off), or on final APP leg (until controlled speed on ground or gone around)
		self.rolling_TKOF_LDG = False # TKOF or LDG roll; most instructions rejected when True
		self.lift_off_climb = None # PressureAlt under which "initial" vectors not followed (resets to None when reached)
		self.DEP_LDG_surface = None # DepLdgSurface expected or in use, if any
		self.APP_type = None  # expected/current ApproachType enum value, if any
		self.LDG_surface_reported_in_sight = False
		self.racetrack_holding = None # when fixed wing in racetrack: (inbound Heading, outbound timedelta flown)
		self.radio_msg_after_taxi = None

	@staticmethod
	def mkReadyForDep(dep_sfc):
		status = Status(airborne=False)
		status.DEP_LDG_surface = dep_sfc
		status.ready_for_DEP = True
		return status

	def dup(self):
		status = Status(not self.snapped_GND)
		status.ready_for_DEP = self.ready_for_DEP
		status.snapped_lined_up = self.snapped_lined_up
		status.rolling_TKOF_LDG = self.rolling_TKOF_LDG
		status.lift_off_climb = self.lift_off_climb
		status.DEP_LDG_surface = self.DEP_LDG_surface
		status.APP_type = self.APP_type
		status.LDG_surface_reported_in_sight = self.LDG_surface_reported_in_sight
		status.racetrack_holding = self.racetrack_holding
		status.radio_msg_after_taxi = self.radio_msg_after_taxi
		return status



class FlightParams:
	"""
	If "xpdrCode" is given, default XPDR mode is set, else XPDR will be off.
	"""
	def __init__(self, init_pos, init_alt, init_hdg, init_ias, xpdrCode=None):
		self.position = init_pos
		self.altitude = init_alt
		self.heading = init_hdg
		self.ias = init_ias
		self.XPDR_mode = '0' if xpdrCode is None else default_XPDR_mode # possible values are: '0', 'A', 'C', 'S' ('S' may squat depending on ACFT setting)
		self.XPDR_code = some(xpdrCode, settings.uncontrolled_VFR_XPDR_code)
		self.XPDR_idents = False
	
	def dup(self):
		params = FlightParams(self.position, self.altitude, self.heading, self.ias)
		params.XPDR_mode = self.XPDR_mode
		params.XPDR_code = self.XPDR_code
		params.XPDR_idents = self.XPDR_idents
		return params
	
	def realAltitude(self):
		return self.altitude.ftAMSL(env.QNH())



# Goals and arg types
#   APPROACH: ApproachType enum (int) requested
#   PARK:     str parking position
#   FLY_OUT:  (Navpoint, cruise alt/lvl, Airfield) tuple; navpoint and FL to be brought to before hand over (either can be None if don't matter), AD is destination
#   TRANSIT:  Airfield destination (CTR solo mode)

class Goal: # TODO use this?
	APPROACH, PARK, FLY_OUT, TRANSIT = range(4)

	def __init__(self, init_goal, init_arg):
		self.type = init_goal
		self.arg = init_arg

	def __str__(self):
		return 'G:%d:%s' % (self.type, self.arg)

	def dup(self):
		return Goal(self.type, self.arg)
