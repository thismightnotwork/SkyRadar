
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

from datetime import timedelta
from math import tan
from random import random, choice

from PyQt5.QtCore import QTimer

from ai.aircraft import AbstractAiAcft

from base.conflict import ground_separated
from base.coords import EarthCoords
from base.db import take_off_speed, touch_down_speed, stall_speed, maximum_speed, cruise_speed
from base.instr import Instruction, ApproachType
from base.nav import Airfield, NavpointError, world_navpoint_db
from base.params import AltFlSpec, PressureAlt, Speed, distance_travelled, time_to_fly, wind_effect
from base.route import Route
from base.text import TextMessage
from base.util import pop_all, some, m2NM, upper_1st

from ext.tts import new_voice, speech_str2txt, speech_str2tts, speak_callsign_commercial_flight, speak_callsign_tail_number

from gui.misc import signals

from session.config import settings
from session.env import env
from session.manager import SessionType


# ---------- Constants ----------

pilot_turn_speed_fixed_wing = 3 # degrees per second
pilot_turn_speed_helo = 7 # degrees per second
pilot_vert_speed = 1800 # ft / min
pilot_accel_fixed_wing = 3 # kt / s
pilot_accel_helo = 20 # kt / s
pilot_hdg_precision = 2 # degrees
pilot_alt_precision = 20 # ft
pilot_spd_precision = 5 # kt
pilot_nav_precision = 1 # NM
pilot_taxi_precision = .002 # NM
pilot_sight_range = 7.5 # NM
pilot_sight_ceiling = PressureAlt.fromFL(100)

fast_turn_factor = 2.5
fast_climb_descend_factor = 1.75
fast_accel_decel_factor = 1.75

airtaxi_height = 4 # ft
touch_down_distance_tolerance = .03 # NM
touch_down_height_tolerance = 50 # ft
touch_down_heading_tolerance = 5 # degrees
touch_down_speed_tolerance = 10 # kt
touch_down_speed_drop = 10 # kt
min_clearToLand_height = 50 # ft
taxi_max_turn_without_decel = 5 # degrees
intercept_max_angle = 10 # degrees on each side

taxi_speed = Speed(15)
taxi_turn_speed = Speed(2)
ldg_roll_speed = Speed(45)
default_initial_climb_AMSL = 5000 # ft
default_initial_climb_ASFC = 3000 # ft
min_lift_off_climb_ASFC = 800 # ft

short_final_dist = 4 # NM
short_final_dist_helos = 1.2 # NM
inbound_speed_reduce_start_FL = 150
descent_max_speed = Speed(235)
default_turn_off_angle = -60 # degrees
turn_off_choice_prob = .8
approach_angle = 30 # degrees
hldg_leg_ttf = timedelta(minutes=1)
ready_max_dist_to_tkof_point = .05 # NM
park_max_dist_to_gate_node = .1 # NM
vacate_min_dist_to_rwy = m2NM * 6

simulated_radio_signal_timeout = 2000 # ms

# -------------------------------



def ck_instr(accept_condition, msg_if_rejected):
	if not accept_condition:
		raise Instruction.Error(msg_if_rejected)


def GS_alt(thr_elev, fpa, dist):
	return PressureAlt.fromAMSL(thr_elev + 60.761 * fpa * dist, qnh=env.QNH())


def default_initial_climb_spec():
	return AltFlSpec(False, max(default_initial_climb_AMSL,
			default_initial_climb_ASFC + int(env.airport_data.field_elevation / 1000 + 1) * 1000))



def select_RWY_turnoff(rwy, acft_pos):
	lst_good, lst_sharp, lst_rwys, lst_backtrack = env.airport_data.ground_net.runwayTurnOffs(rwy, minRoll=rwy.threshold().distanceTo(acft_pos))
	turn_offs_ahead = lst_good if lst_good else lst_sharp + lst_rwys
	if turn_offs_ahead:
		selected_turn_off = turn_offs_ahead.pop(0)
		while turn_offs_ahead and random() < turn_off_choice_prob:
			selected_turn_off = turn_offs_ahead.pop(0)
		return Instruction(Instruction.TAXI, arg=selected_turn_off)
	elif not lst_backtrack: # no turn-offs available anywhere; step off "into the wild"
		return Instruction(Instruction.TAXI, arg=[], arg2=acft_pos.moved(rwy.orientation() + default_turn_off_angle,
				m2NM * (env.airport_data.physicalRunwayData(rwy.physicalRwyIndex())[0] / 2 + vacate_min_dist_to_rwy) / tan(abs(default_turn_off_angle))))
	else:
		return None # all turn-offs are behind; will have to backtrack



class ControlledAiAircraft(AbstractAiAcft):
	"""
	This class represents an AI aircraft in radio contact (controlled),
	usually with intentions (goal) unless acting as teacher.
	"""
	
	def __init__(self, callsign, acft_type, init_params, init_status, init_goal):
		"""
		Set init_goal to None for teacher aircraft.
		Solo aircraft should have a goal, which can change during the session, e.g. APPROACH can change to PARK after LDG.
		"""
		if cruise_speed(acft_type) is None:
			raise ValueError('Aborting ControlledAiAircraft construction: unknown cruise speed for %s' % acft_type)
		AbstractAiAcft.__init__(self, callsign, acft_type, init_params, init_status)
		self.pilot_voice = new_voice() if settings.session_manager.session_type == SessionType.SOLO else None
		self.goal = init_goal
		self.touch_and_go_on_LDG = False # set by teacher panel
		self.skid_off_RWY_on_LDG = False # set by teacher panel
		self.instructions = []
	
	
	## GENERAL ACCESS METHODS
	
	def pilotVoice(self):
		return self.pilot_voice

	def isInboundGoal(self):
		return self.goal in ApproachType.types or isinstance(self.goal, str)

	def isOutboundGoal(self):
		return isinstance(self.goal, tuple)
	
	def wantsToPark(self):
		return isinstance(self.goal, str)
	
	def canPark(self):
		if self.wantsToPark() and env.airport_data is not None: # wants a gate/pkpos
			pkg_pos = env.airport_data.ground_net.parkingPosition(self.goal)
			return self.params.position.distanceTo(pkg_pos) <= park_max_dist_to_gate_node
		else:
			return False
	
	def instrOfType(self, t):
		"""
		returns the instruction of given type, or None
		"""
		return next((i for i in self.instructions if i.type == t), None)

	def isClearedApp(self):
		return self.instrOfType(Instruction.CLEARED_APP) is not None
	
	def groundPointInSight(self, point):
		return self.params.altitude.diff(pilot_sight_ceiling) <= 0 \
			and self.params.position.distanceTo(point) <= pilot_sight_range
	
	def maxTurn(self, td):
		return (pilot_turn_speed_helo if self.isHelo() else pilot_turn_speed_fixed_wing) * td.total_seconds()
	
	def maxClimb(self, td):
		return pilot_vert_speed * td.total_seconds() / 60
	
	def maxSpdIncr(self, td):
		return (pilot_accel_helo if self.isHelo() else pilot_accel_fixed_wing) * td.total_seconds()
	
	
	
	## TICKING
	
	def doTick(self):
		for instr in self.instructions[:]:
			self.followInstruction(instr)
		if not self.status.snapped_GND and self.instrOfType(Instruction.HOLD_POSITION) is None: # control airborne horiz. displacement
			# 1. adjust speed (assigned, cruise, restricted), unless on final descent (covered by rwyFinalLegApproach/heloStraightInApproach methods)
			if self.instrOfType(Instruction.VECTOR_SPD) is None and not self.status.snapped_lined_up:
				if self.params.ias.diff(Speed(250)) >= 0 and self.params.altitude.diff(PressureAlt.fromFL(100)) <= 0:
					self.accelDecelTowards(Speed(250), fast=True) # enforce speed restriction under FL100
				elif self.isInboundGoal():
					if self.params.altitude.FL() <= inbound_speed_reduce_start_FL:
						instr = self.instrOfType(Instruction.VECTOR_ALT)
						if instr is not None and env.pressureAlt(instr.arg).FL() <= inbound_speed_reduce_start_FL:
							self.accelDecelTowards(descent_max_speed, accelOK=False) # we may already be slower
				else:
					self.accelDecelTowards(cruise_speed(self.aircraft_type).tas2ias(self.params.altitude))
			# 2. update position from air speed and wind
			tas = self.params.ias.ias2tas(self.params.altitude)
			w = env.primaryWeather()
			wind_info = None if w is None else w.mainWind()
			if self.status.snapped_lined_up or wind_info is None or wind_info[0] is None: # snapped on final approach (pilot compensates wind), or no wind to factor
				course = self.params.heading
				ground_speed = tas
			else: # factor in the effect of the wind
				course, ground_speed = wind_effect(self.params.heading, tas, wind_info[0], Speed(wind_info[1], unit=wind_info[3]))
			#DEBUGprint('Ticking %s (%s): alt. %s, course %s, gnd spd %s' % (self.identifier, self.aircraft_type, self.params.altitude.ft1013(), course.read(), ground_speed))
			self.params.position = self.params.position.moved(course, distance_travelled(self.tick_interval, ground_speed))
		pop_all(self.instructions, self.instructionDone)



	## INSTRUCTIONS

	## This is where instruction is given, possibly rejected by ACFT if makes no sense
	def instruct(self, instructions, read_back):
		"""
		instructions may pop others out of the list, but they ALWAYS end up in the list if no error is generated
		"""
		backup_instr = self.instructions[:]
		backup_status = self.status.dup()
		try:
			for instr in instructions:
				self.ingestInstruction(instr) # this modifies self.instructions if the instr is not rejected (and maybe status)
			if read_back:
				self.readBack(instructions)
		except Instruction.Error as exn:
			self.instructions = backup_instr
			self.status = backup_status
			if read_back:
				self.say('Unable. %s' % exn, True)
			raise exn


	def ingestInstruction(self, instr):
		# FIRST resolve additional data if needed
		# arg is a RWY or helipad name to expect
		if instr.type == Instruction.EXPECT_SFC:
			ck_instr(env.airport_data is not None, 'Sorry, at what airfield??')
			try:
				instr.resolved_arg = env.airport_data.runway(instr.arg)
			except KeyError:
				try:
					instr.resolved_arg = next(hpad for hpad in env.airport_data.helipads() if hpad.name == instr.arg)
				except StopIteration:
					raise Instruction.Error('Sorry, which %s??' % ('surface point' if self.isHelo() else 'runway'))
		# arg is navpoint name
		elif instr.type in [Instruction.VECTOR_DCT, Instruction.HOLD_AT_FIX, Instruction.INTERCEPT_NAV]:
			try:
				instr.resolved_arg = env.navpoints.findClosest(env.radarPos(), code=instr.arg)
			except NavpointError as err:
				raise Instruction.Error('Cannot identify \\NAVPOINT{%s}' % err)
		# arg is a string (route to go), optionally containing past (DEP) spec
		elif instr.type == Instruction.FOLLOW_ROUTE:
			# ensure destination if possible (never mind duplicating it)
			if self.isInboundGoal():
				dest = env.airport_data.navpoint
			elif self.isOutboundGoal():
				dest = self.goal[2]
			elif isinstance(self.goal, Airfield): # transiting in solo CTR session
				dest = self.goal
			else: # goal presumably None (teaching); we need to get a destination token from the end of the route string
				rsplit = instr.arg.rsplit(maxsplit=1)
				if len(rsplit) == 0:
					raise Instruction.Error('Empty route string.')
				try:
					dest = world_navpoint_db.fromSpec(rsplit[-1])
				except NavpointError as err:
					raise Instruction.Error('Could not interpret route destination %s' % err)
			instr.resolved_arg = Route(self.params.position, dest, instr.arg)

		# SECOND test if instruction acceptable, i.e. reject if nonsensical
		if instr.type in [Instruction.VECTOR_HDG, Instruction.VECTOR_DCT, Instruction.FOLLOW_ROUTE]:
			if self.status.snapped_GND: # instr = initial heading
				ck_instr(not self.isInboundGoal(), 'Not outbound.')
			else: # instr. while airborne
				ck_instr(not self.isClearedApp(), 'Already cleared for approach. Should I cancel clearance?')
				if instr.type == Instruction.VECTOR_DCT and not self.isHelo():
					ck_instr(self.params.position.distanceTo(instr.resolved_arg.coordinates) > pilot_nav_precision, 'Already at %s.' % instr.resolved_arg.code)
			pop_all(self.instructions, lambda i: i.type in [Instruction.VECTOR_HDG, Instruction.VECTOR_DCT, Instruction.FOLLOW_ROUTE,
					Instruction.INTERCEPT_NAV, Instruction.INTERCEPT_LOC, Instruction.HOLD_AT_FIX, Instruction.HOLD_POSITION])
			self.status.racetrack_holding = None

		elif instr.type == Instruction.VECTOR_ALT:
			if self.status.snapped_GND: # instr = initial climb
				ck_instr(not self.isInboundGoal(), 'Not outbound.')
			else: # instr = climb/descend
				ck_instr(not self.isClearedApp(), 'Already cleared for approach. Should I cancel clearance?')
			pop_all(self.instructions, lambda i: i.type == Instruction.VECTOR_ALT)

		elif instr.type == Instruction.VECTOR_SPD:
			ck_instr(not self.status.snapped_GND, 'Not airborne.')
			if self.status.snapped_lined_up:
				ck_instr(self.params.position.distanceTo(self.status.DEP_LDG_surface.touchDownPoint()) > short_final_dist, 'On short final.')
			ck_instr(instr.arg.ias2tas(self.params.altitude).diff(stall_speed(self.aircraft_type)) >= 0, 'Speed is too low.')
			ck_instr(instr.arg.ias2tas(self.params.altitude).diff(maximum_speed(self.aircraft_type)) <= 0, 'Cannot reach such speed.')
			pop_all(self.instructions, lambda i: i.type in [Instruction.VECTOR_SPD, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.CANCEL_SPD:
			if not self.isHelo() or self.instrOfType(Instruction.HOLD_POSITION) is None:
				ck_instr(self.instrOfType(Instruction.VECTOR_SPD) is not None, 'Observing no speed restriction.')
			pop_all(self.instructions, lambda i: i.type in [Instruction.CANCEL_SPD, Instruction.VECTOR_SPD, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.INTERCEPT_NAV:
			if self.status.snapped_GND: # instr. for departure, after initial climb
				ck_instr(not self.isInboundGoal(), 'Not outbound.')
			else:
				ck_instr(self.instrOfType(Instruction.HOLD_AT_FIX) is None and self.instrOfType(Instruction.HOLD_POSITION) is None, 'Still holding.')
				ck_instr(not self.status.snapped_lined_up and not self.isClearedApp(), 'Already cleared for approach.')
			pop_all(self.instructions, lambda i: i.type in [Instruction.INTERCEPT_NAV, Instruction.INTERCEPT_LOC])

		elif instr.type == Instruction.HOLD_AT_FIX:
			ck_instr(not self.status.snapped_GND, 'Not even airborne.')
			ck_instr(not self.status.snapped_lined_up, 'Already landing.')
			pop_all(self.instructions, lambda i: i.type in [Instruction.VECTOR_HDG, Instruction.VECTOR_DCT, Instruction.FOLLOW_ROUTE,
					Instruction.INTERCEPT_NAV, Instruction.INTERCEPT_LOC, Instruction.HOLD_AT_FIX, Instruction.HOLD_POSITION])
			self.status.racetrack_holding = None

		elif instr.type == Instruction.SQUAWK:
			pop_all(self.instructions, lambda i: i.type == Instruction.SQUAWK)

		elif instr.type == Instruction.CANCEL_APP:
			ck_instr(not self.status.snapped_GND and self.status.snapped_lined_up or self.isClearedApp(), 'Not on approach.')
			if self.status.APP_type == ApproachType.STRAIGHT_IN:
				self.ingestInstruction(Instruction(Instruction.HOLD_POSITION))
			pop_all(self.instructions, lambda i: i.type == Instruction.CANCEL_APP)

		elif instr.type == Instruction.TAXI:
			ck_instr(self.status.snapped_GND, 'Currently airborne!')
			ck_instr(not self.status.rolling_TKOF_LDG, ('Already taking off.' if self.status.ready_for_DEP else 'Still landing, stand by.'))
			self.status.ready_for_DEP = False
			self.status.snapped_lined_up = False
			self.status.radio_msg_after_taxi = None
			pop_all(self.instructions, lambda i: i.type in [Instruction.TAXI, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.HOLD_POSITION:
			if self.status.snapped_GND:
				ck_instr(not self.status.rolling_TKOF_LDG, ('Already taking off.' if self.status.ready_for_DEP else 'Still landing, stand by.'))
				pop_all(self.instructions, lambda i: i.type in [Instruction.HOLD_POSITION, Instruction.LINE_UP, Instruction.CLEARED_TKOF, Instruction.TAXI])
			else: # airborne
				ck_instr(self.isHelo(), 'Cannot just stop airborne.')
				pop_all(self.instructions, lambda i: i.type in [Instruction.HOLD_POSITION, Instruction.VECTOR_ALT, Instruction.VECTOR_SPD, Instruction.CLEARED_APP])
				# keep lateral vectors: position won't be updated anyway, and VECTOR_SPD reactivates them (as well as should "resume nav." instr. if later impl'ed)

		elif instr.type == Instruction.LINE_UP:
			ck_instr(self.status.snapped_GND, 'Currently airborne!')
			ck_instr(self.status.ready_for_DEP, 'Not ready for departure.')
			ck_instr(not self.status.rolling_TKOF_LDG, 'Already rolling!')
			ck_instr(instr.arg is None or instr.arg == self.status.DEP_LDG_surface, 'Ready for departure from %s. Confirm change?' % self.status.DEP_LDG_surface.readOut(tts=True))
			pop_all(self.instructions, lambda i: i.type in [Instruction.LINE_UP, Instruction.CLEARED_TKOF, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.CLEARED_TKOF:
			self.ingestInstruction(Instruction(Instruction.LINE_UP))
			pop_all(self.instructions, lambda i: i.type in [Instruction.CLEARED_TKOF, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.EXPECT_SFC:
			if self.status.snapped_GND: # resolved_arg surface is for DEP
				ck_instr(not self.status.rolling_TKOF_LDG, 'Already taking off.')
				ck_instr(not self.isInboundGoal(), 'Not requesting departure.')
				ck_instr(instr.resolved_arg.acceptsAcftType(self.aircraft_type), 'My aircraft type cannot use %s.' % instr.resolved_arg.readOut(tts=True))
			else: # ACFT is airborne, resolved_arg surface is for APP
				ck_instr(not self.isOutboundGoal(), 'Outbound.')
				ck_instr(not self.isClearedApp(), 'Already cleared for approach.')
				ck_instr(instr.resolved_arg.acceptsAcftType(self.aircraft_type), 'Cannot land on %s.' % instr.resolved_arg.readOut(tts=True))
				if instr.arg2 is not None: # instruction specified a type of approach
					ck_instr(self.goal is None or instr.arg2 == self.goal, 'Requesting %s approach.' % ApproachType.tts(self.goal))
					if instr.arg2 == ApproachType.STRAIGHT_IN:
						ck_instr(self.isHelo(), 'Do you mean \\SPLIT_CHARS{ILS} or visual?')
				self.status.APP_type = instr.arg2
				if self.status.APP_type is None and self.goal in ApproachType.types:
					self.status.APP_type = self.goal
				if self.status.APP_type == ApproachType.ILS: # RWY exists, requires ILS-capable
					ck_instr(instr.resolved_arg.hasILS(), 'Runway \\RWY{%s} has no \\SPLIT_CHARS{ILS}.' % instr.arg)
				self.status.LDG_surface_reported_in_sight = False
			self.status.DEP_LDG_surface = instr.resolved_arg
			pop_all(self.instructions, lambda i: i.type in [Instruction.EXPECT_SFC, Instruction.INTERCEPT_LOC])

		elif instr.type == Instruction.INTERCEPT_LOC:
			ck_instr(not self.status.snapped_GND, 'Not airborne!')
			ck_instr(not self.status.snapped_lined_up, 'Already landing.')
			ck_instr(self.instrOfType(Instruction.HOLD_AT_FIX) is None and self.instrOfType(Instruction.HOLD_POSITION) is None, 'Still holding.')
			if self.status.DEP_LDG_surface is None:
				ck_instr(instr.arg is not None, 'No runway given.')
				self.ingestInstruction(Instruction(Instruction.EXPECT_SFC, arg=instr.arg, arg2=ApproachType.ILS))
			else:
				ck_instr(instr.arg is None or instr.arg == self.status.DEP_LDG_surface.name, 'Was expecting %s.' % self.status.DEP_LDG_surface.readOut(tts=True))
			if self.status.APP_type is not None:
				ck_instr(self.status.APP_type == ApproachType.ILS, 'Was not expecting \\SPLIT_CHARS{ILS}.')
			if self.goal is not None:
				ck_instr(self.goal == ApproachType.ILS, 'Requesting %s approach.' % ApproachType.tts(self.goal))
			pop_all(self.instructions, lambda i: i.type in [Instruction.INTERCEPT_NAV, Instruction.INTERCEPT_LOC])

		elif instr.type == Instruction.CLEARED_APP:
			ck_instr(not self.status.snapped_GND, 'Not airborne!')
			ck_instr(not self.status.snapped_lined_up, 'Already on approach.')
			if self.status.DEP_LDG_surface is None:
				ck_instr(instr.arg is not None, 'Sorry, approach what %s?' % ('surface point' if self.isHelo() else 'runway'))
				self.ingestInstruction(Instruction(Instruction.EXPECT_SFC, arg=instr.arg, arg2=instr.arg2)) # this checks against goal if any and falls back on it if no arg2
			else:
				ck_instr(instr.arg is None or instr.arg == self.status.DEP_LDG_surface.name, 'Was expecting %s. Confirm change?' % self.status.DEP_LDG_surface.readOut(tts=True))
				if self.status.APP_type is None and self.goal in ApproachType.types:
					self.status.APP_type = self.goal
				if self.status.APP_type is None: # teaching or non-landing goal
					ck_instr(instr.arg2 is not None or settings.session_manager.session_type == SessionType.TEACHER, 'Sorry, what kind of approach?')
					self.status.APP_type = instr.arg2
				else:
					ck_instr(instr.arg2 is None or instr.arg2 == self.status.APP_type, 'Was expecting %s.' % ApproachType.tts(self.status.APP_type))
			if self.instrOfType(Instruction.HOLD_AT_FIX) is not None or self.instrOfType(Instruction.HOLD_POSITION) is not None:
				ck_instr(self.isHelo(), 'Still on hold; should we break out?')
				ck_instr(self.status.APP_type == ApproachType.STRAIGHT_IN, 'Was holding; how should we break out for this approach?')
			if self.status.APP_type == ApproachType.VISUAL:
				ck_instr(self.status.LDG_surface_reported_in_sight, 'Runway not in sight yet.')
				# TODO check on correct side of RWY: ck_instr(..., 'On wrong side of the runway.')
			pop_all(self.instructions, lambda i: i.type in [Instruction.INTERCEPT_LOC, Instruction.CLEARED_APP, Instruction.HOLD_AT_FIX, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.CLEARED_LDG:
			if settings.session_manager.session_type == SessionType.SOLO:
				ck_instr(settings.solo_role_TWR, 'Expecting \\ATC{TWR} to issue this instruction.')
			ck_instr(not self.status.snapped_GND, 'Already on the ground!')
			ck_instr(self.status.snapped_lined_up, 'Not on final approach.')
			if instr.arg is not None:
				ck_instr(instr.arg == self.status.DEP_LDG_surface.name, 'Established for %s. Should I cancel clearance?' % self.status.DEP_LDG_surface.readOut(tts=True))
			pop_all(self.instructions, lambda i: i.type in [Instruction.CLEARED_LDG, Instruction.HOLD_POSITION])

		elif instr.type == Instruction.HAND_OVER:
			if settings.session_manager.session_type == SessionType.SOLO:
				ck_instr(settings.session_manager.handoverGuard(self, instr.arg) is None, 'Staying with you.')
			pop_all(self.instructions, lambda i: i.type == Instruction.HAND_OVER)

		elif instr.type == Instruction.DEP_CLEARANCE:
			ck_instr(False, 'Unable to interpret departure clearances.')

		# FINALLY: no instruction error raised, so ingest the instruction
		self.instructions.append(instr)


	## This is where the ACFT does something about instructions in its currently active list, considering one at a time
	def followInstruction(self, instr):
		if instr.type == Instruction.VECTOR_HDG:
			if not self.status.snapped_GND and self.status.lift_off_climb is None:
				self.turnTowards(instr.arg, tolerance=pilot_hdg_precision, rightTurn=instr.arg2)

		elif instr.type == Instruction.VECTOR_DCT:
			navpoint = instr.resolved_arg
			if not self.status.snapped_GND and self.status.lift_off_climb is None:
				self.flyTowards(navpoint.coordinates)
			if self.params.position.distanceTo(navpoint.coordinates) <= pilot_nav_precision: # navpoint reached
				if self.isHelo(): # hold position; fixed wing are assumed to continue on present heading
					self.ingestInstruction(Instruction(Instruction.HOLD_POSITION))
				self.say(('Reached' if self.isHelo() else 'Passing') + ' \\NAVPOINT{%s}' % navpoint.code, False)

		elif instr.type == Instruction.VECTOR_ALT:
			if not self.status.snapped_GND:
				self.climbDescendTowards(env.pressureAlt(instr.arg))
				if self.status.lift_off_climb and self.params.altitude.diff(self.status.lift_off_climb) > 0:
					self.status.lift_off_climb = None

		elif instr.type == Instruction.VECTOR_SPD:
			self.accelDecelTowards(instr.arg)

		elif instr.type == Instruction.FOLLOW_ROUTE:
			if not self.status.snapped_GND and self.status.lift_off_climb is None:
				self.flyTowards(instr.resolved_arg.currentWaypoint(self.params.position).coordinates)

		elif instr.type == Instruction.CANCEL_SPD:
			pass # this instruction is immediately performed on "instruct" (VECTOR_SPD popped on "ingest")

		elif instr.type == Instruction.HOLD_AT_FIX: # navpoint prior resolved when instruction ingested
			if self.status.racetrack_holding is None: # going for fix first
				if self.params.position.distanceTo(instr.resolved_arg.coordinates) > pilot_nav_precision: # fix not reached yet
					self.flyTowards(instr.resolved_arg.coordinates)
				elif self.isHelo(): # helo at fix should just stay there; helos do not racetrack
					self.ingestInstruction(Instruction.HOLD_POSITION)
				else: # fixed wing just reached fix; begin racetrack
					self.status.racetrack_holding = self.params.heading + 300, timedelta(0) # until inbound hdg from instr.: outbound legs = 120 degree right-turn from first fix entrance
			else: # in a fixed wing's racetrack pattern
				inbound_hdg, outbound_time_flown = self.status.racetrack_holding
				if outbound_time_flown < hldg_leg_ttf: # turning or flying away from fix
					outbound_hdg = inbound_hdg.opposite()
					self.turnTowards(outbound_hdg, rightTurn=True)
					if self.params.heading.diff(outbound_hdg, tolerance=pilot_hdg_precision) == 0:
						self.status.racetrack_holding = inbound_hdg, outbound_time_flown + self.tick_interval
				else: # turning or flying back to fix
					self.flyTowards(instr.resolved_arg.coordinates, rightTurn=True)
					if self.params.position.distanceTo(instr.resolved_arg.coordinates) <= pilot_nav_precision:
						self.status.racetrack_holding = inbound_hdg, timedelta(0)

		elif instr.type == Instruction.SQUAWK:
			if self.params.XPDR_mode == '0':
				self.params.XPDR_mode = 'C'
			self.params.XPDR_code = instr.arg

		elif instr.type == Instruction.CANCEL_APP:
			if self.status.APP_type != ApproachType.STRAIGHT_IN: # they ingest a HOLD_POSITION with the CANCEL_APP
				self.MISAP()

		elif instr.type == Instruction.INTERCEPT_NAV: # navpoint prior resolved when instruction ingested
			if not self.status.snapped_GND and self.status.lift_off_climb is None:
				self.intercept(instr.resolved_arg.coordinates, instr.arg2) # FUTURE navaid intercept range limit?

		elif instr.type == Instruction.INTERCEPT_LOC: # self.status.DEP_LDG_surface is necessarily a runway
			self.intercept(self.status.DEP_LDG_surface.threshold(), self.status.DEP_LDG_surface.appCourse(), radial=False, rangeLimit=self.status.DEP_LDG_surface.LOC_range)

		elif instr.type == Instruction.EXPECT_SFC:
			sfc = instr.resolved_arg
			if self.status.snapped_GND: # instruction used for DEP; should we report ready?
				if self.instrOfType(Instruction.TAXI) is None and not self.status.rolling_TKOF_LDG:
					if sfc.isRunway():
						thr = sfc.threshold(dthr=False)
						rwy_limit = thr.moved(sfc.orientation(), sfc.length() / 2) # FUTURE roll-off dist depending on ACFT type
						ready = self.params.position.toRadarCoords().isBetween(thr.toRadarCoords(), rwy_limit.toRadarCoords(), ready_max_dist_to_tkof_point, offsetBeyondEnds=True)
					else: # sfc is a helipad
						ready = self.params.position.distanceTo(sfc.centre) < ready_max_dist_to_tkof_point
					if ready:
						self.status.ready_for_DEP = True
						self.say('Short of %s, ready for departure.' % sfc.readOut(tts=True), False)
			else: # ACFT is airborne; should we report a LDG surface in sight?
				if self.status.APP_type == ApproachType.VISUAL and not self.isClearedApp() \
						and not self.status.LDG_surface_reported_in_sight and self.groundPointInSight(sfc.touchDownPoint()):
					self.say(upper_1st(sfc.readOut(tts=True)) + ' in sight.', False)
					self.status.LDG_surface_reported_in_sight = True

		elif instr.type == Instruction.TAXI:
			if instr.arg: # still got nodes to taxi
				if self.taxiTowardsReached(env.airport_data.ground_net.nodePosition(instr.arg[0])):
					del instr.arg[0]
					if len(instr.arg) == 0 and self.canPark():
						self.say('Request contact with ramp', False)
			elif instr.arg2 is not None: # point coords or final pkg pos name
				if self.taxiTowardsReached(instr.arg2 if isinstance(instr.arg2, EarthCoords) else env.airport_data.ground_net.parkingPosition(instr.arg2)):
					instr.arg2 = None
			if not instr.arg and instr.arg2 is None and self.status.radio_msg_after_taxi is not None: # just cleared last taxi waypoint in instruction
				self.say(self.status.radio_msg_after_taxi, False)
				self.status.radio_msg_after_taxi = None

		elif instr.type == Instruction.HOLD_POSITION:
			self.params.ias = Speed(0)

		elif instr.type == Instruction.CLEARED_APP:
			if self.status.snapped_lined_up:
				if self.status.APP_type == ApproachType.STRAIGHT_IN:
					self.heloStraightInApproach(self.status.DEP_LDG_surface)
				else: # RWY APP requiring a final leg
					self.rwyFinalLegApproach(self.status.DEP_LDG_surface)
			else: # not yet on final APP; should we snap onto it?
				if self.status.APP_type == ApproachType.STRAIGHT_IN or self.status.APP_type == ApproachType.VISUAL or \
						self.intercept(self.status.DEP_LDG_surface.threshold(), self.status.DEP_LDG_surface.appCourse(),
						radial=False, tolerant=False, rangeLimit=self.status.DEP_LDG_surface.LOC_range):
					self.status.snapped_lined_up = True  # pops VECTOR_ALT from instructions

		elif instr.type == Instruction.LINE_UP:
			assert self.status.DEP_LDG_surface, 'Got a "line up" instruction without a RWY/helipad.'
			if self.status.DEP_LDG_surface.isRunway():
				rwy_end = self.status.DEP_LDG_surface.opposite().threshold(dthr=False)
				minimum_roll_dist = self.status.DEP_LDG_surface.length() * 2 / 3 # FUTURE roll-off dist depending on ACFT type
				gn = env.airport_data.ground_net
				line_up_nodes = gn.nodes(lambda n: gn.nodeIsInSourceData(n) and
						gn.nodeIsRwyCentre(n, self.status.DEP_LDG_surface.name) and gn.nodePosition(n).distanceTo(rwy_end) >= minimum_roll_dist)
				if len(line_up_nodes) > 0:
					line_up_point = min((gn.nodePosition(n) for n in line_up_nodes), key=self.params.position.distanceTo)
				else: # no nodes; choose a direct point on RWY
					rwy_dthr = self.status.DEP_LDG_surface.threshold(dthr=True)
					rcoords_me = self.params.position.toRadarCoords()
					rcoords_dthr = rwy_dthr.toRadarCoords()
					rcoords_end = rwy_end.toRadarCoords()
					if rcoords_me.isBetween(rcoords_dthr, rcoords_end, ready_max_dist_to_tkof_point + 1, offsetBeyondEnds=False):
						line_up_point = EarthCoords.fromRadarCoords(rcoords_me.orthProj(rcoords_dthr, rcoords_end))
					else: # ACFT is behind the DTHR (not between RWY ends)
						line_up_point = rwy_dthr.moved(self.status.DEP_LDG_surface.orientation(), .03)
				line_up_hdg = self.status.DEP_LDG_surface.orientation()
			else: # sfc is a helipad
				line_up_point = self.status.DEP_LDG_surface.centre
				hdg_instr = self.instrOfType(Instruction.VECTOR_HDG)
				line_up_hdg = self.status.DEP_LDG_surface.param_preferred_DEP_course if hdg_instr is None else hdg_instr.arg
			# THEN (when point reached) do line-up action on RWY, or turn to face DEP heading on helipad if any is given
			if self.taxiTowardsReached(line_up_point):
				if line_up_hdg is not None and self.params.heading.diff(line_up_hdg, tolerance=.01) != 0:
					self.turnTowards(line_up_hdg, fastOK=True)
				elif self.instrOfType(Instruction.CLEARED_TKOF):
					self.status.rolling_TKOF_LDG = True

		elif instr.type == Instruction.CLEARED_TKOF:
			assert self.status.DEP_LDG_surface, 'Got a "take off" clearance without a RWY/helipad.'
			if self.status.rolling_TKOF_LDG:
				self.taxiForward()
				self.params.ias += self.maxSpdIncr(self.tick_interval)
				if self.params.ias.ias2tas(self.params.altitude).diff(take_off_speed(self.aircraft_type)) >= 0: # LIFT OFF!
					self.status.snapped_GND = False
					self.status.ready_for_DEP = False
					self.status.snapped_lined_up = False
					self.status.rolling_TKOF_LDG = False
					if self.status.DEP_LDG_surface.isRunway():
						self.status.lift_off_climb = self.params.altitude + min_lift_off_climb_ASFC
					if self.instrOfType(Instruction.VECTOR_ALT) is None:
						self.instructions.append(Instruction(Instruction.VECTOR_ALT, arg=default_initial_climb_spec()))

		elif instr.type == Instruction.CLEARED_LDG:
			if self.status.rolling_TKOF_LDG: # decelerating/breaking or skidding on RWY (or virtual equivalent on helipad)
				if self.status.DEP_LDG_surface.isRunway(): # make a slow-down roll before vacating or skidding off
					self.taxiForward()
					if self.isHelo():
						self.turnTowards(self.status.DEP_LDG_surface.orientation(), fastOK=True)
					if self.params.ias.diff(ldg_roll_speed) > 0: # keep breaking until controlled speed
						self.accelDecelTowards(ldg_roll_speed, tol=0)
					elif self.skid_off_RWY_on_LDG:
						if self.status.snapped_lined_up: # start skidding
							self.status.snapped_lined_up = False # but keep rolling_TKOF_LDG until excursion complete
							self.params.heading += 10 if random() < .5 else -10
						elif not self.status.DEP_LDG_surface.pointIsOnSurface(self.params.position):
							self.status.rolling_TKOF_LDG = False # CLEARED_LDG will pop
							self.params.ias = Speed(0)
					else: # controlled speed just reached; select turn-off and ingest as taxi instruction
						self.status.snapped_lined_up = False
						self.status.rolling_TKOF_LDG = False # CLEARED_LDG will pop; taxi instruction takes over
						# Decide on a taxi turn-off
						taxi_instr = select_RWY_turnoff(self.status.DEP_LDG_surface, self.params.position)
						if taxi_instr is None: # backtrack needed
							taxi_instr = Instruction(Instruction.TAXI, arg=[], arg2=self.params.position.moved(self.status.DEP_LDG_surface.orientation(), m2NM * 10)) # just a small taxi forward
							radio_msg = 'Request backtrack on %s to vacate' % self.status.DEP_LDG_surface.readOut(tts=True)
							if self.wantsToPark():
								radio_msg += ' and taxi to %s \\SPELL_ALPHANUMS{%s}' % (env.airport_data.ground_net.parkingPosInfo(self.goal)[2], self.goal)
						else: # regular turn-off ahead or off-net side step when no route could be found to exit RWY
							radio_msg = upper_1st(self.status.DEP_LDG_surface.readOut(tts=True)) + ' clear'
							if self.wantsToPark():
								radio_msg += ', requesting taxi to %s \\SPELL_ALPHANUMS{%s}' % (env.airport_data.ground_net.parkingPosInfo(self.goal)[2], self.goal)
						self.ingestInstruction(taxi_instr)
						self.status.radio_msg_after_taxi = radio_msg
				else: # just reached a helipad
					self.status.snapped_lined_up = False
					self.status.rolling_TKOF_LDG = False # CLEARED_LDG will pop; taxi instruction takes over
					self.ingestInstruction(Instruction(Instruction.TAXI, arg=[], arg2=self.params.position.moved(self.params.heading, m2NM * 10))) # just a small taxi forward
					if self.wantsToPark():
						self.status.radio_msg_after_taxi = 'Requesting taxi to %s \\SPELL_ALPHANUMS{%s}' % (env.airport_data.ground_net.parkingPosInfo(self.goal)[2], self.goal)

		elif instr.type == Instruction.HAND_OVER:
			link = env.cpdlc.liveDataLink(self.identifier)
			if link is not None:
				link.terminate(False)
			self.released = True

		elif instr.type == Instruction.DEP_CLEARANCE:
			pass # This is rejected on ingest. Nothing to do at this point.

		elif instr.type == Instruction.SAY_INTENTIONS:
			pass # This is followed on read-back. Nothing to do at this point.


	## This is where the conditions are given for getting rid of instructions in ACFT instr. lists
	def instructionDone(self, instr):
		if instr.type in [Instruction.VECTOR_HDG, Instruction.VECTOR_SPD]:
			return False
		elif instr.type == Instruction.VECTOR_ALT:
			return not self.status.snapped_GND and self.status.snapped_lined_up
		elif instr.type == Instruction.VECTOR_DCT:
			return self.params.position.distanceTo(instr.resolved_arg.coordinates) <= pilot_nav_precision
		elif instr.type == Instruction.FOLLOW_ROUTE:
			return self.params.position.distanceTo(instr.resolved_arg.waypoint(instr.resolved_arg.legCount() - 1).coordinates) <= pilot_nav_precision
		elif instr.type == Instruction.CANCEL_SPD:
			return self.instrOfType(Instruction.VECTOR_SPD) is None
		elif instr.type == Instruction.HOLD_AT_FIX:
			return False
		elif instr.type == Instruction.SQUAWK:
			return self.params.XPDR_code == instr.arg
		elif instr.type == Instruction.CANCEL_APP:
			return not self.status.snapped_GND and self.instrOfType(Instruction.CLEARED_LDG) is None and not self.isClearedApp()
		elif instr.type == Instruction.INTERCEPT_NAV:
			return False
		elif instr.type == Instruction.INTERCEPT_LOC:
			return self.isClearedApp()
		elif instr.type == Instruction.EXPECT_SFC: # used for either DEP or APP
			return self.status.ready_for_DEP or self.status.snapped_lined_up
		elif instr.type == Instruction.TAXI:
			return len(instr.arg) == 0 and instr.arg2 is None
		elif instr.type == Instruction.HOLD_POSITION:
			return False
		elif instr.type == Instruction.CLEARED_APP:
			return self.status.snapped_GND
		elif instr.type == Instruction.CLEARED_LDG: # decelerated enough to vacate or performing touch-and-go
			return self.status.snapped_GND and not self.status.snapped_lined_up and not self.status.rolling_TKOF_LDG or self.instrOfType(Instruction.CLEARED_TKOF) is not None
		elif instr.type == Instruction.LINE_UP:
			return self.status.rolling_TKOF_LDG
		elif instr.type == Instruction.CLEARED_TKOF:
			return not self.status.snapped_GND
		elif instr.type == Instruction.HAND_OVER:
			return self.released
		elif instr.type == Instruction.SAY_INTENTIONS: # This is followed directly in the read-back
			return True
		elif instr.type == Instruction.DEP_CLEARANCE: # This is rejected immediately, so do not keep
			return True
		else:
			assert False, 'instructionDone: unknown instruction %s' % instr


	## AUXILIARY METHODS FOR INSTRUCTION FOLLOWING

	def taxiForward(self, maxdist=None):
		dist = distance_travelled(self.tick_interval, self.params.ias)
		if maxdist is not None and dist > maxdist:
			dist = maxdist
		new_pos = self.params.position.moved(self.params.heading, dist)
		if self.status.rolling_TKOF_LDG \
				or all(ground_separated(other, new_pos, self.aircraft_type) or
					new_pos.distanceTo(other.params.position) > self.params.position.distanceTo(other.params.position)
					for other in settings.session_manager.getAircraft() if other is not self and other.status.snapped_GND):
			self.params.position = new_pos
			self.params.altitude = env.groundPressureAlt(self.params.position)
	
	def taxiTowardsReached(self, target):
		dist = self.params.position.distanceTo(target)
		if dist <= pilot_taxi_precision: # target reached: stop
			self.params.ias = Speed(0)
			return True
		else: # must move
			hdg = self.params.position.headingTo(target)
			diff = self.params.heading.diff(hdg, tolerance=taxi_max_turn_without_decel)
			if diff == 0: # more or less facing goal
				self.params.heading = hdg
				self.params.ias = taxi_speed
			else: # must turn towards target point
				self.turnTowards(hdg, fastOK=True)
				self.params.ias = taxi_turn_speed
			self.taxiForward(maxdist=dist)
			return False # target not known to be reached yet

	def flyTowards(self, coords, rightTurn=None):
		self.turnTowards(self.params.position.headingTo(coords), tolerance=pilot_hdg_precision, rightTurn=rightTurn)
	
	def turnTowards(self, hdg, tolerance=0, fastOK=False, rightTurn=None):
		"""
		works airborne and on ground. "direction": True to force a right turn, False to force a left turn
		"""
		diff = hdg.diff(self.params.heading, tolerance)
		if diff != 0:
			max_abs_turn = self.maxTurn(self.tick_interval)
			if fastOK:
				max_abs_turn *= fast_turn_factor
			self.params.heading += (1 if some(rightTurn, diff > 0) else -1) * min(abs(diff), max_abs_turn)
	
	def climbDescendTowards(self, alt, climbOK=True, descendOK=True):
		diff = alt.diff(self.params.altitude, tolerance=pilot_alt_precision)
		if diff < 0 and descendOK or diff > 0 and climbOK:
			vert = min(self.maxClimb(self.tick_interval), abs(diff))
			self.params.altitude = self.params.altitude + (vert if diff > 0 else -vert)

	def accelDecelTowards(self, spd, accelOK=True, decelOK=True, fast=False, tol=pilot_spd_precision):
		diff = spd.diff(self.params.ias, tolerance=tol)
		if diff < 0 and decelOK or diff > 0 and accelOK:
			spdincr = min((fast_accel_decel_factor if fast else 1) * self.maxSpdIncr(self.tick_interval), abs(diff))
			self.params.ias = self.params.ias + (spdincr if diff > 0 else -spdincr)

	def intercept(self, point, hdg, radial=True, bearing=True, tolerant=True, force=False, rangeLimit=None):
		dct = self.params.position.toRadarCoords().headingTo(point.toRadarCoords())
		todiff = hdg.diff(dct)
		tonotfrom = abs(todiff) < 90
		diff = todiff if tonotfrom else hdg.diff(dct.opposite())
		interception = False
		if bearing and tonotfrom or radial and not tonotfrom:
			interception |= abs(diff) <= intercept_max_angle
		if rangeLimit is not None:
			interception &= self.params.position.distanceTo(point) <= rangeLimit
		if interception or force: # inside cone or forced
			pop_all(self.instructions, lambda i: i.type in [Instruction.VECTOR_HDG, Instruction.VECTOR_DCT, Instruction.FOLLOW_ROUTE])
			self.turnTowards(hdg + -diff * approach_angle / intercept_max_angle, tolerance=(pilot_hdg_precision if tolerant else 0), fastOK=True)
		return interception

	def heloStraightInApproach(self, ldgsfc):
		target_point = ldgsfc.touchDownPoint()
		hdist = self.params.position.distanceTo(target_point)
		vdist = self.params.altitude.diff(env.groundPressureAlt(target_point)) - airtaxi_height # feet to descend
		self.turnTowards(self.params.position.headingTo(target_point), fastOK=True)
		if hdist <= short_final_dist: # must reach touch-down speed soon; forget any ATC speed restrictions
			pop_all(self.instructions, lambda i: i.type == Instruction.VECTOR_SPD)
		if hdist <= short_final_dist_helos:
			self.accelDecelTowards(touch_down_speed(self.aircraft_type).tas2ias(self.params.altitude), fast=True)
		try:
			v_spd = vdist * timedelta(minutes=1) / time_to_fly(hdist, self.params.ias.ias2tas(self.params.altitude))
			if v_spd > pilot_vert_speed: # steep approach; consider max vertical speed and adjust IAS accordingly
				v_spd = pilot_vert_speed
				self.params.ias = Speed(hdist * 60 * v_spd / vdist).tas2ias(self.params.altitude)
			self.params.altitude -= min(vdist, v_spd * self.tick_interval / timedelta(minutes=1))
		except ValueError: # raised by time_to_fly if speed too low, e.g. zero when coming out of a hold
			pass
		if hdist < pilot_taxi_precision and abs(vdist) < pilot_alt_precision: # REACHED TARGET POSITION!
			pop_all(self.instructions, lambda i: i.type == Instruction.VECTOR_SPD)
			self.params.ias = Speed(0)
			self.status.snapped_GND = True
			self.status.rolling_TKOF_LDG = True

	def rwyFinalLegApproach(self, runway):
		self.intercept(runway.threshold(), runway.appCourse(), radial=False, tolerant=False, force=True)
		touch_down_point = runway.touchDownPoint()
		touch_down_dist = self.params.position.distanceTo(touch_down_point)
		touch_down_elev = env.elevation(touch_down_point) # assuming XPDR in the wheels for here. OK for radar; live FGMS pos packet corrected if FGFS model height is known
		gs_diff = self.params.altitude.diff(GS_alt(touch_down_elev, runway.param_FPA, touch_down_dist))
		if gs_diff > 0: # must descend
			self.params.altitude -= max(0, min(fast_climb_descend_factor * self.maxClimb(self.tick_interval), gs_diff))
		on_short = touch_down_dist <= short_final_dist
		if on_short: # must reach touch-down speed soon; forget any ATC speed restrictions
			pop_all(self.instructions, lambda i: i.type == Instruction.VECTOR_SPD)
		if self.instrOfType(Instruction.VECTOR_SPD) is None:
			if not self.isHelo() or on_short: # reduce speed for touchdown
				self.accelDecelTowards(touch_down_speed(self.aircraft_type).tas2ias(self.params.altitude), fast=on_short)
		rwy_ori = runway.orientation()
		height = self.params.altitude.diff(env.groundPressureAlt(self.params.position))
		if not settings.teacher_ACFT_touch_down_without_clearance and height < min_clearToLand_height and self.instrOfType(Instruction.CLEARED_LDG) is None:
			self.say('Going around; not cleared to land.', False)
			self.MISAP()
		elif touch_down_dist <= touch_down_distance_tolerance: # Attempt touch down
			alt_check = height <= touch_down_height_tolerance
			hdg_check = self.params.heading.diff(rwy_ori, tolerance=touch_down_heading_tolerance) == 0
			speed_check = self.params.ias.ias2tas(self.params.altitude).diff(touch_down_speed(self.aircraft_type), tolerance=touch_down_speed_tolerance) <= 0
			if alt_check and hdg_check and speed_check and random() >= settings.solo_MISAP_probability: # TOUCH DOWN!
				self.params.heading = rwy_ori
				self.params.ias -= touch_down_speed_drop
				self.status.snapped_GND = True
				self.status.rolling_TKOF_LDG = True
				if self.touch_and_go_on_LDG:
					self.status.ready_for_DEP = True
					self.instructions.append(Instruction(Instruction.CLEARED_TKOF))
				elif settings.session_manager.session_type == SessionType.SOLO and settings.solo_role_GND: # set new parking goal
					self.goal = choice(env.airport_data.ground_net.parkingPositions(acftType=self.aircraft_type))
			else: # Missed approach
				reason = (('not stabilised' if speed_check else 'too fast') if hdg_check else 'not lined up') if alt_check else 'too high'
				self.say('Going around, %s for touch-down.' % reason, False)
				self.MISAP()
	
	def MISAP(self):
		self.status.snapped_lined_up = False
		pop_all(self.instructions, lambda i: i.type in [Instruction.CLEARED_LDG, Instruction.CLEARED_APP, Instruction.INTERCEPT_LOC])
		alt_spec = default_initial_climb_spec()
		if self.params.altitude.diff(env.pressureAlt(alt_spec)) < 0:
			self.instructions.append(Instruction(Instruction.VECTOR_ALT, arg=alt_spec))
	
	
	## RADIO
	
	def say(self, txt_message, responding, initAddressee=None):
		"""
		responding: True if callsign should come last on the radio; False will place callsign first.
		initAddressee: msg starts with addressee callsign, followed by own without shortening.
		"""
		signals.incomingTextRadioMsg.emit(TextMessage(self.identifier, speech_str2txt(txt_message)))
		if settings.session_manager.session_type == SessionType.SOLO:
			if settings.solo_voice_readback and self.pilotVoice() is not None:
				if self.airline is None:
					cs = speak_callsign_tail_number(self.identifier, shorten=(initAddressee is None))
				else:
					cs = speak_callsign_commercial_flight(self.airline, self.identifier[len(self.airline):])
				msg = speech_str2tts(txt_message)
				if initAddressee is None:
					tts_struct = [msg, cs] if responding else [cs, msg]
				else: # explicitly addressing
					tts_struct = [initAddressee, cs, msg]
				signals.voiceMsg.emit(self, ', '.join(tts_struct)) # takes care of the RDF signal
			else: # Not synthesising voice, but should simulate a radio signal for RDF system
				def endrdf():
					if env.rdf is not None: # safeguard in case window closes (env resets) before "end RDF" timer is shot
						self.resetPtt()
				self.setPtt()
				QTimer.singleShot(simulated_radio_signal_timeout, endrdf)
	
	def readBack(self, instr_sequence): # assuming instructions were ingested with no error (nothing rejected)
		lst = []
		for instr in instr_sequence:
			if instr.type == Instruction.VECTOR_HDG:
				if instr.arg2 is None:
					msg = 'Heading '
				else:
					msg = 'Turn ' + ('right' if instr.arg2 else 'left') + ' heading '
				msg += '\\SPELL_ALPHANUMS{%s}' % instr.arg.read()
				if self.status.snapped_GND:
					msg += ' after departure'
			elif instr.type == Instruction.VECTOR_ALT:
				msg = 'Initial climb ' if self.status.snapped_GND else ''
				msg += '\\FL_ALT{%s}' % instr.arg.toStr()
			elif instr.type == Instruction.VECTOR_SPD:
				msg = '\\SPEED{%s}' % instr.arg
			elif instr.type == Instruction.VECTOR_DCT:
				msg = 'Direct \\NAVPOINT{%s}' % instr.resolved_arg.code
				if self.status.snapped_GND:
					msg += ' after departure'
			elif instr.type == Instruction.CANCEL_SPD:
				msg = 'Speed my discretion'
			elif instr.type == Instruction.FOLLOW_ROUTE:
				msg = 'Route copied'
				if not self.status.snapped_GND:
					', proceeding to \\NAVPOINT{%s}' % instr.resolved_arg.currentWaypoint(self.params.position)
			elif instr.type == Instruction.HOLD_AT_FIX:
				msg = 'Hold at \\NAVPOINT{%s}' % instr.resolved_arg.code
			elif instr.type == Instruction.SQUAWK:
				msg = '\\SPELL_ALPHANUMS{%04o}' % instr.arg
			elif instr.type == Instruction.CANCEL_APP:
				msg = 'Cancel approach'
				if self.status.APP_type == ApproachType.STRAIGHT_IN:
					msg +=', holding position'
			elif instr.type == Instruction.HAND_OVER:
				msg = 'With \\ATC{%s}, thank you, good bye' % instr.arg
			elif instr.type == Instruction.LINE_UP:
				msg = 'Line up and wait %s' % self.status.DEP_LDG_surface.readOut(tts=True)
			elif instr.type == Instruction.INTERCEPT_NAV:
				msg = 'Intercept \\NAVPOINT{%s} \\SPELL_ALPHANUMS{%s}' % (instr.resolved_arg.code, instr.arg2.read())
				if self.status.snapped_GND:
					msg += ' after departure'
			elif instr.type == Instruction.INTERCEPT_LOC:
				msg = 'Intercept localiser %s' % self.status.DEP_LDG_surface.readOut(tts=True)
			elif instr.type == Instruction.EXPECT_SFC:
				sfc_tts = instr.resolved_arg.readOut(tts=True)
				if self.status.snapped_GND:
					msg = upper_1st(sfc_tts) + ', will report ready for departure'
				else: # airborne
					msg = 'Expecting '
					if self.status.APP_type is not None:
						msg += ApproachType.tts(self.status.APP_type) + ' '
					msg += sfc_tts
			elif instr.type == Instruction.TAXI:
				if env.airport_data is None:
					msg = 'Unable to taxi'
				else:
					msg = env.airport_data.ground_net.taxiInstrStr(instr.arg, finalNonNode=instr.arg2, tts=True)
			elif instr.type == Instruction.HOLD_POSITION:
				msg = 'Hold position'
			elif instr.type == Instruction.CLEARED_APP:
				msg = 'Cleared '
				msg += 'approach' if self.status.APP_type is None else ApproachType.tts(self.status.APP_type)
				msg += ' ' + self.status.DEP_LDG_surface.readOut(tts=True)
			elif instr.type == Instruction.CLEARED_TKOF:
				msg = 'Cleared for take-off %s' % self.status.DEP_LDG_surface.readOut(tts=True)
			elif instr.type == Instruction.CLEARED_LDG:
				msg = 'Clear to land ' + self.status.DEP_LDG_surface.readOut(tts=True)
			elif instr.type == Instruction.DEP_CLEARANCE:
				msg = instr.arg
			elif instr.type == Instruction.SAY_INTENTIONS:
				if self.wantsToPark():
					msg = 'Park at \\SPELL_ALPHANUMS{%s}' % self.goal
				elif self.isInboundGoal():
					msg = '%s approach' % ApproachType.tts(self.goal)
					if self.status.DEP_LDG_surface is not None:
						msg += ', expecting %s' % self.status.DEP_LDG_surface.readOut(tts=True)
				elif self.isOutboundGoal():
					msg = 'Departing to \\SPELL_ALPHANUMS{%s}' % self.goal[2]
					if self.goal[0] is not None:
						msg += ' via \\NAVPOINT{%s}' % self.goal[0].code
					if self.goal[1] is not None:
						msg += ', cruise \\FL_ALT{%s}' % self.goal[1]
				elif isinstance(self.goal, Airfield): # Transiting with destination
					msg = 'En-route to \\SPELL_ALPHANUMS{%s}' % self.goal
				else: # teacher aircraft with no set goal
					msg = 'Following teacher instructions'
					if self.status.DEP_LDG_surface is not None:
						msg += ', expecting %s' % self.status.DEP_LDG_surface.readOut(tts=True)
			else:
				msg = 'Please report: undefined read-back message'
			lst.append(msg)
		## Now SAY IT!
		self.say(', '.join(lst), True)
	
	
	## SNAPSHOTS
	@staticmethod
	def fromStatusSnapshot(snapshot):
		cs, t, params, status, goal, spawned, frozen, instr = snapshot
		acft = ControlledAiAircraft(cs, t, params.dup(), status.dup(), goal)
		acft.instructions = [i.dup() for i in instr]
		acft.spawned = spawned
		acft.frozen = frozen
		return acft
	
	def statusSnapshot(self):
		return self.identifier, self.aircraft_type, self.params.dup(), self.params.dup(), self.goal, self.spawned, self.frozen, [i.dup() for i in self.instructions]
