
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

from copy import copy

from base.params import Heading, AltFlSpec, Speed
from base.util import some, upper_1st

from session.env import env


# ---------- Constants ----------

taxi_instr_node_max_dist = .1 # NM

# -------------------------------

class ApproachType:
	types = ILS, VISUAL, STRAIGHT_IN = range(3) # ILS/VISUAL for RWY final legs; STRAIGHT_IN for helos, to helipad or RWY numbers

	@staticmethod
	def toStr(t):
		return {ApproachType.ILS: 'ILS', ApproachType.VISUAL: 'visual', ApproachType.STRAIGHT_IN: 'straight-in'}[t]

	@staticmethod
	def tts(t):
		return {ApproachType.ILS: '\\SPLIT_CHARS{ILS}', ApproachType.VISUAL: 'visual', ApproachType.STRAIGHT_IN: 'straight-in'}[t]


# Instructions and arg types
#   - "OPT" means arg can be None
#   - "*" means instr. can be given on ground as initial after-DEP vector
#
# ==== instr ==== | ========== instr.arg ========== | ========== instr.arg2 ========== | ==== instr.arg resolves to ... on ingestInstruction ====
# VECTOR_HDG *    |  Heading                        |  [OPT] bool turn direction (True for right)
# VECTOR_ALT *    |  AltFlSpec
# VECTOR_SPD      |  Speed
# VECTOR_DCT *    |  str navpoint name                                                 |  'navpoint' (Navpoint)
# CANCEL_SPD      |
# FOLLOW_ROUTE *  |  str route (waypoints and leg specs)                               |  'route' (Route)
# HOLD_AT_FIX     |  str navpoint name                                                 |  'navpoint' (Navpoint)
# SQUAWK          |  int (octal)
# HAND_OVER       |  str next ATC callsign          |  [OPT] str frequency
# CANCEL_APP      |  
# INTERCEPT_NAV * |  str navpoint name              |  Heading radial/bearing          |  'navpoint' (Navpoint)
# INTERCEPT_LOC   |  [OPT] RWY name
# TAXI            |  GND net node sequence (list)   |  [OPT] off-net EarthCoords or str PKG position to taxi to after the arg route
# HOLD_POSITION   |  
# DEP_CLEARANCE   |  str text clearance
# EXPECT_SFC      |  str RWY/pad name (TKOF or LDG) |  [OPT] ApproachType enum if LDG  |  'sfc' (DepLdgSurface)
# LINE_UP         |  [OPT] TKOF RWY/pad name
# CLEARED_TKOF    |  [OPT] TKOF RWY/pad name
# CLEARED_APP     |  [OPT] LDG RWY/pad name         |  [OPT] ApproachType enum
# CLEARED_LDG     |  [OPT] LDG RWY/pad name
# SAY_INTENTIONS  |  


class Instruction:
	enum = VECTOR_HDG, VECTOR_ALT, VECTOR_SPD, VECTOR_DCT, CANCEL_SPD, FOLLOW_ROUTE, HOLD_AT_FIX, \
		HOLD_POSITION, SQUAWK, HAND_OVER, TAXI, CANCEL_APP, LINE_UP, INTERCEPT_NAV, INTERCEPT_LOC, \
		EXPECT_SFC, DEP_CLEARANCE, CLEARED_APP, CLEARED_TKOF, CLEARED_LDG, SAY_INTENTIONS = range(21)

	@staticmethod
	def type2str(t):
		return {
				Instruction.VECTOR_HDG: 'VECTOR_HDG',
				Instruction.VECTOR_ALT: 'VECTOR_ALT',
				Instruction.VECTOR_SPD: 'VECTOR_SPD',
				Instruction.VECTOR_DCT: 'VECTOR_DCT',
				Instruction.CANCEL_SPD: 'CANCEL_SPD',
				Instruction.FOLLOW_ROUTE: 'FOLLOW_ROUTE',
				Instruction.HOLD_AT_FIX: 'HOLD_AT_FIX',
				Instruction.HOLD_POSITION: 'HOLD_POSITION',
				Instruction.SQUAWK: 'SQUAWK',
				Instruction.HAND_OVER: 'HAND_OVER',
				Instruction.TAXI: 'TAXI',
				Instruction.CANCEL_APP: 'CANCEL_APP',
				Instruction.LINE_UP: 'LINE_UP',
				Instruction.INTERCEPT_NAV: 'INTERCEPT_NAV',
				Instruction.INTERCEPT_LOC: 'INTERCEPT_LOC',
				Instruction.EXPECT_SFC: 'EXPECT_SFC',
				Instruction.DEP_CLEARANCE: 'DEP_CLEARANCE',
				Instruction.CLEARED_APP: 'CLEARED_APP',
				Instruction.CLEARED_TKOF: 'CLEARED_TKOF',
				Instruction.CLEARED_LDG: 'CLEARED_LDG',
				Instruction.SAY_INTENTIONS: 'SAY_INTENTIONS'
			}[t]
	
	class Error(Exception):
		pass
	
	def __init__(self, init_type, arg=None, arg2=None):
		self.type = init_type
		self.arg = arg
		self.arg2 = arg2
		self.resolved_arg = None # Navpoint, Route or DepLdgSurface (see doc table up this file)
	
	def __str__(self):
		return '%s:%s:%s' % (Instruction.type2str(self.type), self.arg, self.arg2)
	
	def dup(self):
		res = Instruction(self.type, arg=copy(self.arg), arg2=copy(self.arg2))
		res.resolved_arg = self.resolved_arg
		return res

	def readOutStr(self, acft):
		"""
		Non-TTS string to read out to paramter ACFT.
		Parameter "acft" can be None, then a more generic read-out will be suggested.
		"""
		if self.type == Instruction.VECTOR_HDG:
			if acft is not None and acft.considerOnGround():
				fmt = 'Initial heading %s'
			elif self.arg2 is None:
				fmt = 'Fly heading %s'
			else: # right/left turn specified
				fmt = 'Turn right heading %s' if self.arg2 else 'Turn left heading %s'
			return fmt % self.arg.read()
		elif self.type == Instruction.VECTOR_ALT:
			verb_prefix = 'Fly'
			qnh_suffix = ''
			if acft is not None:
				if acft.considerOnGround():
					verb_prefix = 'Initial climb' # Override
				else:
					c_alt = acft.xpdrAlt()
					if c_alt is not None:
						try:
							v_alt = env.pressureAlt(self.arg)
						except ValueError:
							pass
						else:
							if c_alt.diff(v_alt) < 0:
								verb_prefix = 'Climb' # Override
							else:
								verb_prefix = 'Descend' # Override
								qnh = env.QNH(noneSafe=False)
								if qnh is not None and v_alt.FL() < env.transitionLevel() <= c_alt.FL():
									qnh_suffix = ', QNH %d' % qnh # Override
			return '%s %s%s' % (verb_prefix, self.arg.toStr(), qnh_suffix)
		elif self.type == Instruction.VECTOR_SPD:
			return 'Speed %s' % self.arg
		elif self.type == Instruction.VECTOR_DCT:
			if acft is not None and acft.considerOnGround():
				return 'Direct %s after departure' % self.arg
			else:
				return 'Proceed direct %s' % self.arg
		elif self.type == Instruction.CANCEL_SPD:
			return 'Speed your discretion'
		elif self.type == Instruction.FOLLOW_ROUTE:
			if acft is None or acft.considerOnGround():
				return 'Cleared route %s' % self.arg
			else:
				return 'Proceed %s' % self.arg
		elif self.type == Instruction.HOLD_AT_FIX:
			return 'Hold at %s, as published' % self.arg
		elif self.type == Instruction.SQUAWK:
			return 'Squawk %04o' % self.arg
		elif self.type == Instruction.HAND_OVER:
			imsg = 'Contact ' + self.arg
			if self.arg2 is not None:
				imsg += ' on ' + self.arg2
			return imsg + ', good bye.'
		elif self.type == Instruction.CANCEL_APP:
			return 'Cancel approach, stand by for vectors'
		elif self.type == Instruction.LINE_UP:
			if self.arg is None:
				return 'Line up and wait'
			else:
				return '%s, line up and wait' % read_out_DEP_LDG_surface_name(self.arg)
		elif self.type == Instruction.INTERCEPT_NAV:
			imsg = 'Intercept %s from/to %s' % (self.arg2.read(), self.arg)
			if acft is not None and acft.considerOnGround():
				imsg += ' after departure'
			return imsg
		elif self.type == Instruction.INTERCEPT_LOC:
			msg = 'Intercept localiser'
			if self.arg is not None:
				msg += ' for ' + read_out_DEP_LDG_surface_name(self.arg)
			return msg
		elif self.type == Instruction.EXPECT_SFC:
			if self.arg2 is None:
				return 'Expect ' + read_out_DEP_LDG_surface_name(self.arg)
			else:
				return 'Expect %s approach for %s' % (ApproachType.toStr(self.arg2), read_out_DEP_LDG_surface_name(self.arg))
		elif self.type == Instruction.TAXI:
			if env.airport_data is None:
				return 'Taxi... without an airport!'
			else:
				return env.airport_data.ground_net.taxiInstrStr(self.arg, finalNonNode=self.arg2)
		elif self.type == Instruction.HOLD_POSITION:
			return 'Hold position'
		elif self.type == Instruction.CLEARED_APP:
			msg = 'Cleared approach' if self.arg2 is None else 'Cleared %s approach' % ApproachType.toStr(self.arg2)
			if self.arg is not None:
				msg += ' ' + read_out_DEP_LDG_surface_name(self.arg)
			return msg
		elif self.type == Instruction.CLEARED_TKOF:
			if self.arg is None:
				return 'Cleared for take-off'
			else:
				return upper_1st(read_out_DEP_LDG_surface_name(self.arg)) + ', cleared for take-off'
		elif self.type == Instruction.CLEARED_LDG:
			msg = 'Cleared to land'
			if self.arg is not None:
				msg += ' ' + read_out_DEP_LDG_surface_name(self.arg)
			w = env.primaryWeather()
			if w is not None:
				msg += ', wind %s' % w.readWind()
			return msg
		elif self.type == Instruction.SAY_INTENTIONS:
			return 'Say intentions?'
		elif self.type == Instruction.DEP_CLEARANCE:
			return self.arg
	
	
	## CPDLC MESSAGE ELEMENTS

	@staticmethod
	def fromCpdlcMsgElement(msg_element):
		id_split = msg_element.split(' ', maxsplit=1)
		elt_id = id_split[0]
		argstr = '' if len(id_split) < 2 else id_split[1]
		# UPLINK elements, used by solo aircraft, and teacher receiving student's uplinks
		if elt_id == 'LATU-11' or elt_id == 'LATU-16': # resp. "turn left/right heading" and "fly heading"
			return Instruction(Instruction.VECTOR_HDG, arg=Heading(int(argstr[-3:]), False),
												arg2=(argstr.startswith('R') if elt_id[-1] == '1' else None))
		elif elt_id == 'LVLU-5' or elt_id == 'LVLU-6' or elt_id == 'LVLU-9':
			return Instruction(Instruction.VECTOR_ALT, arg=AltFlSpec.fromStr(argstr))
		elif elt_id == 'SPDU-9' or elt_id == 'SPDU-11':
			return Instruction(Instruction.VECTOR_SPD, arg=Speed(int(argstr)))
		elif elt_id == 'RTEU-2':
			return Instruction(Instruction.VECTOR_DCT, arg=argstr)
		elif elt_id == 'SPDU-13':
			return Instruction(Instruction.CANCEL_SPD)
		elif elt_id == 'RTEU-7':
			return Instruction(Instruction.FOLLOW_ROUTE, arg=argstr)
		elif elt_id == 'RTEU-12':
			return Instruction(Instruction.HOLD_AT_FIX, arg=argstr)
		elif elt_id == 'ADVU-9':
			return Instruction(Instruction.SQUAWK, arg=int(argstr, base=8))
		elif elt_id == 'COMU-1':
			split = argstr.split(' ', maxsplit=1)
			if split[0] == '':
				raise ValueError('Empty callsign after COMU-1')
			frq = None if len(split) < 2 or split[1] == '' else split[1]
			return Instruction(Instruction.HAND_OVER, arg=split[0], arg2=frq)
		elif elt_id == 'RTEU-1':
			return Instruction(Instruction.DEP_CLEARANCE, arg=argstr)
		# DOWNLINK elements
		if elt_id == 'RTED-6':
			return Instruction(Instruction.VECTOR_HDG, arg=Heading(int(argstr), False))
		elif elt_id == 'LVLD-1':
			return Instruction(Instruction.VECTOR_ALT, arg=AltFlSpec.fromStr(argstr))
		elif elt_id == 'SPDD-1':
			return Instruction(Instruction.VECTOR_SPD, arg=Speed(int(argstr)))
		elif elt_id == 'RTED-1':
			return Instruction(Instruction.VECTOR_DCT, arg=argstr)
		elif elt_id == 'RTED-3':
			return Instruction(Instruction.FOLLOW_ROUTE, arg=argstr)
		raise ValueError('Unhandled message element identifier "%s"' % elt_id)
	
	def toCpdlcUplinkMsgElt(self, acft): # PoV = ACFT requesting a specific instruction (useful for vectors for example)
		if self.type == Instruction.VECTOR_HDG:
			if self.arg2 is None:
				return 'LATU-16 %s' % self.arg.read() # fly heading
			else: # turn direction specified in instruction
				return 'LATU-11 %s %s' % (('RIGHT' if self.arg2 else 'LEFT'), self.arg.read())
		elif self.type == Instruction.VECTOR_ALT: # NOTE might be used for "initial climb"
			argstr = self.arg.toStr(unit=False)
			alt = None if acft is None else acft.xpdrAlt()
			if alt is not None:
				try:
					tgt = env.pressureAlt(self.arg)
					if alt.diff(tgt, tolerance=100) < 0:
						return 'LVLU-6 %s' % argstr # climb
					elif alt.diff(tgt, tolerance=100) > 0:
						return 'LVLU-9 %s' % argstr # descend
				except ValueError: # syntax error in arg reading
					pass
			return 'LVLU-5 %s' % argstr # maintain
		elif self.type == Instruction.VECTOR_SPD:
			ias = None if acft is None else acft.IAS()
			if ias is None or ias.diff(self.arg) < 0: # capture None case because no escape CPDLC option
				return 'SPDU-9 %03d' % self.arg.kt() # increase speed
			else:
				return 'SPDU-11 %03d' % self.arg.kt() # reduce speed
		elif self.type == Instruction.VECTOR_DCT:
			return 'RTEU-2 %s' % self.arg
		elif self.type == Instruction.CANCEL_SPD:
			return 'SPDU-13'
		elif self.type == Instruction.FOLLOW_ROUTE:
			return 'RTEU-7 %s' % self.arg
		elif self.type == Instruction.HOLD_AT_FIX:
			return 'RTEU-12 %s' % self.arg
		elif self.type == Instruction.SQUAWK:
			return 'ADVU-9 %04o' % self.arg
		elif self.type == Instruction.HAND_OVER:
			return 'COMU-1 %s %s' % (self.arg, some(self.arg2, ''))
		elif self.type == Instruction.DEP_CLEARANCE:
			return 'RTEU-1 %s' % self.arg.upper()
		else: # fall back on a free text message
			if self.type == Instruction.SAY_INTENTIONS:
				return 'TXTU-1 ADVISE INTENTIONS' # resp. attr. "R"
			else:
				return 'TXTU-4 %s' % self.readOutStr(acft).upper() # resp. attr. "W/U"
	
	def toCpdlcDownlinkRequestElt(self, acft):
		if self.type == Instruction.VECTOR_HDG:
			return 'RTED-6 %s' % self.arg.read()
		elif self.type == Instruction.VECTOR_ALT: # NOTE might be used for "initial climb"
			return 'LVLD-1 %s' % self.arg.toStr(unit=False)
		elif self.type == Instruction.VECTOR_SPD:
			return 'SPDD-1 %s' % self.arg
		elif self.type == Instruction.VECTOR_DCT:
			return 'RTED-1 %s' % self.arg
		elif self.type == Instruction.FOLLOW_ROUTE:
			return 'RTED-3 %s' % self.arg
		else:
			raise ValueError('Downlink request not supported for this instruction.')



def read_out_DEP_LDG_surface_name(key):
	if env.airport_data is not None and any(rwy.name == key for rwy in env.airport_data.directionalRunways()):
		return 'runway %s' % key
	else:
		return key
