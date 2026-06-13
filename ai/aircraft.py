
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

import re
from math import sqrt

from base.acft import Aircraft, Xpdr
from base.db import all_airline_codes, stall_speed
from base.params import PressureAlt, Speed
from base.util import rounded, bounded, linear
from base.weather import stdTempC
from ext.fgfs import FGFS_model_liveries, FGFS_model_position

from ext.fgms import mk_fgms_position_packet, FGMS_prop_code_by_name, FGMS_prop_XPDR_capability, FGMS_prop_XPDR_code, FGMS_prop_XPDR_ident, \
		FGMS_prop_XPDR_alt, FGMS_prop_XPDR_gnd, FGMS_prop_XPDR_ias, FGMS_prop_XPDR_mach, FGMS_prop_helo_main_rotor, FGMS_prop_helo_tail_rotor

from session.config import settings


# ---------- Constants ----------

RPM_low = 100
RPM_high = 1000
turn_roll_thr = 3 # degrees / s
right_turn_roll = 12 # degrees roll
pitch_factor = 30 / 3000 # degrees / (ft/min)
final_pitch = 2 # degrees
nose_lift_off_pitch = 4 # degrees
nose_lift_off_speed_factor = .9 # mult. stall speed
helos_max_pitch = 10
helos_max_pitch_ias = Speed(60)
runway_excursion_roll = 2 # degrees
runway_excursion_pitch = -3 # degrees
gear_compression_low = 0
gear_compression_high = .5
gear_out_dist = 6 # NM

# commercial callsign regexp groups: 1=airline code; 2=flight number
commercial_callsign_regexp = re.compile(r'([0-9A-Z]{1,2}[A-Z])(\d{4})')

# for speed of sound
ratio_of_specific_heats = 1.4 # for air
gas_constant = 287.053 # J/kg/K for air

# -------------------------------


FGMS_prop_livery_file = FGMS_prop_code_by_name('sim/model/livery/file')
FGMS_props_gear_position = [FGMS_prop_code_by_name('gear/gear[%d]/position-norm' % i) for i in range(5)]
FGMS_props_gear_compression = [FGMS_prop_code_by_name('gear/gear[%d]/compression-norm' % i) for i in range(5)]
FGMS_props_engine_RPM = [FGMS_prop_code_by_name('engines/engine[%d]/rpm' % i) for i in range(4)]






class AbstractAiAcft(Aircraft):
	"""
	This class represents an abstract class for an AI aircraft.
	Derived classes should reimplement the "doTick" method, called on every "tickOnce" (unless ACFT is frozen),
	after "self.tick_interval" is updated with a duration. It should perform the horizontal/vertical displacements, etc.
	of the past tick_interval duration.
	"""
	
	def __init__(self, callsign, acft_type, init_params, init_status):
		Aircraft.__init__(self, callsign, acft_type, init_params.position, init_params.realAltitude())
		match = commercial_callsign_regexp.match(callsign)
		if match and match.group(1) in all_airline_codes():
			self.airline = match.group(1)
		else:
			self.airline = None
		self.params = init_params
		self.status = init_status
		self.mode_S_squats = True
		self.tick_interval = None
		self.hdg_tick_diff = 0
		self.alt_tick_diff = 0
		self.released = False


	## TICKING
	
	def doTick(self):
		raise NotImplementedError('AbstractAiAcft.doTick')
	
	def tickOnce(self):
		if not self.frozen:
			self.tick_interval = settings.session_manager.clockTime() - self.lastLiveUpdateTime()
			hdg_before_tick = self.params.heading
			alt_before_tick = self.params.altitude
			self.doTick()
			self.hdg_tick_diff = self.params.heading.diff(hdg_before_tick)
			self.alt_tick_diff = self.params.altitude.diff(alt_before_tick)
		self.updateLiveStatus(self.params.position, self.params.realAltitude(), self.xpdrData())


	## FLIGHT PARAMETER LOOK-UP
	
	def xpdrGndBit(self):
		return self.params.XPDR_mode == 'S' and self.mode_S_squats and self.status.snapped_GND

	def machNumber(self): # the sqrt is the speed of sound in air at estimated temperature
		alt = PressureAlt(self.params.altitude.ft1013())
		return self.params.ias.ias2tas(alt).mps() / sqrt(ratio_of_specific_heats * gas_constant * (stdTempC(alt) + 273.15))
	
	def xpdrData(self):
		res = {}
		if self.params.XPDR_mode != '0':
			res[Xpdr.CODE] = self.params.XPDR_code
			res[Xpdr.IDENT] = self.params.XPDR_idents
		if self.params.XPDR_mode not in '0A':
			res[Xpdr.ALT] = PressureAlt(rounded(self.params.altitude.ft1013(), step=(100 if self.params.XPDR_mode == 'C' else 10)))
			if self.params.XPDR_mode != 'C':
				res[Xpdr.CALLSIGN] = self.identifier
				res[Xpdr.ACFT] = self.aircraft_type
				res[Xpdr.GND] = self.xpdrGndBit()
				res[Xpdr.IAS] = self.params.ias
				res[Xpdr.MACH] = self.machNumber()
		return res


	## FGMS PACKET
	
	def fgmsPositionPacket(self):
		if not self.status.snapped_GND and not self.status.snapped_lined_up and self.hdg_tick_diff != 0:
			deg_roll = (1 if self.hdg_tick_diff > 0 else -1) * right_turn_roll
		elif self.status.rolling_TKOF_LDG and not self.status.snapped_lined_up: # skidding off RWY
			deg_roll = runway_excursion_roll
		else:
			deg_roll = 0
		if self.isHelo():
			deg_pitch = bounded(0, linear(0, 0, helos_max_pitch_ias.kt(), helos_max_pitch, self.params.ias.kt()), helos_max_pitch)
		else: # consider fixed wing ACFT
			if not self.status.snapped_GND and not self.status.snapped_lined_up:
				deg_pitch = final_pitch
			elif self.status.rolling_TKOF_LDG and self.status.ready_for_DEP \
					and self.params.ias.diff((stall_speed(self.aircraft_type) * nose_lift_off_speed_factor).tas2ias(self.params.altitude)) > 0:
				deg_pitch = nose_lift_off_pitch
			elif self.status.rolling_TKOF_LDG and not self.status.snapped_lined_up and self.params.ias.kt() < 1e-5: # crashed off RWY
				deg_pitch = runway_excursion_pitch
			elif self.status.snapped_GND or self.tick_interval is None: # tick interval None when started frozen (never ticked once)
				deg_pitch = 0
			else:
				deg_pitch = pitch_factor * self.alt_tick_diff * 60 / self.tick_interval.total_seconds()
		# Build property dictionary...
		pdct = {}
		if self.airline is not None:
			try:
				pdct[FGMS_prop_livery_file] = FGFS_model_liveries[self.aircraft_type][self.airline]
			except KeyError:
				pass
		# XPDR prop's
		pdct[FGMS_prop_XPDR_capability] = 1 if self.params.XPDR_mode in '0AC' else 2
		if self.params.XPDR_mode != '0':
			pdct[FGMS_prop_XPDR_code] = int('%o' % self.params.XPDR_code, base=10)
			if self.params.XPDR_mode != 'A':
				pdct[FGMS_prop_XPDR_alt] = int(self.params.altitude.ft1013())
			if self.params.XPDR_mode == 'S':
				pdct[FGMS_prop_XPDR_gnd] = self.xpdrGndBit()
				pdct[FGMS_prop_XPDR_ias] = int(self.params.ias.kt())
				pdct[FGMS_prop_XPDR_mach] = self.machNumber()
			pdct[FGMS_prop_XPDR_ident] = self.params.XPDR_idents
		# engines/propellers
		if self.isHelo(): # TODO check assigned values
			pdct[FGMS_prop_helo_main_rotor] = RPM_high
			pdct[FGMS_prop_helo_tail_rotor] = RPM_high
		else:
			for prop in FGMS_props_engine_RPM:
				pdct[prop] = RPM_low if self.status.snapped_GND and self.params.ias.kt() < 1 else RPM_high
		# landing gear
		for prop in FGMS_props_gear_position: # FLOAT: 0=retracted; 1=extended
			pdct[prop] = float(self.status.snapped_GND or self.status.snapped_lined_up and self.params.position.distanceTo(self.status.DEP_LDG_surface.touchDownPoint()) < gear_out_dist)
		for prop in FGMS_props_gear_compression: # FLOAT: 0=free; 1=compressed
			pdct[prop] = gear_compression_high if self.status.snapped_GND else gear_compression_low
		# finished
		model, coords, amsl = FGFS_model_position(self.aircraft_type, self.liveCoords(), self.liveRealAlt(), self.params.heading)
		return mk_fgms_position_packet(self.identifier, model, coords, amsl,
				hdg=self.params.heading.trueAngle(), pitch=deg_pitch, roll=deg_roll, properties=pdct)
