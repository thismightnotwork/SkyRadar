
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

from base.db import acft_cat
from base.conflict import Conflict
from base.params import Speed, PressureAlt

from session.config import settings
from session.env import env


# ---------- Constants ----------

snapshot_diff_time = 6 # seconds (how long to look back in history for snapshot diffs)
min_taxiing_speed = Speed(5)
max_ground_height = 100 # ft

# -------------------------------



class Xpdr:
	keys = CODE, IDENT, ALT, CALLSIGN, ACFT, GND, IAS, MACH = range(8)

	@staticmethod
	def encodeData(key, val):
		if val is None:
			return '-'
		if key == Xpdr.CODE: # int to show as octal
			return '%04o' % val
		elif key == Xpdr.IDENT or key == Xpdr.GND: # bool
			return '01'[val]
		elif key == Xpdr.ALT: # PressureAlt
			return str(val.ft1013())
		elif key == Xpdr.CALLSIGN or key == Xpdr.ACFT: # str
			return val.replace(' ', '-') # lossy conv, but unlikely a problem and avoids problem with space-sep'ed fields
		elif key == Xpdr.IAS: # Speed
			return str(val.kt())
		elif key == Xpdr.MACH: # float
			return str(val)
		raise ValueError(key)

	@staticmethod
	def decodeData(key, s):
		if s == '-':
			return None
		if key == Xpdr.CODE: # int shown as octal
			return int(s, base=8)
		elif key == Xpdr.IDENT or key == Xpdr.GND: # bool
			return s == '1'
		elif key == Xpdr.ALT: # PressureAlt
			return PressureAlt(float(s))
		elif key == Xpdr.CALLSIGN or key == Xpdr.ACFT: # str
			return s
		elif key == Xpdr.IAS: # Speed
			return Speed(float(s))
		elif key == Xpdr.MACH: # float
			return float(s)
		raise ValueError(key)



class RadarSnapshot:
	def __init__(self, time_stamp, coords, xpdr_data_dict):
		# Obligatory constructor data
		self.time_stamp = time_stamp
		self.coords = coords
		# XPDR data
		self.xpdrData = xpdr_data_dict # Xpdr key -> value (dict does not have to cover all of Xpdr.keys)
		# Inferred values, only filled when appending to a history
		self.heading = None
		self.groundSpeed = None
		self.verticalSpeed = None





class Aircraft:
	"""
	This class represents a live aircraft, whether visible or invisible,
	with a live status (position + XPDR data) and a radar snapshot history.
	"""
	def __init__(self, identifier, acft_type, init_position, init_real_alt):
		self.identifier = identifier # assumed unique
		self.aircraft_type = acft_type
		# "REAL TIME" VALUES
		self.live_update_time = settings.session_manager.clockTime()
		self.live_position = init_position, init_real_alt # EarthCoords, real alt. AMSL
		self.live_XPDR_data = {} # sqkey -> value mappings available from live update
		# UPDATED DATA
		self.radar_history = [RadarSnapshot(self.live_update_time, init_position, {})] # snapshot history; list must not be empty
		self.conflict = Conflict.NO_CONFLICT
		# USER OPTIONS
		self.individual_cheat = False
		self.flagged = False
		self.ignored = False
		# OTHER FLAGS/ATTRIBUTES
		self.spawned = True # for teacher
		self.frozen = False # for solo/student/teacher
		self.radio_ptt = False
		self.tx_freq = None # frequency on which ACFT is transmitting (useful for live RDF updates)
	
	def __str__(self):
		return self.identifier

	def isHelo(self):
		return acft_cat(self.aircraft_type) == 'helos'
	
	def setIndividualCheat(self, b):
		self.individual_cheat = b

	##
	##  LIVE DATA QUERY AND UPDATE
	##
	def liveCoords(self):
		return self.live_position[0]
	
	def liveRealAlt(self):
		return self.live_position[1]
	
	def lastLiveUpdateTime(self):
		return self.live_update_time
	
	def isRadarVisible(self):
		"""
		a radar can draw a spot (possibly helped by a cheat)
		"""
		if self.individual_cheat:
			return True
		visible = settings.radar_cheat \
				or settings.primary_radar_active \
				or settings.SSR_mode_capability != '0' and self.live_XPDR_data != {} # radar contact
		visible &= settings.radar_signal_floor_level == 0 \
				or settings.radar_cheat \
				or self.liveRealAlt() >= settings.radar_signal_floor_level # vert. range
		visible &= env.pointInRadarRange(self.liveCoords()) # horiz. range
		return visible
	
	def updateLiveStatus(self, pos, real_alt, xpdr_data):
		self.live_update_time = settings.session_manager.clockTime()
		self.live_position = pos, real_alt
		self.live_XPDR_data = xpdr_data
		if self.radio_ptt:
			env.rdf.radioSignal(self.tx_freq, env.rdf.antennaPos()[0].headingTo(pos))
	
	def setPtt(self, freq=None):
		self.radio_ptt = True
		self.tx_freq = freq
		env.rdf.radioSignal(freq, env.rdf.antennaPos()[0].headingTo(self.liveCoords()))

	def resetPtt(self):
		self.radio_ptt = False
		env.rdf.endOfSignal(self.tx_freq)

	##
	##  RADAR SNAPSHOTS
	##
	def appendToRadarHistory(self, snapshot):
		assert all(x is None for x in [snapshot.heading, snapshot.groundSpeed, snapshot.verticalSpeed]), 'inferred values should still be blank'
		prev = self.radar_history[-1] # always exists
		if snapshot.time_stamp <= prev.time_stamp:
			print('WARNING: Should only be appending to history, not changing the past. Removing prior history.')
			self.radar_history = [snapshot]
			return
		# Fill inferred values
		if self.frozen: # copy from previous snapshot
			snapshot.heading = prev.heading
			snapshot.groundSpeed = prev.groundSpeed
			snapshot.verticalSpeed = prev.verticalSpeed
		else: # compute values from change between snapshots
			# Search history for best snapshot to use for diff
			diff_seconds = (snapshot.time_stamp - prev.time_stamp).total_seconds()
			i = 1 # index of currently selected prev
			while i < len(self.radar_history) and diff_seconds < snapshot_diff_time:
				i += 1
				prev = self.radar_history[-i]
				diff_seconds = (snapshot.time_stamp - prev.time_stamp).total_seconds()
			# Fill "inferred" snapshot diffs
			# TODO check if test was really needed for ground speed and heading: if prev.coords is not None and snapshot.coords is not None:
			# ground speed
			snapshot.groundSpeed = Speed(prev.coords.distanceTo(snapshot.coords) * 3600 / diff_seconds)
			# heading
			if snapshot.groundSpeed.diff(min_taxiing_speed) > 0: # acft moving across the ground
				try:
					snapshot.heading = snapshot.coords.headingFrom(prev.coords)
				except ValueError:
					snapshot.heading = prev.heading # stopped: keep prev. hdg
			else:
				snapshot.heading = prev.heading
			# vertical speed
			prev_alt = prev.xpdrData.get(Xpdr.ALT)
			this_alt = snapshot.xpdrData.get(Xpdr.ALT)
			if prev_alt is not None and this_alt is not None:
				snapshot.verticalSpeed = (this_alt.diff(prev_alt)) * 60 / diff_seconds
		# Append snapshot to history
		self.radar_history.append(snapshot)
	
	def saveRadarSnapshot(self):
		"""
		Saves a snapshot from current live status.
		"""
		if self.radar_history[-1].time_stamp == self.live_update_time: # not called in playback so no future in history list
			return # Do not save values again: live status was not updated since last snapshot
		xpdr = self.live_XPDR_data.copy()
		if settings.radar_cheat or self.individual_cheat:
			# We try to compensate, but cannot always win so None values are possible.
			# Plus: CODE, IDENT and GND have no useful compensation.
			if Xpdr.ALT not in xpdr:
				stdpa = PressureAlt.fromAMSL(self.liveRealAlt(), env.QNH())
				xpdr[Xpdr.ALT] = PressureAlt(stdpa.ft1013())
			if Xpdr.CALLSIGN not in xpdr:
				xpdr[Xpdr.CALLSIGN] = self.identifier
			if Xpdr.ACFT not in xpdr:
				xpdr[Xpdr.ACFT] = self.aircraft_type
		else: # contact is not cheated
			if settings.SSR_mode_capability == '0': # no SSR so no XPDR data can be snapshot
				xpdr.clear()
			else: # SSR on; check against A/C/S capability
				if settings.SSR_mode_capability == 'A': # radar does not have the capability to pick up altitude
					if Xpdr.ALT in xpdr:
						del xpdr[Xpdr.ALT]
				if settings.SSR_mode_capability != 'S': # radar does not have mode S interrogation capability
					for k in (Xpdr.CALLSIGN, Xpdr.ACFT, Xpdr.IAS, Xpdr.MACH, Xpdr.GND):
						if k in xpdr:
							del xpdr[k]
		snapshot = RadarSnapshot(self.live_update_time, self.liveCoords(), xpdr)
		self.appendToRadarHistory(snapshot)
		settings.session_recorder.proposeAcftBlip(self.identifier, snapshot)

	def lastSnapshot(self):
		"""
		Returns the latest snapshot in saved radar history before current session time (inclusive).
		Raises StopIteration if none.
		"""
		t = settings.session_manager.clockTime()
		return next(snap for snap in reversed(self.radar_history) if snap.time_stamp <= t)

	def positionHistory(self, hist, before):
		"""
		Returns the history of snapshot coordinates for the given timedelta "hist" until "before", in chronological order.
		"""
		try:
			i_start = next(i for i, snap in enumerate(self.radar_history) if before - snap.time_stamp <= hist)
			if self.radar_history[-1].time_stamp <= before:
				i_off_end = len(self.radar_history)
			else: # looking for shorter than whole tail of the history; must slice list
				i_off_end = next(-i for i, snap in enumerate(reversed(self.radar_history)) if snap.time_stamp <= before)
			return [snap.coords for snap in self.radar_history[i_start:i_off_end]]
		except StopIteration: # if no live update in the time frame requested (can happen if app freezes for a while)
			return []

	## Reading latest radar snapshot
	def coords(self):
		return self.lastSnapshot().coords
	
	def xpdrOn(self):
		return self.lastSnapshot().xpdrData != {}
	
	def xpdrCode(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.CODE)
	
	def xpdrIdent(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.IDENT)
	
	def xpdrAlt(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.ALT)
	
	def xpdrCallsign(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.CALLSIGN)
	
	def xpdrAcftType(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.ACFT)
	
	def xpdrIAS(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.IAS)
	
	def xpdrGND(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.GND)

	def xpdrMachNumber(self):
		return self.lastSnapshot().xpdrData.get(Xpdr.MACH)
	
	## Inferred values
	
	def heading(self):
		return self.lastSnapshot().heading
	
	def groundSpeed(self):
		return self.lastSnapshot().groundSpeed
	
	def verticalSpeed(self):
		return self.lastSnapshot().verticalSpeed
	
	def considerOnGround(self):
		if self.xpdrGND():
			return True
		else:
			alt = self.xpdrAlt()
			return alt is not None and alt.ft1013() - env.elevation(self.coords()) <= max_ground_height
	
	def IAS(self):
		"""
		Get real IAS if squawked, or estimate. None result is possible if missing alt or ground speed.
		When estimating: TAS = ground speed (no wind correction because wind is not known here).
		"""
		squawked = self.xpdrIAS()
		if squawked is not None:
			return squawked
		# else: estimate...
		gs = self.lastSnapshot().groundSpeed
		if gs is not None:
			alt = self.xpdrAlt()
			if alt is not None:
				return gs.tas2ias(alt)
		return None
