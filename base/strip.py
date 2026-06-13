
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

from sys import stderr

from base.fpl import FPL, detail2str, str2detail
from base.nav import NavpointError, world_navpoint_db
from base.params import Heading, AltFlSpec, Speed
from base.route import Route
from base.util import some

from session.config import settings


# ---------- Constants ----------

strip_mime_type = 'application/x-strip'
unfollowedRouteWarning_min_distToAD = 40

parsed_route_detail = 'parsed_route'  # Route
received_from_detail = 'fromATC'      # str (callsign)
sent_to_detail = 'toATC'              # str (callsign)
recycled_detail = 'recycled'          # bool
shelved_detail = 'shelved'            # bool
auto_printed_detail = 'auto_printed'  # bool
assigned_heading_detail = 'assHdg'    # Heading
assigned_altitude_detail = 'assAlt'   # AltFlSpec
assigned_speed_detail = 'assSpd'      # Speed
assigned_SQ_detail = 'assSQ'          # int (*OCTAL* code)
departure_clearance_detail = 'DEPclr' # str
soft_link_detail = 'softlink'         # Aircraft (the identified radar contact)
rack_detail = 'rack'                  # str (or None if strip is unracked, i.e. loose or boxed)
runway_box_detail = 'rwybox'          # int (the physical RWY index in AD output, or None if strip is not boxed)
duplicate_callsign_detail = 'dupCS'   # bool (duplicate callsign detected)
student_ok_detail = 'studentOK'       # bool (never sent to student since connection)

strip_editable_FPL_details = [FPL.CALLSIGN, FPL.ACFT_TYPE, FPL.WTC, FPL.ICAO_DEP, FPL.ICAO_ARR,
		FPL.CRUISE_ALT, FPL.TAS, FPL.FLIGHT_RULES, FPL.ROUTE, FPL.COMMENTS]
handover_details = list(FPL.details) + \
		[assigned_SQ_detail, assigned_heading_detail, assigned_altitude_detail, assigned_speed_detail, departure_clearance_detail]

# -------------------------------


class Strip:
	def __init__(self):
		self.details = {} # strip can contain any string or FPL detail key values
		self.linked_aircraft = None
		self.linked_FPL = None
	
	def __str__(self): # only for debugging
		return '[%s:%s]' % (some(self.lookup(rack_detail), ''), some(self.callsign(), ''))
	
	def _parseRoute(self):
		dep = self.lookup(FPL.ICAO_DEP, fpl=False)
		arr = self.lookup(FPL.ICAO_ARR, fpl=False)
		mid = self.lookup(FPL.ROUTE, fpl=False)
		if dep is None or arr is None:
			self.details[parsed_route_detail] = None
		else:
			try:
				self.details[parsed_route_detail] = Route(world_navpoint_db.findAirfield(dep),
						world_navpoint_db.findAirfield(arr), some(mid, ''))
			except NavpointError: # One of the end airports is missing or unrecognised
				self.details[parsed_route_detail] = None
	
	
	## ENCODE/DECODE
	
	# double-backslash separates details (top-level separator)
	# backslash+n encodes new line
	# backslash+space encodes normal backslash

	def encodeDetails(self, details):
		unescaped_details = []
		for d in details:
			v = self.lookup(d)
			if v is not None:
				try:
					if d in FPL.details:
						vstr = detail2str(d, v)
					elif d == assigned_SQ_detail: # int
						vstr = str(v)
					elif d == assigned_speed_detail: # Speed
						vstr = str(int(v.kt()))
					elif d == assigned_heading_detail: # Heading
						vstr = str(int(v.read()))
					elif d == assigned_altitude_detail: # AltFlSpec
						vstr = v.toStr(unit=False) # avoid spaces
					unescaped_details.append('%s %s' % (d, vstr)) # assumes there is no str detail key that looks like an int
				except ValueError as err:
					print('ERROR: %s' % err, file=stderr)
		return r'\\'.join(dvstr.replace('\\', r'\ ').replace('\n', r'\n') for dvstr in unescaped_details)

	@staticmethod
	def fromEncodedDetails(encoded_details):
		strip = Strip()
		for encoded_detail in encoded_details.split(r'\\'):
			unescaped = encoded_detail.replace(r'\n', '\n').replace(r'\ ', '\\')
			tokens = unescaped.split(maxsplit=1)
			if len(tokens) == 0:
				continue # Ignore empty detail sections. Normally happens only if strip has no details at all.
			try:
				try: # assumes there is no str detail key that looks like an int
					d = int(tokens[0])
				except ValueError:
					d = tokens[0]
				vstr = '' if len(tokens) < 2 else tokens[1]
				if d in FPL.details:
					v = str2detail(d, vstr) # may raise ValueError
				elif d == assigned_SQ_detail: # int
					v = int(vstr)
				elif d == assigned_speed_detail: # Speed
					v = Speed(int(vstr))
				elif d == assigned_heading_detail: # Heading
					v = Heading(int(vstr), False)
				elif d == assigned_altitude_detail: # AltFlSpec
					v = AltFlSpec.fromStr(vstr)
				strip.writeDetail(d, v)
			except ValueError as err:
				print('ERROR: %s' % err, file=stderr)
		return strip
	
	
	## ACCESS
	
	def linkedFPL(self):
		return self.linked_FPL
	
	def linkedAircraft(self):
		return self.linked_aircraft
	
	def lookup(self, key, fpl=False):
		"""
		returns the value written on the strip. If None while 'fpl' is True: look up linked flight plan
		"""
		if key in self.details: # Strip has detail of its own
			return self.details[key]
		elif fpl and key in FPL.details and self.linkedFPL() is not None:
			return self.linkedFPL()[key]
		return None
	
	def callsign(self):
		res = self.lookup(FPL.CALLSIGN, fpl=True)
		if res is None:
			acft = self.linkedAircraft()
			if acft is not None:
				return acft.xpdrCallsign()
		return res
	
	def fplConflicts(self):
		"""
		Details on linked FPL that are different from strip information if any.
		"""
		conflicts = []
		fpl = self.linkedFPL()
		if fpl is not None:
			for d in FPL.details:
				here = self.lookup(d, fpl=False)
				there = fpl[d]
				if here is not None and there is not None and here != there:
					conflicts.append(d)
		return conflicts

	def xpdrConflicts(self):
		"""
		Details among FPL.CALLSIGN, FPL.ACFT_TYPE and assigned_SQ_detail picked up
		from linked XPDR and that are different from strip information if any.
		"""
		conflicts = []
		acft = self.linkedAircraft()
		if acft is not None:
			xcs = acft.xpdrCallsign()
			scs = self.lookup(FPL.CALLSIGN, fpl=True)
			if xcs is not None and scs is not None and xcs.upper() != scs.upper():
				conflicts.append(FPL.CALLSIGN)
			xatd = acft.xpdrAcftType()
			satd = self.lookup(FPL.ACFT_TYPE, fpl=True)
			if xatd is not None and satd is not None and xatd.upper() != satd.upper():
				conflicts.append(FPL.ACFT_TYPE)
			xsq = acft.xpdrCode()
			ssq = self.lookup(assigned_SQ_detail)
			if xsq is not None and ssq is not None and xsq != ssq:
				conflicts.append(assigned_SQ_detail)
		return conflicts
	
	def vectoringConflicts(self, qnh):
		"""
		Returns a dict of (conflicting detail --> value diff) associations where conflict exceeds tolerance
		"""
		conflicts = {}
		acft = self.linkedAircraft()
		if acft is not None and not acft.considerOnGround():
			curHdg = acft.heading()
			assHdg = self.lookup(assigned_heading_detail)
			if curHdg is not None and assHdg is not None:
				diff = curHdg.diff(assHdg, settings.heading_tolerance)
				if diff != 0:
					conflicts[assigned_heading_detail] = diff
			curAlt = acft.xpdrAlt() # PressureAlt
			assAlt = self.lookup(assigned_altitude_detail) # AltFlSpec
			if curAlt is not None and assAlt is not None:
				diff = curAlt.diff(assAlt.toPressureAlt(qnh), settings.altitude_tolerance)
				if diff != 0:
					conflicts[assigned_altitude_detail] = diff
			curIAS = acft.IAS()
			assIAS = self.lookup(assigned_speed_detail)
			if curIAS is not None and assIAS is not None:
				diff = curIAS.diff(assIAS, settings.speed_tolerance)
				if diff != 0:
					conflicts[assigned_speed_detail] = diff
		return conflicts
	
	def routeConflict(self):
		acft = self.linkedAircraft()
		route = self.lookup(parsed_route_detail)
		if acft is None or route is None or self.lookup(assigned_heading_detail) is not None:
			return False
		else:
			pos = acft.coords()
			hdg = acft.heading()
			leg = route.currentLegIndex(pos)
			wp = route.waypoint(leg).coordinates
			return hdg is not None \
				and not (leg == 0 and pos.distanceTo(route.originCoords()) < unfollowedRouteWarning_min_distToAD) \
				and not (leg == route.legCount() - 1 and pos.distanceTo(wp) < unfollowedRouteWarning_min_distToAD) \
				and hdg.diff(pos.headingTo(wp), settings.heading_tolerance) != 0
				
	
	## MODIFY
	
	def writeDetail(self, key, value):
		if value is None or isinstance(value, str) and value == '':
			if key in self.details:
				del self.details[key]
		else:
			self.details[key] = value
		if key in [FPL.ROUTE, FPL.ICAO_DEP, FPL.ICAO_ARR]:
			self._parseRoute()
	
	def linkFPL(self, fpl):
		self.linked_FPL = fpl
		self._parseRoute()
		if fpl is not None and settings.strip_autofill_on_FPL_link:
			self.fillFromFPL(useFpl=fpl)
	
	def linkAircraft(self, acft):
		self.linked_aircraft = acft
	
	def pushToFPL(self, ovr=False):
		fpl = self.linkedFPL()
		if fpl is not None:
			for d, v in self.details.items():
				if d in FPL.details and (ovr or fpl[d] is None):
					fpl[d] = v
	
	def fillFromFPL(self, useFpl=None, ovr=False):
		fpl = some(useFpl, self.linkedFPL())
		if fpl is not None:
			for detail in strip_editable_FPL_details:
				if fpl[detail] is not None and (ovr or detail not in self.details):
					self.writeDetail(detail, fpl[detail])
	
	def fillFromXPDR(self, ovr=False):
		acft = self.linkedAircraft()
		if acft is not None:
			details = {FPL.CALLSIGN: acft.xpdrCallsign(), FPL.ACFT_TYPE: acft.xpdrAcftType(), assigned_SQ_detail: acft.xpdrCode()}
			for detail, xpdr_value in details.items():
				if xpdr_value is not None and (ovr or detail not in self.details):
					self.writeDetail(detail, xpdr_value)
	
	def clearVectors(self):
		for detail in [assigned_heading_detail, assigned_altitude_detail, assigned_speed_detail]:
			self.writeDetail(detail, None)
	
	def insertRouteWaypoint(self, navpoint):
		route = self.lookup(parsed_route_detail)
		assert route is not None, 'Strip.insertRouteWaypoint: invalid route'
		lost_specs = route.insertWaypoint(navpoint)
		self.details[FPL.ROUTE] = route.enRouteStr() # bypass parse induced by writeDetail method
		return lost_specs
	
	def removeRouteWaypoint(self, navpoint):
		route = self.lookup(parsed_route_detail)
		assert route is not None, 'Strip.removeRouteWaypoint: invalid route'
		lost_specs = route.removeWaypoint(navpoint)
		self.details[FPL.ROUTE] = route.enRouteStr() # bypass parse induced by writeDetail method
		return lost_specs
