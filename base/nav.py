
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

from base.coords import EarthCoords
from base.params import Heading
from base.util import A_star_search


# ---------- Constants ----------

# -------------------------------

class NavpointError(Exception):
	pass


class Navpoint:
	types = AD, VOR, NDB, FIX, ILS, RNAV = range(6)

	@staticmethod
	def tstr(t):
		return {
			Navpoint.AD:  'AD',  Navpoint.VOR: 'VOR', Navpoint.NDB: 'NDB',
			Navpoint.FIX: 'Fix', Navpoint.ILS: 'ILS', Navpoint.RNAV: 'RNAV'
		}[t]

	@staticmethod
	def findType(typestr):
		try:
			return {
				'AD':  Navpoint.AD,  'VOR': Navpoint.VOR, 'NDB': Navpoint.NDB,
				'FIX': Navpoint.FIX, 'ILS': Navpoint.ILS, 'RNAV': Navpoint.RNAV
			}[typestr.upper()]
		except KeyError:
			raise ValueError('Invalid navpoint type specifier "%s"' % typestr)
	
	def __init__(self, t, code, coords):
		"""
		code should be upper case
		"""
		self.type = t
		self.code = code
		self.coordinates = coords
		self.icao_region = None # to be overridden if applicable
		self.long_name = '' # to be overridden if applicable
	
	def __str__(self):
		return self.code


class Airfield(Navpoint):
	def __init__(self, icao, coords, airport_name):
		Navpoint.__init__(self, Navpoint.AD, icao, coords)
		self.long_name = airport_name

class VOR(Navpoint):
	def __init__(self, identifier, coords, region, frq, long_name, tacan=False):
		Navpoint.__init__(self, Navpoint.VOR, identifier, coords)
		self.icao_region = region
		self.long_name = long_name
		self.frequency = frq
		self.dme = False
		self.tacan = tacan

class NDB(Navpoint):
	def __init__(self, identifier, coords, region, frq, long_name):
		Navpoint.__init__(self, Navpoint.NDB, identifier, coords)
		self.icao_region = region
		self.long_name = long_name
		self.frequency = frq
		self.dme = False

class Fix(Navpoint):
	def __init__(self, name, coords, region):
		Navpoint.__init__(self, Navpoint.FIX, name, coords)
		self.icao_region = region

class Rnav(Navpoint):
	def __init__(self, name, coords, region):
		Navpoint.__init__(self, Navpoint.RNAV, name, coords)
		self.icao_region = region





navpoint_spec_regexp = re.compile(r'(\((?P<type>.+)\))?(?P<code>[^()@]+)(@(?P<region>.+))?')

def _navpoint_spec_filters(specstr):
	"""
	Parse a unique named navpoint with optional filters:
		- type restriction prefix of the form "(TYPE)";
		- region selection suffix "@REGION".
	Returns (navpoint code, type_list, region_filter) where:
		- type_list is the list of navpoint types to use with the NavDB.find* methods as "types" filter;
		- region_filter is the specified region selection if provided, None otherwise.
	Raises a "ValueError" if TYPE is invalid.
	"""
	match = navpoint_spec_regexp.fullmatch(specstr)
	if match is None:
		raise ValueError('Bad navpoint spec "%s"' % specstr)
	gtype = match.group('type')
	tlst = Navpoint.types if gtype is None else [Navpoint.findType(gtype)]
	return match.group('code'), tlst, match.group('region')





class NavDB:
	def __init__(self):
		self.by_type = {t: [] for t in Navpoint.types} # type -> navpoint list (KeyError safe)
		self.by_code = {} # code -> navpoint list (KeyError is possible)
	
	def add(self, p):
		self.by_type[p.type].append(p)
		try:
			self.by_code[p.code].append(p)
		except KeyError:
			self.by_code[p.code] = [p]
	
	def clear(self):
		for key in self.by_type:
			self.by_type[key] = []
		self.by_code.clear()
	
	def subDB(self, pred):
		result = NavDB()
		result.by_type = {t: [p for p in plst if pred(p)] for t, plst in self.by_type.items()}
		result.by_code = {c: [p for p in plst if pred(p)] for c, plst in self.by_code.items() if plst != []}
		return result
	
	def byType(self, t): # WARNING: do not alter result
		return self.by_type[t]
	
	def findAll(self, code=None, types=Navpoint.types, region=None):
		"""
			Returns a list of navpoints in the NavDB, satisfying the given filters.
			WARNING: region filter without a code may be slow.
			WARNING: do not alter result.
		"""
		if code is None:
			result = []
			for t in types:
				result += self.byType(t)
		else:
			result = [p for p in self.by_code.get(code.upper(), []) if p.type in types]
		return result if region is None else [p for p in result if p.icao_region == region]
	
	def findUnique(self, code, types=Navpoint.types, region=None):
		"""
		raises NavpointError if non-single navpoint found with given code and type in "types" list + region filter
		"""
		candidates = self.findAll(code, types, region)
		if len(candidates) != 1:
			raise NavpointError('' if code is None else str(code))
		return candidates[0]
	
	def findClosest(self, ref, code=None, types=Navpoint.types, region=None, maxDist=None):
		"""
		raises NavpointError if no navpoint is found with given code and type in "types" list + region filter
		"""
		candidates = self.findAll(code, types, region)
		if len(candidates) > 0:
			closest = min(candidates, key=(lambda p: ref.distanceTo(p.coordinates)))
			if maxDist is None or closest.coordinates.distanceTo(ref) <= maxDist:
				return closest
		raise NavpointError('' if code is None else str(code))
	
	def findAirfield(self, icao):
		"""
		raises NavpointError if no airfield is found with given code
		"""
		return self.findUnique(icao, types=[Navpoint.AD]) # can raise NavpointError

	def fromSpec(self, specstr):
		if '~' in specstr: # name closest to point spec
			name, near = specstr.split('~', maxsplit=1)
			name, types_filter, region_filter = _navpoint_spec_filters(name)
			return self.findClosest(self.coordsFromPointSpec(near), code=name, types=types_filter, region=region_filter)
		else:
			name, types_filter, region_filter = _navpoint_spec_filters(specstr)
			return self.findUnique(name, types=types_filter, region=region_filter)

	def coordsFromPointSpec(self, spec):
		mvlst = spec.split('>')
		pbase = mvlst.pop(0)
		if ',' in pbase and '~' not in pbase:
			result = EarthCoords.fromString(pbase)
		else:
			result = self.fromSpec(pbase).coordinates
		while mvlst:
			mv = mvlst.pop(0).split(',')
			if len(mv) == 2:
				radial = Heading(float(mv[0]), True)
				distance = float(mv[1])
				result = result.moved(radial, distance)
			else:
				raise ValueError('Bad use of `>\' in point spec "%s"' % spec)
		return result


world_navpoint_db = NavDB()







class RoutingDB:
	def __init__(self):
		self.airways = {} # navpoint -> (navpoint -> (str name, int FL_min, int FL_max))
		self.entries = {} # str ICAO code -> (navpoint, str list leg spec) list
		self.exits = {}   # str ICAO code -> (navpoint, str list leg spec) list
	
	
	## POPULATE/CLEAR
	
	def addAwy(self, p1, p2, name, fl_lo, fl_hi):
		try:
			self.airways[p1][p2] = name, fl_lo, fl_hi # may override an adge if already one between those two points
		except KeyError:
			self.airways[p1] = {p2: (name, fl_lo, fl_hi)}
	
	def addEntryPoint(self, ad, p, leg_spec):
		try:
			self.entries[ad.code].append((p, leg_spec))
		except KeyError:
			self.entries[ad.code] = [(p, leg_spec)]
	
	def addExitPoint(self, ad, p, leg_spec):
		try:
			self.exits[ad.code].append((p, leg_spec))
		except KeyError:
			self.exits[ad.code] = [(p, leg_spec)]
	
	def clearEntryExitPoints(self):
		self.entries.clear()
		self.exits.clear()
	
	
	## ACCESS
	
	def airfieldsWithEntryPoints(self):
		return [world_navpoint_db.findAirfield(icao) for icao in self.entries]
	
	def airfieldsWithExitPoints(self):
		return [world_navpoint_db.findAirfield(icao) for icao in self.exits]
	
	def entriesTo(self, ad):
		return self.entries.get(ad.code, [])
	
	def exitsFrom(self, ad):
		return self.exits.get(ad.code, [])
	
	
	## ROUTING
	
	def _waypointsFrom(self, p1, destination):
		try: # FUTURE depend on a current FL for AWYs (or at least a hi/lo layer)?
			res = [(p2, p1.coordinates.distanceTo(p2.coordinates), awy[0]) for p2, awy in self.airways[p1].items()]
		except KeyError:
			res = []
		if p1.type == Navpoint.AD:
			res.extend((p2, p1.coordinates.distanceTo(p2.coordinates), ' '.join(legspec)) for p2, legspec in self.exitsFrom(p1))
		for entry_point, legspec in self.entriesTo(destination):
			if p1 == entry_point:
				res.append((destination, p1.coordinates.distanceTo(destination.coordinates), ' '.join(legspec)))
		return res
	
	def shortestRoute(self, p1, p2):
		"""
		returns a PAIR of lists: waypoint hops, AWY legs
		result is the shortest route in distance using AWYs, with no intermediate waypoints along AWYs
		p1 and p2 can be any Navpoint; raises ValueError if no route exists
		"""
		fh = lambda p: p.coordinates.distanceTo(p2.coordinates)
		waypoints, awys = A_star_search(p1, p2, (lambda p: self._waypointsFrom(p, p2)), heuristic=fh) # may raise ValueError
		# Simplify lists: remove waypoints when remaining on same AWY
		i = 0
		while i < len(waypoints) - 1:
			if awys[i] == awys[i + 1]:
				del awys[i]
				del waypoints[i]
			else:
				i += 1
		return waypoints, awys
	
	def shortestRouteStr(self, p1, p2):
		"""
		returns a string spec of the shortest route, without p1 and p2
		raises ValueError if no route exists
		"""
		waypoints, awys = self.shortestRoute(p1, p2) # may raise ValueError
		if len(waypoints) == 0:
			return ''
		else:
			pairs = list(zip(waypoints, awys))
			s = ' '.join('%s %s' % (awy, wp) for wp, awy in pairs[:-1]) + ' ' + awys[-1]
			return s.strip()


world_routing_db = RoutingDB()
