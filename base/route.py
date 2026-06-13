
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

from base.coords import EarthCoords
from base.nav import world_navpoint_db, NavpointError


# ---------- Constants ----------

route_acceptable_WPdistToArr_increase = 100 # NM

# -------------------------------


class Route:
	def __init__(self, origin, destination_navpoint, via_string):
		"""
		:param origin: can be EarthCoords or Navpoint
		:param destination_navpoint: must be a Navpoint
		:param via_string: whitespace-separated tokens mixing waypoints and leg specifications in between
		"""
		if isinstance(origin, EarthCoords):
			self.origin_coords = origin
			self.origin_navpoint = None
		else:
			self.origin_coords = origin.coordinates
			self.origin_navpoint = origin
		self.dest_navpoint = destination_navpoint
		self.enroute_waypoints = [] # recognised navigation points in init_string
		self.leg_specs = [[]] # list of leg specs; each leg spec is a list of tokens
		init_tokens = via_string.split()
		prev_coords = self.origin_coords
		for token in init_tokens:
			try:
				next_waypoint = world_navpoint_db.findClosest(prev_coords, code=token)
				dist_limit = prev_coords.distanceTo(self.dest_navpoint.coordinates) + route_acceptable_WPdistToArr_increase
				if next_waypoint.coordinates.distanceTo(self.dest_navpoint.coordinates) > dist_limit:
					next_waypoint = None
			except NavpointError:
				next_waypoint = None
			if next_waypoint is None: # consider as a leg spec token to next waypoint to come
				self.leg_specs[-1].append(token)
			else: # we have found the next waypoint
				self.enroute_waypoints.append(next_waypoint)
				self.leg_specs.append([])
				prev_coords = next_waypoint.coordinates
		# Remove duplicated end airfields if no leg specs
		if len(self.enroute_waypoints) > 0 and self.enroute_waypoints[0] is self.origin_navpoint and len(self.leg_specs[0]) == 0:
			del self.enroute_waypoints[0]
			del self.leg_specs[0]
		if len(self.enroute_waypoints) > 0 and self.enroute_waypoints[-1] is self.dest_navpoint and len(self.leg_specs[-1]) == 0:
			del self.enroute_waypoints[-1]
			del self.leg_specs[-1]
	
	def dup(self):
		dup = Route(self.origin_navpoint, self.dest_navpoint, '')
		dup.enroute_waypoints = self.enroute_waypoints[:]
		dup.leg_specs = self.leg_specs[:]
		return dup
	
	
	## ACCESS
	
	def legCount(self):
		return len(self.leg_specs)
	
	def originCoords(self):
		return self.origin_coords
	
	def knownOriginNavpoint(self):
		return self.origin_navpoint
	
	def destinationNavpoint(self):
		return self.dest_navpoint
	
	def waypoint(self, n):
		"""
		waypoint 0 is the first waypoint after departure
		"""
		return self.enroute_waypoints[n] if n < self.legCount() - 1 else self.destinationNavpoint()
	
	def legSpec(self, n):
		"""
		this method returns the leg spec tokens of the leg to waypoint 'n' (n=0 is departure leg)
		"""
		return self.leg_specs[n]
	
	def __contains__(self, navpoint):
		"""
		tests if navpoint is an enroute waypoint (this excludes departure and arrival points)
		"""
		try:
			ignore = next(wp for wp in self.enroute_waypoints if wp is navpoint)
			return True
		except StopIteration:
			return False
	
	def totalDistance(self):
		result = self.originCoords().distanceTo(self.waypoint(0).coordinates)
		for i in range(self.legCount() - 1):
			result += self.waypoint(i).coordinates.distanceTo(self.waypoint(i + 1).coordinates)
		return result
	
	def routePointCoords(self):
		return [self.originCoords()] + [self.waypoint(i).coordinates for i in range(self.legCount())]
	
	def currentLegIndex(self, position):
		"""
		returns the number of the route leg to be followed, based on distance to arrival, given a position on Earth
		0 is first; legCount-1 is last
		"""
		dist_to_dep = position.distanceTo(self.originCoords())
		dist_to_arr = position.distanceTo(self.destinationNavpoint().coordinates)
		if dist_to_dep < dist_to_arr and dist_to_dep < self.originCoords().distanceTo(self.waypoint(0).coordinates):
			return 0
		for i in reversed(range(self.legCount() - 1)):
			if self.waypoint(i).coordinates.distanceTo(self.destinationNavpoint().coordinates) >= dist_to_arr:
				return i + 1
		return 0
	
	def currentWaypoint(self, position):
		return self.waypoint(self.currentLegIndex(position))
	
	def SID(self):
		if self.legCount() > 1 and 'SID' in [token.upper() for token in self.legSpec(0)]:
			return str(self.waypoint(0))
		else:
			return None
	
	def STAR(self):
		if self.legCount() > 1 and 'STAR' in [token.upper() for token in self.legSpec(self.legCount() - 1)]:
			return str(self.waypoint(self.legCount() - 2))
		else:
			return None
	
	
	## STRINGS
	
	def __str__(self):
		orig_prefix = '' if self.origin_navpoint.code is None else '%s ' % self.origin_navpoint
		return orig_prefix + ' '.join(self.legStr(i, start=False) for i in range(self.legCount()))
	
	def enRouteStr(self):
		result = ' '.join(self.legStr(i, start=False) for i in range(self.legCount() - 1))
		last_leg_spec = ' '.join(self.legSpec(self.legCount() - 1))
		if last_leg_spec != '':
			result += ' ' + last_leg_spec
		return result
	
	def legStr(self, n, start=True):
		if start and not (n == 0 and self.origin_navpoint is None):
			leg_start = str(self.origin_navpoint if n == 0 else self.waypoint(n - 1)) + ' '
		else:
			leg_start = ''
		leg_specs = ' '.join(self.legSpec(n))
		if leg_specs != '':
			leg_specs += ' '
		return leg_start + leg_specs + str(self.waypoint(n))
	
	def toGoStr(self, position):
		return ' '.join(self.legStr(i, start=False) for i in range(self.currentLegIndex(position), self.legCount()))
	
	
	## MODIFIERS
	
	def removeWaypoint(self, navpoint):
		"""
		returns the lost leg specs (before wp, after wp)
		"""
		leg = next(ileg for ileg in reversed(range(self.legCount() - 1)) if self.waypoint(ileg) is navpoint)
		del self.enroute_waypoints[leg]
		lost_before = self.leg_specs.pop(leg)
		lost_after = self.leg_specs.pop(leg)
		self.leg_specs.insert(leg, [])
		return lost_before, lost_after
	
	def insertWaypoint(self, navpoint):
		"""
		returns the lost leg spec
		"""
		leg = self.currentLegIndex(navpoint.coordinates)
		self.enroute_waypoints.insert(leg, navpoint)
		old_leg_spec = self.legSpec(leg)
		self.leg_specs[leg] = []
		self.leg_specs.insert(leg, [])
		return old_leg_spec
