
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

from datetime import datetime

from base.db import acft_cat
from base.params import Heading
from base.util import m2NM, pop_all, A_star_search

from session.config import settings


# ---------- Constants ----------

default_RWY_disp_line_length = 25 # NM  (keep int)
default_RWY_FPA = 5.2 # percent

max_twy_edge_length = .1 # NM
inserted_twy_node_prefix = 'ADDED:'

straight_taxi_max_turn = 20 # degrees

# -------------------------------




class AirportData:
	def __init__(self):
		self.navpoint = None
		self.physical_runways = []    # list of (dirRWY 1, dirRWY 2, float width in metres, surface type X-plane code) tuples
		self.directional_runways = {} # str -> DirRunway dict
		self.helicopter_pads = []     # Helipad list
		self.frequencies = []         # (CommFrequency, str descr, str type) list
		self.ground_net = GroundNetwork()
		self.field_elevation = None
		self.viewpoints = []          # apt.dat specified viewpoints (tower)
		self.windsocks = []           # coordinates
		self.transition_altitude = None
		# ATTRIBUTE BELOW: list items match indices in self.physical_runways
		self.RWY_separation_timers = [] # opt. time, opt. WTC (reset manually or when runway box freed)
	
	def addPhysicalRunway(self, width, surface, rwy1, rwy2):
		rwy1._opposite_runway = rwy2
		rwy2._opposite_runway = rwy1
		rwy1._orientation = rwy1.thr.headingTo(rwy2.thr)
		rwy2._orientation = rwy2.thr.headingTo(rwy1.thr)
		rwy1._physical_runway = rwy2._physical_runway = len(self.physical_runways)
		rwy = min(rwy1, rwy2, key=(lambda r: r.name))
		self.physical_runways.append((rwy, rwy.opposite(), width, surface))
		self.RWY_separation_timers.append((datetime(1, 1, 1), None))
		self.directional_runways[rwy1.name] = rwy1
		self.directional_runways[rwy2.name] = rwy2
	
	def runway(self, rwy): # raise KeyError if RWY does not exist
		return self.directional_runways[rwy]

	def directionalRunways(self):
		return sorted(self.directional_runways.values(), key=lambda rwy: rwy.name)

	def helipads(self):
		return sorted(self.helicopter_pads, key=lambda hpad: hpad.name)
	
	def physicalRunwayCount(self):
		return len(self.physical_runways)
	
	def physicalRunway(self, index):
		rwy1, rwy2, w, s = self.physical_runways[index]
		return rwy1, rwy2
	
	def physicalRunwayData(self, index):
		rwy1, rwy2, w, s = self.physical_runways[index]
		return w, s
	
	def physicalRunwayNameFromUse(self, index):
		rwy1, rwy2, w, s = self.physical_runways[index]
		if rwy1.inUse() == rwy2.inUse():
			return '%s/%s' % (rwy1.name, rwy2.name)
		else:
			return rwy1.name if rwy1.inUse() else rwy2.name
	
	def rwySepTimer(self, phyrwy_index):
		t, wtc = self.RWY_separation_timers[phyrwy_index]
		return settings.session_manager.clockTime() - t, wtc
	
	def resetRwySepTimer(self, phyrwy_index, wtc):
		self.RWY_separation_timers[phyrwy_index] = settings.session_manager.clockTime(), wtc






class DepLdgSurface:
	def __init__(self, name):
		self.name = name # str
		self._is_runway = NotImplemented # to implement in inherited classes
		# Changeable parameters below
		self.use_for_departures = False
		self.use_for_arrivals = False

	def isRunway(self):
		return self._is_runway

	def inUse(self):
		return self.use_for_departures or self.use_for_arrivals

	def acceptsAcftType(self, t):
		"""
		NOTE: returns False if the X-plane category is unknown for the given ICAO type
		"""
		return NotImplemented

	def touchDownPoint(self):
		return NotImplemented

	def pointIsOnSurface(self, pos):
		return NotImplemented

	def readOut(self, tts=False):
		return NotImplemented



class Helipad(DepLdgSurface):
	def __init__(self, name, centre, surface, length, width, ori):
		DepLdgSurface.__init__(self, name)
		self._is_runway = False
		self.centre = centre # EarthCoords
		self.surface = surface # int surface code
		self.length = length
		self.width = width
		self.orientation = ori
		self.use_for_departures = True # switch default
		self.use_for_arrivals = True # switch default
		# Saved parameters
		self.param_preferred_DEP_course = Heading(360, False)

	def setDepCourse(self, magnetic_degrees):
		self.param_preferred_DEP_course = Heading(magnetic_degrees, False)

	def acceptsAcftType(self, t):
		return acft_cat(t) == 'helos'

	def touchDownPoint(self):
		return self.centre

	def pointIsOnSurface(self, pos):
		ctrpt = self.centre.toRadarCoords()
		hh = self.length / 2 # half height
		return pos.toRadarCoords().isBetween(ctrpt.moved(self.orientation, hh), ctrpt.moved(self.orientation.opposite(), hh), m2NM * self.width / 2)

	def readOut(self, tts=False):
		return tts_if(tts, 'SPELL_ALPHANUMS', self.name)



class DirRunway(DepLdgSurface):
	def __init__(self, name, rwy_start, disp_thr, width):
		DepLdgSurface.__init__(self, name)
		self._is_runway = True
		self.thr = rwy_start # EarthCoords
		self.dthr = disp_thr # float metres
		self.width = width
		# ILS properties
		self.ILS_cat = None     # or None if no LOC
		self.LOC_freq = None    # or None if no LOC
		self.LOC_range = None   # or None if no LOC
		self.LOC_bearing = None # or None if no LOC
		self.GS_range = None    # or None if no GS
		self.IM_pos = None      # or None
		self.MM_pos = None      # or None
		self.OM_pos = None      # or None
		# Saved parameters
		self.param_FPA = default_RWY_FPA # this is GS angle if ILS, or manually set (%)
		self.param_disp_line_length = default_RWY_disp_line_length
		self.param_acceptProps = True
		self.param_acceptTurboprops = True
		self.param_acceptJets = True
		self.param_acceptHeavy = True

	def acceptsAcftType(self, t):
		return {
				'props': self.param_acceptProps, 'turboprops': self.param_acceptTurboprops,
				'jets': self.param_acceptJets, 'heavy': self.param_acceptHeavy, 'helos': True
			}.get(acft_cat(t), False)

	def touchDownPoint(self):
		return self.threshold(dthr=True)

	def pointIsOnSurface(self, pos):
		return pos.toRadarCoords().isBetween(self.threshold().toRadarCoords(), self.threshold().toRadarCoords(), m2NM * self.width / 2)

	def readOut(self, tts=False):
		return 'runway ' + tts_if(tts, 'RWY', self.name)
	
	def physicalRwyIndex(self):
		return self._physical_runway
	
	def orientation(self):
		return self._orientation
	
	def opposite(self):
		return self._opposite_runway
	
	def threshold(self, dthr=False):
		if dthr:
			return self.thr.moved(self.orientation(), m2NM * self.dthr)
		else:
			return self.thr
	
	def length(self, dthr=False):
		return self.threshold(dthr=dthr).distanceTo(self.opposite().threshold(dthr=False))
	
	def hasILS(self):
		return self.LOC_range is not None and self.GS_range is not None
	
	def appCourse(self):
		return self.orientation() if self.LOC_bearing is None else self.LOC_bearing






class GroundNetwork:
	"""
	Contains all nodes of ground nets, including those on runways and apron.
	An edge is considered on apron if the taxiway name connecting its two end nodes is None.
	"""
	def __init__(self):
		self._nodes = {} # node ID (str) -> EarthCoords
		self._neighbours = {} # node ID (str) -> (node ID (neighbour) -> taxiway name or None, str RWY spec or None, float length)
		self._pkpos = {} # pk ID (str) -> EarthCoords, Heading, str (gate|hangar|tie-down), cat list or [] for all
		self._twy_edges = {} # TWY -> node pair set # EDGES IN MOVEMENT AREA (controlled) OTHER THAN RUNWAYS
		self._apron_edges = set() # node pair set   # EDGES IN NON MOVEMENT AREA (ramp/apron)
		self.inserted_twy_node_counter = 0 # increments to generate new name for every inserted node (used to avoid too long edges)
	
	# BUILDERS
	def addNode(self, node, position):
		self._nodes[node] = position
		self._neighbours[node] = {}
	
	def addEdge(self, n1, n2, rwy, twy):
		"""
		Add an edge to the ground net.
		Specify rwy/twy:
		- none = apron edge (non-moving area)
		- RWY only = runway edge (on runway), give a str spec of which RWY the edge is on (usually bidir RWY/OPP format)
		- TWY only = taxiway edge (in moving area), give the name of the TWY the edge is part of
		- both: is invalid
		"""
		p1 = self.nodePosition(n1)
		p2 = self.nodePosition(n2)
		edge_length = p1.distanceTo(p2)
		if edge_length > max_twy_edge_length:
			new_node = inserted_twy_node_prefix + str(self.inserted_twy_node_counter)
			self.inserted_twy_node_counter += 1
			self.addNode(new_node, p1.moved(p1.headingTo(p2), edge_length / 2))
			self.addEdge(n1, new_node, rwy, twy)
			self.addEdge(new_node, n2, rwy, twy)
		else:
			self._neighbours[n1][n2] = self._neighbours[n2][n1] = twy, rwy, edge_length
			if twy is None:
				if rwy is None:
					self._apron_edges.add((n1, n2))
			else:
				try:
					self._twy_edges[twy].add((n1, n2))
				except KeyError:
					self._twy_edges[twy] = {(n1, n2)}
	
	def addParkingPosition(self, pkid, pos, hdg, typ, who):
		self._pkpos[pkid] = pos, hdg, typ, who
	
	# ACCESS NODES
	def nodes(self, pred=None):
		return list(self._nodes) if pred is None else [n for n in self._nodes if pred(n)]
	
	def nodePosition(self, nid):
		return self._nodes[nid]
	
	def neighbours(self, nid, twy=None, ignoreApron=False):
		ok = lambda t, r, l: (twy is None or t == twy) and not (ignoreApron and t is None and r is None)
		return [n for n, data in self._neighbours[nid].items() if ok(*data)]
	
	def nodeIsInSourceData(self, nid):
		return not nid.startswith(inserted_twy_node_prefix)
	
	def nodeIsRwyCentre(self, nid, rwy):
		return rwy in self.connectedRunways(nid, bidir=True)
	
	def connectedRunways(self, nid, bidir=False):
		res = set()
		for n2 in self.neighbours(nid):
			rwy_spec = self._neighbours[nid][n2][1]
			if rwy_spec is not None:
				rwys = rwy_spec.split('/')
				if bidir:
					res.update(rwys)
				else:
					res.add(sorted(rwys)[0])
		return list(res)
	
	def closestNode(self, pos, maxdist=None):
		ndlst = [(n, self.nodePosition(n).distanceTo(pos)) for n in self._nodes]
		if len(ndlst) > 0:
			node, dist = min(ndlst, key=(lambda nd: nd[1]))
			if maxdist is None or dist <= maxdist:
				return node
		return None
	
	# ACCESS EDGES AND TAXIWAYS
	def taxiways(self):
		return list(self._twy_edges)
	
	def connectedTaxiways(self, nid):
		return [twy for twy, rwy, l in self._neighbours[nid].values() if twy is not None]
	
	def apronEdges(self):
		return self._apron_edges
	
	def taxiwayEdges(self, twy):
		return self._twy_edges[twy]
	
	# ACCESS PARKING POSITIONS
	def parkingPositions(self, acftCat=None, acftType=None):
		if acftType is None and acftCat is None: # no ACFT type filter
			return list(self._pkpos)
		if acftType is not None:
			assert acftCat is None
			acftCat = acft_cat(acftType) # may be None but OK
		return [pk for pk, pkinfo in self._pkpos.items() if acftCat in pkinfo[3] or pkinfo[3] == []]
	
	def parkingPosition(self, pkid):
		return self._pkpos[pkid][0]
	
	def parkingPosInfo(self, pkid):
		return self._pkpos[pkid]
	
	def closestParkingPosition(self, pos, maxdist=None):
		pklst = [(pk, self.parkingPosition(pk).distanceTo(pos)) for pk in self.parkingPositions()]
		if pklst != []:
			pk, dist = min(pklst, key=(lambda pk: pk[1]))
			if maxdist is None or dist <= maxdist:
				return pk
		return None
	
	# TURN-OFF POINTS
	def runwayTurnOffs(self, rwy, maxPrefAngle=90, minRoll=0):
		"""
		Returns a list tuple (L1, L2, L3, L4) containing turn-offs after a landing roll down the given RWY, in preferred order:
		- L1: preferred turn-offs, i.e. small turn-off angle ahead and not ending on a runway
		- L2: sharper turn-offs ahead not ending on a runway
		- L3: turn-offs ahead ending on a runway
		- L4: backtrack required
		In all cases a turn-off is a node list representing a taxi route beginning on the runway centre line and ending off the runway.
		All lists are sorted by distance from current point.
		"""
		res = []
		for rwy_node in self.nodes(lambda node: self.nodeIsRwyCentre(node, rwy.name)):
			rwy_node_pos = self.nodePosition(rwy_node)
			for n in self.neighbours(rwy_node):
				if not self.nodeIsRwyCentre(n, rwy.name): # n is a node that starts a turn off the RWY centre line
					for nlst in self._routesOffRwy(rwy, n, rwy_node):
						turn_angle = rwy_node_pos.headingTo(self.nodePosition(nlst[-1])).diff(rwy.orientation())
						res.append(([rwy_node] + nlst, rwy.threshold().distanceTo(rwy_node_pos), turn_angle))
		res.sort(key=(lambda t: t[1])) # sort by distance to THR
		res_backtrack = [rte for rte, d, a in pop_all(res, lambda t: t[1] < minRoll)]
		res_backtrack.reverse()
		res_on_rwys = [rte for rte, d, a in pop_all(res, lambda t: len(self.connectedRunways(t[0][-1])) > 0)] # turn off on a RWY
		res_sharp = [rte for rte, d, a in pop_all(res, lambda t: abs(t[2]) > maxPrefAngle)] # sharp turn-off
		return [rte for rte, d, a in res], res_sharp, res_on_rwys, res_backtrack
	
	# ROUTES
	def _routesOffRwy(self, rwy, from_node, prev_node):
		if rwy.pointIsOnSurface(self.nodePosition(from_node)):
			return [[from_node] + lst for n in self.neighbours(from_node) if n != prev_node for lst in self._routesOffRwy(rwy, n, from_node)]
		else:
			return [[from_node]]

	def _routeHopsFrom(self, n1, avoid_runways):
		res = []
		for n2, (twy, rwy, cost) in self._neighbours[n1].items():
			if avoid_runways: # add penalties for entering/crossing RWYs
				if not all(self.nodeIsRwyCentre(n1, r) for r in self.connectedRunways(n2)): # stepping on a RWY
					cost += 15
				elif rwy is not None: # taxi edge fully on RWY
					cost += 5
			res.append((n2, cost, (twy, rwy))) # edge labels not used anyway
		return res
	
	def shortestTaxiRoute(self, src, goal, avoid_runways):
		fh = lambda n, g=self.nodePosition(goal): self.nodePosition(n).distanceTo(g)
		return A_star_search(src, goal, (lambda n: self._routeHopsFrom(n, avoid_runways)), heuristic=fh)[0]
	
	def taxiInstrStr(self, node_sequence, finalNonNode=None, tts=False):
		# NOTE: keep "runway" spelt out in strings to allow TTS to read
		if len(node_sequence) == 0:
			if finalNonNode is None:
				return 'Hold position'
			else:
				return 'Taxi to %s' % finalNonNode
		elif len(node_sequence) == 1 and finalNonNode is None:
			n = node_sequence[0]
			rwys = self.connectedRunways(n)
			if rwys == []:
				twys = self.connectedTaxiways(n)
				return 'Taxi on %s' % ('apron' if twys == [] else tts_if(tts, 'SPELL_ALPHANUMS', twys[0]))
			else:
				return 'Enter runway %s' % tts_if(tts, 'RWY', rwys[0])
		else:
			instr = []
			n_prev = node_sequence[0]
			edge_prev = None
			hdg_prev = None
			rwys_prev = self.connectedRunways(n_prev) if n_prev in self._nodes else []
			on_prev = []
			for n in node_sequence[1:]:
				twy_lbl, rwy_lbl, ignore_len = self._neighbours[n_prev][n] # n_prev is Not a pkpos (parking comes last)
				hdg = self.nodePosition(n_prev).headingTo(self.nodePosition(n))
				turn = None if hdg_prev is None else hdg.diff(hdg_prev)
				rwys = self.connectedRunways(n)
				for r in on_prev:
					if r in rwys_prev and r not in rwys:
						instr.append('cross runway %s' % tts_if(tts, 'RWY', r))
				if (twy_lbl, rwy_lbl) != edge_prev: # else: staying on same TWY => silent hop
					if turn is None:
						tt = 'Taxi' if instr == [] else 'then'
					elif abs(turn) <= straight_taxi_max_turn:
						tt = 'straight'
					else:
						tt = 'right' if turn > 0 else 'left'
					if twy_lbl is None and rwy_lbl is None:
						edge_str = 'apron'
					elif twy_lbl is None:
						edge_str = 'runway ' + tts_if(tts, 'RWY', rwy_lbl.split('/', maxsplit=1)[0])
					else:
						edge_str = tts_if(tts, 'SPELL_ALPHANUMS', twy_lbl)
					instr.append('%s on %s' % (tt, edge_str))
				edge_prev = twy_lbl, rwy_lbl
				on_prev = [r for r in rwys if r not in rwys_prev]
				n_prev = n
				hdg_prev = hdg
				rwys_prev = rwys
			if finalNonNode is not None: # recognise "to point"
				instr.append('%s to %s' % (('Taxi' if instr == [] else 'then'), tts_if(tts, 'SPELL_ALPHANUMS', finalNonNode)))
			if on_prev != []:
				instr.append('enter runway %s' % tts_if(tts, 'RWY', on_prev[0]))
			return ', '.join(instr)


def tts_if(condition, tts_cmd, contents):
	return '\\%s{%s}' % (tts_cmd, contents) if condition else contents
