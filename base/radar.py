
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

from math import pi, atan2

from PyQt5.QtCore import pyqtSignal, QObject

from base.util import pop_all, pop_one
from base.strip import soft_link_detail, assigned_SQ_detail, runway_box_detail, received_from_detail
from base.fpl import FPL
from base.acft import Aircraft
from base.conflict import Conflict, position_conflict_test, path_conflict_test

from session.config import settings
from session.env import env
from session.manager import SessionType

from gui.misc import signals
from gui.widgets.basicWidgets import Ticker


# ---------- Constants ----------

XPDR_emergency_codes = [0o7500, 0o7600, 0o7700]
sector_count = 30  # 12 degrees per sector

# -------------------------------

def radar_sector(acft):
	coords = acft.liveCoords().toRadarCoords()
	return int((atan2(coords.y(), coords.x()) + pi) / (2 * pi) * sector_count) % sector_count


class Radar(QObject):
	acftBlip = pyqtSignal(Aircraft)
	newContact = pyqtSignal(Aircraft)
	lostContact = pyqtSignal(Aircraft)
	
	def __init__(self, gui):
		QObject.__init__(self)
		self.ticker = Ticker(gui, self.scanNewSector)
		self.scan_sector = 0
		self.known_contacts = []    # Aircraft list known from all prior sector scans and not yet lost
		self.sectors = {s: [] for s in range(sector_count)} # int sector -> Aircraft list visible during current sweep (to snapshot)
		self.blips_invisible = {}   # str -> int; number of blips for which ACFT callsign has been invisible
		self.EMG_squawkers = set()  # set of ACFT identifiers
		self.soft_links = []        # (Strip, Aircraft) pairs
		self.RWY_occupation = []    # for each physical RWY index: list of ACFT
		if env.airport_data is not None:
			for i in range(env.airport_data.physicalRunwayCount()):
				self.RWY_occupation.append([])

	## ACCESSORS
	def contacts(self):
		"""
		Returns a list of connected aircraft contacts
		"""
		return self.known_contacts[:]

	def missedOnLastScan(self, acft_id):
		"""
		True if ACFT is known (i.e. not already lost) but was not picked up on last radar scan.
		"""
		try:
			return self.blips_invisible[acft_id] > 0
		except KeyError:
			return False

	def runwayOccupation(self, phrwy):
		return self.RWY_occupation[phrwy]


	## CONTROL
	def _loseContact(self, acft):
		pop_all(self.known_contacts, lambda a: a is acft) # there should be one
		del self.blips_invisible[acft.identifier]
		self.EMG_squawkers.discard(acft.identifier)
		for occlst in self.RWY_occupation:
			pop_all(occlst, lambda a: a is acft) # there should be zero or one
		self.lostContact.emit(acft)

	def startSweeping(self):
		self.ticker.startTicking(settings.radar_sweep_interval / sector_count)
	
	def stopSweeping(self):
		self.ticker.stop()

	def scanSingleAcft(self, acft):
		self.blips_invisible[acft.identifier] = 0
		if not any(a is acft for a in self.known_contacts):
			self.known_contacts.append(acft)
			self.newContact.emit(acft)

		# ACFT checks after snapshot
		self.checkEmgSquawk(acft)
		if env.airport_data is not None:
			self.checkRunwayOccupation(acft)

	def scanNewSector(self):
		# UPDATE SECTOR MAP IF NEW SWEEP BEGINNING
		if self.scan_sector == 0:
			self.sectors = {s: [] for s in range(sector_count)}
			for acft in settings.session_manager.getAircraft():
				if acft.isRadarVisible():
					self.sectors[radar_sector(acft)].append(acft)

		# SNAPSHOTS FOR ACFT IN SECTOR (new contacts can appear)
		scanned_acft = self.sectors[self.scan_sector]
		for acft in scanned_acft:
			acft.saveRadarSnapshot() #NOTE does nothing in playback sessions (not to tamper with history)
			self.scanSingleAcft(acft)

		# END OF SECTOR SCAN REACHED (check for lost contacts)
		self.scan_sector += 1
		if self.scan_sector == sector_count: # finished a sweep
			for prev in self.known_contacts:
				if not any(a is prev for lst in self.sectors.values() for a in lst): # not seen after last whole sweep
					count = self.blips_invisible[prev.identifier]
					if count < settings.invisible_blips_before_contact_lost:
						self.blips_invisible[prev.identifier] = count + 1
					else: # lost contact
						self._loseContact(prev)
			self.scan_sector = 0

		# global checks after sector snapshots
		self.globalChecks()

		# BLIPS FOR DISPLAY UPDATES (keep after all strip changes)
		if settings.session_manager.session_type != SessionType.PLAYBACK and settings.radar_sweeping_display:
			for acft in scanned_acft:
				self.acftBlip.emit(acft)
		elif self.scan_sector == 0: # non-sweeping display, or playback refresh
			for acft in self.known_contacts:
				self.acftBlip.emit(acft)
				#DEBUGprint('Blip:', acft.identifier)

	def instantSweep(self):
		for i in range(sector_count):
			self.scanNewSector()

	def forgetContact(self, acft):
		for lst in self.sectors.values():
			try:
				pop_one(lst, lambda a: a is acft) # StopIteration if not in this sector
				self._loseContact(acft)
				break
			except StopIteration: # ACFT not in this sector
				pass

	def resetContacts(self):
		self.scan_sector = 0
		while self.known_contacts:
			self.lostContact.emit(self.known_contacts.pop())
		self.sectors.clear()
		self.blips_invisible.clear()
		self.soft_links.clear()
		self.EMG_squawkers.clear()
		for occlst in self.RWY_occupation:
			occlst.clear()


	## RADAR CHECKS
	def globalChecks(self): #CHECKME does signals.stripInfoChanged call this?
		if settings.traffic_identification_assistant:
			self.checkRadarIdentifications()
		self.checkPositionRouteConflicts()
		env.strips.refreshViews()

	def checkEmgSquawk(self, acft):
		if acft.xpdrCode() in XPDR_emergency_codes:
			if acft.identifier not in self.EMG_squawkers:
				self.EMG_squawkers.add(acft.identifier)
				signals.emergencySquawk.emit(acft)
		else:
			self.EMG_squawkers.discard(acft.identifier)

	def checkRunwayOccupation(self, acft):
		for iphrwy, occlst in enumerate(self.RWY_occupation):
			if acft.considerOnGround() and env.airport_data.physicalRunway(iphrwy)[0].pointIsOnSurface(acft.coords()): # ACFT now on RWY
				if not any(a is acft for a in occlst): # ACFT *just* entered the RWY
					occlst.append(acft)
					if settings.monitor_runway_occupation: # check if alarm must sound
						try:
							boxed_link = env.strips.findStrip(lambda s: s.lookup(runway_box_detail) == iphrwy).linkedAircraft()
						except StopIteration: # no strip boxed on this runway, but is it active?
							rwy1, rwy2 = env.airport_data.physicalRunway(iphrwy)
							if rwy1.inUse() or rwy2.inUse(): # entering a non-reserved but active RWY
								signals.runwayIncursion.emit(iphrwy, acft)
						else: # RWY is reserved; is this the right aircraft?
							if boxed_link is None and env.linkedStrip(acft) is None or boxed_link is acft:
								# entering ACFT is the one cleared to enter, or can be
								if len(self.RWY_occupation[iphrwy]) > 0: # some ACFT was/were already on RWY
									call_guilty = acft if boxed_link is None else self.RWY_occupation[iphrwy][0]
									signals.runwayIncursion.emit(iphrwy, call_guilty)
							else: # entering ACFT is known to be different from the one cleared to enter
								signals.runwayIncursion.emit(iphrwy, acft)
			else: # ACFT not on RWY
				pop_all(occlst, lambda a: a is acft) # never mind if already not in set

	def checkRadarIdentifications(self):
		found_S_links = []
		found_A_links = []
		for strip in env.strips.listAll():
			if strip.linkedAircraft() is None:
				mode_S_found = False
				# Try mode S identification
				if strip.lookup(FPL.CALLSIGN, fpl=True) is not None:
					scs = strip.lookup(FPL.CALLSIGN).upper()
					if env.strips.count(lambda s: s.lookup(FPL.CALLSIGN) is not None and s.lookup(FPL.CALLSIGN).upper() == scs) == 1:
						candidates = [acft for acft in self.contacts() if acft.xpdrCallsign() is not None and acft.xpdrCallsign().upper() == scs]
						if len(candidates) == 1:
							found_S_links.append((strip, candidates[0]))
							mode_S_found = True
				# Try mode A identification
				if not mode_S_found:
					ssq = strip.lookup(assigned_SQ_detail)
					if ssq is not None and env.strips.count(lambda s:
							s.lookup(assigned_SQ_detail) == ssq and s.linkedAircraft() is None) == 1: # only one non-linked strip with this SQ
						candidates = [acft for acft in self.contacts() if not any(a is acft for s, a in found_S_links)
										and acft.xpdrCode() == ssq and env.linkedStrip(acft) is None]
						if len(candidates) == 1: # only one aircraft matching
							found_A_links.append((strip, candidates[0]))
		for s, a in pop_all(self.soft_links, lambda sl: not any(sl[0] is s and sl[1] is a for s, a in found_S_links + found_A_links)):
			s.writeDetail(soft_link_detail, None)
		for s, a, m in [(s, a, True) for s, a in found_S_links] + [(s, a, False) for s, a in found_A_links]:
			if not any(sl[0] is s and sl[1] is a for sl in self.soft_links): # new found soft link
				if s.lookup(received_from_detail) is not None and settings.strip_autolink_mode_S and m:
					s.linkAircraft(a)
				else: # strip not automatically linked; notify of a new identification
					self.soft_links.append((s, a))
					s.writeDetail(soft_link_detail, a)
					signals.aircraftIdentification.emit(s, a, m)

	def checkPositionRouteConflicts(self):
		conflicts = {acft.identifier: Conflict.NO_CONFLICT for acft in self.contacts()}
		my_traffic_for_route_checks = []
		# Check for NEAR MISSES first against ALL traffic (incl. uncontrolled)
		for strip in env.strips.listAll():
			acft = strip.linkedAircraft()
			if acft is not None and acft.identifier in conflicts: # controlled traffic with radar contact
				for other in self.contacts():
					if other is not acft and position_conflict_test(acft, other) == Conflict.NEAR_MISS: # positive separation loss detected
						conflicts[acft.identifier] = conflicts[other.identifier] = Conflict.NEAR_MISS
				if not bypass_route_conflict_check(strip):
					my_traffic_for_route_checks.append(acft)
		# Check for PATH CONFLICTS now against CONTROLLED traffic only
		if settings.route_conflict_warnings: # check for route conflicts
			while len(my_traffic_for_route_checks) > 0: # progressively emptying the list
				acft = my_traffic_for_route_checks.pop()
				for other in my_traffic_for_route_checks:
					c = path_conflict_test(acft, other)
					conflicts[acft.identifier] = max(conflicts[acft.identifier], c)
					conflicts[other.identifier] = max(conflicts[other.identifier], c)
		# now update aircraft conflicts and emit signals if any are new
		new_near_miss = new_path_conflict = False
		for contact in self.contacts():
			new_conflict = conflicts[contact.identifier]
			if new_conflict > contact.conflict:
				new_near_miss |= new_conflict == Conflict.NEAR_MISS
				new_path_conflict |= new_conflict in [Conflict.DEPENDS_ON_ALT, Conflict.PATH_CONFLICT]
			contact.conflict = new_conflict
		if new_path_conflict:
			signals.pathConflict.emit()
		if new_near_miss:
			signals.nearMiss.emit()



def bypass_route_conflict_check(strip):
	rules = strip.lookup(FPL.FLIGHT_RULES, fpl=True)
	return settings.route_conflict_traffic == 0 and rules == 'VFR' or settings.route_conflict_traffic == 1 and rules != 'IFR'
