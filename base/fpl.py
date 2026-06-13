
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
from datetime import datetime, timedelta, timezone

from base.params import AltFlSpec, Speed
from base.utc import rel_session_datetime_str
from base.util import some

from session.config import settings


# ---------- Constants ----------

outdated_DEP_delay = timedelta(hours=6)

# -------------------------------

# CALLSIGN      str
# ACFT_TYPE     str
# WTC           str
# ICAO_DEP      str
# ICAO_ARR      str
# ICAO_ALT      str
# CRUISE_ALT    AltFlSpec
# TAS           Speed
# SOULS         int
# TIME_OF_DEP   datetime
# EET           timedelta
# FLIGHT_RULES  str
# ROUTE         str
# COMMENTS      str



class FPL:
	# STATIC stuff
	statuses = FILED, OPEN, CLOSED = range(3)
	details = CALLSIGN, ACFT_TYPE, WTC, ICAO_DEP, ICAO_ARR, ICAO_ALT, \
		CRUISE_ALT, TAS, SOULS, TIME_OF_DEP, EET, FLIGHT_RULES, ROUTE, COMMENTS = range(14)
	
	detailStrNames = {
		CALLSIGN: 'callsign',
		ACFT_TYPE: 'ACFT type',
		WTC: 'WTC',
		ICAO_DEP: 'origin',
		ICAO_ARR: 'destination',
		ICAO_ALT: 'alternate',
		CRUISE_ALT: 'cruise alt.',
		TAS: 'TAS',
		SOULS: 'POB',
		TIME_OF_DEP: 'DEP time',
		EET: 'EET',
		FLIGHT_RULES: 'rules',
		ROUTE: 'route',
		COMMENTS: 'comments'
	}
	# End STATIC
	
	def __init__(self, details=None):
		self.online_id = None
		self.online_status = None # normally None only if FPL is not online
		self.modified_details = {} # detail modified locally -> online value to revert to if local changes reverted
		self.details = {detail: None for detail in FPL.details}
		if details is not None:
			self.details.update(details)
		self.strip_auto_printed = False
	
	def __str__(self):
		if self.online_id is None:
			return 'Local-%X' % id(self)
		else:
			return 'Online-%s' % self.online_id

	def encode(self):
		if self.isOnline():
			header_section = '%s %s' % (self.online_id, {FPL.FILED: 'F', FPL.OPEN: 'O', FPL.CLOSED: 'C'}[self.onlineStatus()])
		else:
			header_section = ''
		sections = [header_section]
		for d, v in self.details.items():
			if v is not None:
				try:
					sections.append('%s %s' % (d, detail2str(d, v)))
				except ValueError as err:
					print('ERROR: %s' % err, file=stderr)
		return r'\\'.join(s.replace('\\', r'\ ').replace('\n', r'\n') for s in sections)

	@staticmethod
	def fromEncoded(encoded_details):
		'''
		Details are encoded with format: [<online id> [<online status>]] (r"\\" <key> <value>)*
		<online status> is "O" or "C" is it is open or closed, respectively
		<value> can contain spaces
		'''
		fpl = FPL()
		split_feed = encoded_details.split(r'\\')
		hd_split = split_feed.pop(0).split()
		if len(hd_split) == 0:
			pass
		elif len(hd_split) == 2:
			fpl.markAsOnline(hd_split[0])
			try:
				fpl.setOnlineStatus({'F': FPL.FILED, 'O': FPL.OPEN, 'C': FPL.CLOSED}[hd_split[1]])
			except KeyError as err:
				print('ERROR: Unknown FPL status "%s"' % err, file=stderr)
		for encoded_detail in split_feed:
			unescaped = encoded_detail.replace(r'\n', '\n').replace(r'\ ', '\\')
			tokens = unescaped.split(maxsplit=1)
			if len(tokens) == 0:
				continue # Ignore empty detail sections. Normally happens only if FPL has no details at all.
			try:
				d = int(tokens[0])
				fpl.details[d] = str2detail(d, '' if len(tokens) < 2 else tokens[1])
			except ValueError as err:
				print('ERROR: %s' % err, file=stderr)
		return fpl
		
	
	## ACCESS
	
	def __getitem__(self, detail):
		assert detail in FPL.details, 'Not a valid flight plan detail key'
		return self.details[detail]
	
	def isOnline(self):
		return self.online_id is not None
	
	def hasLocalChanges(self):
		return len(self.modified_details) > 0
	
	def onlineStatus(self):
		return self.online_status
	
	def isOutdated(self):
		dep = self[FPL.TIME_OF_DEP]
		return self.onlineStatus() == FPL.FILED and dep is not None and settings.session_manager.clockTime() > dep + outdated_DEP_delay
	
	
	## QUERY
	
	def ETA(self):
		dep = self.details[FPL.TIME_OF_DEP]
		eet = self.details[FPL.EET]
		return dep + eet if dep is not None and eet is not None else None
	
	def flightIsInTimeWindow(self, time_window, ref=None, strict=False):
		dep = self.details[FPL.TIME_OF_DEP]
		if dep is None:
			return False
		if ref is None:
			ref = settings.session_manager.clockTime()
		lo = ref - time_window / 2
		hi = ref + time_window / 2
		eta = some(self.ETA(), dep)
		if strict:
			return dep >= lo and eta <= hi
		else:
			return dep <= hi and eta >= lo
	
	def onlineStatusStr(self):
		if self.isOnline():
			return {FPL.FILED: 'filed', FPL.OPEN: 'open', FPL.CLOSED: 'closed'}.get(self.onlineStatus(), '!!unknown')
		else:
			return 'not online'
	
	def shortDescr(self):
		return '%s, %s, %s' % (some(self.details[FPL.CALLSIGN], '?'), self.shortDescr_AD(), self.shortDescr_time())
	
	def shortDescr_AD(self):
		dep = self.details[FPL.ICAO_DEP]
		arr = self.details[FPL.ICAO_ARR]
		if dep is not None and dep == arr:
			return '%s local' % dep
		else:
			return '%s to %s' % (some(dep, '?'), some(arr, '?'))
	
	def shortDescr_time(self):
		if self.onlineStatus() == FPL.OPEN:
			eta = self.ETA()
			if eta is not None:
				return 'ARR %s' % rel_session_datetime_str(eta, longFormat=True)
		tdep = self.details[FPL.TIME_OF_DEP]
		return '?' if tdep is None else 'DEP %s' % rel_session_datetime_str(tdep, longFormat=True)
	
	
	## MODIFY
	
	def markAsOnline(self, online_id):
		self.online_id = online_id
		self.online_status = FPL.FILED
	
	def setOnlineStatus(self, status):
		self.online_status = status

	def __setitem__(self, detail, new_value):
		assert detail in FPL.details, 'Incorrect FPL detail key: %s' % detail
		if isinstance(new_value, str) and new_value == '':
			new_value = None
		old_value = self.details[detail]
		if new_value != old_value:
			self.details[detail] = new_value
			if self.isOnline() and detail not in self.modified_details:
				self.modified_details[detail] = old_value
	
	def revertToOnlineValues(self):
		for d, v in self.modified_details.items():
			self.details[d] = v
		self.modified_details.clear()




## DETAIL STRING CONVERSIONS

def detail2str(d, v):
	if d in [FPL.CALLSIGN, FPL.ACFT_TYPE, FPL.WTC, FPL.ICAO_DEP, FPL.ICAO_ARR, FPL.ICAO_ALT,
			FPL.ROUTE, FPL.COMMENTS, FPL.FLIGHT_RULES]: # str
		return v
	elif d == FPL.SOULS: # int
		return str(v)
	elif d == FPL.TAS: # Speed
		return str(int(v.kt()))
	elif d == FPL.TIME_OF_DEP: # datetime
		return '%d %d %d %d %d' % (v.year, v.month, v.day, v.hour, v.minute)
	elif d == FPL.EET: # timedelta
		return str(int(v.total_seconds()))
	elif d == FPL.CRUISE_ALT: # AltFlSpec
		return v.toStr(unit=False) # avoid spaces
	else:
		raise ValueError('Unknown key for detail conversion: %s' % d)


def str2detail(d, vstr):
	if d in [FPL.CALLSIGN, FPL.ACFT_TYPE, FPL.WTC, FPL.ICAO_DEP, FPL.ICAO_ARR, FPL.ICAO_ALT,
			FPL.ROUTE, FPL.COMMENTS, FPL.FLIGHT_RULES]: # str
		v = vstr
	elif d == FPL.SOULS: # int
		v = int(vstr)
	elif d == FPL.TAS: # Speed
		v = Speed(int(vstr))
	elif d == FPL.TIME_OF_DEP: # datetime
		year, month, day, hour, minute = vstr.split()
		v = datetime(year=int(year), month=int(month), day=int(day),
						hour=int(hour), minute=int(minute), tzinfo=timezone.utc)
	elif d == FPL.EET: # timedelta
		v = timedelta(seconds=int(vstr))
	elif d == FPL.CRUISE_ALT: # AltFlSpec
		v = AltFlSpec.fromStr(vstr)
	else:
		raise ValueError('Unknown key for detail conversion: %s' % d)
	return v
