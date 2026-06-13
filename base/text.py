
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
from sys import stderr

from base.coords import dist_str
from base.fpl import FPL
from base.strip import rack_detail, runway_box_detail, parsed_route_detail, \
		assigned_SQ_detail, assigned_altitude_detail, assigned_heading_detail, assigned_speed_detail
from base.utc import timestr
from base.util import noNone
from base.weather import hPa2inHg

from session.config import settings
from session.env import env


# ---------- Constants ----------

text_alias_prefix = '$'
text_alias_failed_replacement_prefix = '!!'

# -------------------------------


class TextMessage:
	def __init__(self, sender, text, recipient=None, private=False, timeStamp=None):
		self.sent_by = sender
		self.known_recipient = recipient
		self.disp_prefix = None
		self.text = text
		self.private = private
		if recipient is None and private:
			self.private = False
			print('WARNING: Cannot make message private without an identified recipient; made public.', file=stderr)
		self.msg_time_stamp = timeStamp if timeStamp else settings.session_manager.clockTime()

	def setDispPrefix(self, prefix):
		self.disp_prefix = prefix

	def txtOnly(self):
		return self.text

	def txtMsg(self):
		if self.known_recipient is None or self.known_recipient == '' or self.private:
			s = self.text
		else:
			s = '%s: %s' % (self.known_recipient, self.text)
		return s if self.disp_prefix is None else '[%s] %s' % (self.disp_prefix, s)
	
	def isPrivate(self):
		return self.private

	def sender(self):
		return self.sent_by

	def recipient(self):
		return self.known_recipient
	
	def timeStamp(self):
		return self.msg_time_stamp
	
	def isFromMe(self):
		return self.sender() == settings.my_callsign
	
	def involves(self, name):
		return self.sender() == name or self.recipient() == name





def custom_alias_search(alias, text):
	alias_def_regexp = re.compile('%s=(.*)' % re.escape(alias), flags=re.IGNORECASE)
	for line in text.split('\n'):
		match = alias_def_regexp.match(line)
		if match:
			return match.group(1)
	raise ValueError('Alias not resolved: %s%s' % (text_alias_prefix, alias))


def text_alias_replacement(text_alias, current_selection):
	alias = text_alias.lower()
	weather = env.primaryWeather()
	## Check for general alias
	if alias == 'ad':
		noNone(env.airport_data)
		return env.airport_data.navpoint.long_name
	elif alias == 'atis':
		return noNone(settings.last_recorded_ATIS)[0]
	elif alias == 'decl':
		return env.readDeclination()
	elif alias == 'elev':
		return '%d ft' % noNone(env.airport_data).field_elevation
	elif alias == 'frq':
		return str(noNone(settings.publicised_frequency))
	elif alias == 'helipads':
		pads = [pad.name for pad in noNone(env.airport_data).helipads() if pad.inUse()]
		return ', '.join(pads) if pads else 'N/A'
	elif alias == 'icao':
		return settings.location_code
	elif alias == 'me':
		return settings.location_radio_name if settings.location_radio_name else settings.my_callsign
	elif alias == 'metar':
		return noNone(weather).METAR()
	elif alias == 'qfe':
		noNone(env.airport_data)
		return '%d' % env.QFE(noNone(env.QNH(noneSafe=False)))
	elif alias == 'qnh':
		return '%d' % noNone(env.QNH(noneSafe=False))
	elif alias == 'qnhg':
		return '%.2f' % (hPa2inHg * noNone(env.QNH(noneSafe=False)))
	elif alias == 'runways':
		noNone(env.airport_data)
		return env.readRunwaysInUse()
	elif alias == 'rwyarr':
		rwys = [rwy.name for rwy in noNone(env.airport_data).directionalRunways() if rwy.use_for_arrivals]
		if len(rwys) == 0:
			raise ValueError('No RWY marked for arrival')
		return ', '.join(rwys)
	elif alias == 'rwydep':
		rwys = [rwy.name for rwy in noNone(env.airport_data).directionalRunways() if rwy.use_for_departures]
		if len(rwys) == 0:
			raise ValueError('No RWY marked for departure')
		return ', '.join(rwys)
	elif alias == 'ta':
		return '%d ft' % env.transitionAltitude()
	elif alias == 'tl':
		noNone(env.QNH(noneSafe=False))
		return 'FL%03d' % env.transitionLevel()
	elif alias == 'utc':
		return timestr(settings.session_manager.clockTime(), z=True)
	elif alias == 'vis':
		return noNone(weather).readVisibility()
	elif alias == 'wind':
		return noNone(weather).readWind()
	else: ## Check for selection-dependant alias
		strip = current_selection.strip
		acft = current_selection.acft
		if alias == 'cruise':
			return noNone(noNone(strip).lookup(FPL.CRUISE_ALT, fpl=True)).toStr()
		elif alias == 'dest':
			return noNone(noNone(strip).lookup(FPL.ICAO_ARR, fpl=True))
		elif alias == 'dist':
			coords = noNone(acft).coords()
			return dist_str(noNone(env.airport_data).navpoint.coordinates.distanceTo(coords))
		elif alias == 'nseq':
			return str(env.strips.stripSequenceNumber(noNone(strip))) # rightly fails with ValueError if strip is loose
		elif alias == 'qdm':
			coords = noNone(acft).coords()
			return coords.headingTo(noNone(env.airport_data).navpoint.coordinates).read()
		elif alias == 'rack':
			return noNone(noNone(strip).lookup(rack_detail))
		elif alias == 'route':
			return noNone(noNone(strip).lookup(FPL.ROUTE, fpl=True))
		elif alias == 'rwy':
			box = noNone(noNone(strip).lookup(runway_box_detail))
			return env.airport_data.physicalRunwayNameFromUse(box) # code unreachable if env.airport_data is None
		elif alias == 'sq':
			return '%04o' % noNone(noNone(strip).lookup(assigned_SQ_detail))
		elif alias == 'valt':
			return noNone(noNone(strip).lookup(assigned_altitude_detail)).toStr()
		elif alias == 'vhdg':
			return noNone(noNone(strip).lookup(assigned_heading_detail)).read()
		elif alias == 'vspd':
			return str(noNone(noNone(strip).lookup(assigned_speed_detail)))
		elif alias == 'wpnext':
			coords = noNone(acft).coords()
			route = noNone(strip).lookup(parsed_route_detail)
			return str(noNone(route).currentWaypoint(coords))
		elif alias == 'wpsid':
			route = noNone(strip).lookup(parsed_route_detail)
			return noNone(noNone(route).SID())
		elif alias == 'wpstar':
			route = noNone(strip).lookup(parsed_route_detail)
			return noNone(noNone(route).STAR())
		else: ## Check for custom alias, in order: general notes, location-specific notes, selected strip comments
			try:
				return custom_alias_search(alias, settings.general_notes)
			except ValueError:
				try:
					return custom_alias_search(alias, settings.local_notes)
				except ValueError:
					return custom_alias_search(alias, noNone(noNone(strip).lookup(FPL.COMMENTS)))


text_alias_regexp = re.compile(r'%s(\w+)' % re.escape(text_alias_prefix))

def _match_repl_failsafe(alias_match, current_selection):
	try:
		return text_alias_replacement(alias_match.group(1), current_selection)
	except ValueError:
		return text_alias_failed_replacement_prefix + alias_match.group(1)


def replace_text_aliases(text, current_selection, value_error_if_missing):
	if value_error_if_missing:
		repl = lambda match: text_alias_replacement(match.group(1), current_selection)
	else:
		repl = lambda match: _match_repl_failsafe(match, current_selection)
	return text_alias_regexp.sub(repl, text)

