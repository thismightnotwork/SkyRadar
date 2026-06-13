
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

from datetime import datetime, timezone

from base.util import some

from session.config import settings


# ---------- Constants ----------

# -------------------------------

def realTime():
	return datetime.now(timezone.utc)


def timestr(t, seconds=False, z=False):
	res = ('%02d%02d' if z else '%02d:%02d') % (t.hour, t.minute)
	if seconds:
		res += ':%02d' % t.second
	if z:
		res += 'Z'
	return res

def datestr(t, year=True):
	res = '%d/%02d' % (t.day, t.month)
	if year:
		res += '/%d' % t.year
	return res

def duration_str(td):
	seconds = int(round(td.total_seconds()))
	hours = seconds // 3600
	if hours == 0:
		return '%d min %02d s' % (seconds // 60, seconds % 60)
	else:
		return '%d h %02d min' % (hours, seconds % 3600 // 60)


def rel_session_datetime_str(dt, longFormat=False, seconds=False):
	dref = settings.session_manager.clockTime()
	if dt.date() == dref.date():
		prefix = 'today at ' if longFormat else ''
	else: # not today
		prefix = datestr(dt, year=(dt.year != dref.year)) + (' at ' if longFormat else ', ')
	return prefix + timestr(dt.time(), seconds=seconds)


program_started_at = realTime()



class VirtualClock:
	"""
	variable speed, continuous relationship with real time (no ticking),
	with ability to pause/resume and set clock on virtual timeline
	"""
	def __init__(self, startPausedAt=None):
		now = realTime()
		self.beacon = now, some(startPausedAt, now)
		self.paused = startPausedAt is not None
		self.real_time_factor = 1

	def _setRealTimeBeacon(self, virtual_time):
		self.beacon = realTime(), virtual_time

	def isPaused(self):
		return self.paused

	def pause(self):
		if not self.isPaused():
			self._setRealTimeBeacon(self.readTime())
			self.paused = True

	def resume(self):
		if self.isPaused():
			self._setRealTimeBeacon(self.readTime())
			self.paused = False

	def readTime(self):
		if self.isPaused():
			return self.beacon[1]
		else:
			return self.beacon[1] + self.real_time_factor * (realTime() - self.beacon[0])

	def setTime(self, new_time):
		self._setRealTimeBeacon(new_time)

	def offsetTime(self, offset):
		self._setRealTimeBeacon(self.readTime() + offset)

	def setTimeFactor(self, f):
		self._setRealTimeBeacon(self.readTime())
		self.real_time_factor = f

	def encodeTime(self):
		return str(self.readTime().timestamp())

	def setTimeEncoded(self, s):
		self.setTime(datetime.fromtimestamp(float(s), timezone.utc))
