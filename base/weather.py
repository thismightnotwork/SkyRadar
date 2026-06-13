
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

from base.params import Heading
from base.util import linear, m2ft, m2mi


# ---------- Constants ----------

hPa2inHg = 29.92 / 1013.25

gust_diff_threshold = 5 # kt

# QNH regexp groups: 1=unit; 2=value
QNH_regexp = re.compile(r' ([QA])(\d{4})[ =]')

# Visibility regexp
visibility_m_regexp = re.compile(r' (\d{4})[ =]') # CAUTION: pattern can be found in RMK section with different semantics
visibility_SM_regex = re.compile(r'[MP]?((?P<inof>\d{1,2})|(?P<fnoi>\d{1,2}/\d{1,2})|(?P<iwithf>\d{1,2}) (?P<fwithi>\d{1,2}/\d{1,2}))SM[ =]')

# update_time regexp groups: 1=day; 2=time
update_time_regexp = re.compile(r' (\d{2})(\d{4})Z[ =]')

# temperature regexp groups: 1=temperature; 2=dew point
temperatures_regexp = re.compile(r' (M?\d{2})/(M?\d{2})[ =]')

# wind regexps
wind_regexp = re.compile(r' ((?P<hdg>\d{3})(?P<speed_nrm>\d{2})|VRB(?P<speed_vrb>\d{2}))(G(?P<gusts>\d{2}))?(?P<unit>KT|MPS|KMH)[ =]')
wind_hdgvrb_regexp = re.compile(r' (\d{3})V(\d{3})[ =]')

# -------------------------------



def tempF2C(fahrenheit):
	return (fahrenheit - 32) / 1.8

def tempC2F(celsius):
	return (1.8 * celsius) + 32

def stdTempC(alt):
	alt_m = alt.ft1013() / m2ft
	if alt_m < 11000:
		return linear(0, 15, 11000, -56.5, alt_m)
	elif alt_m < 20000:
		return -56.5
	elif alt_m < 32000:
		return linear(20000, -56.5, 32000, -44, alt_m)
	elif alt_m < 47000:
		return linear(32000, -44, 47000, -2.5, alt_m)
	elif alt_m < 51500:
		return -2.5
	else:
		return linear(51500, -2.5, 70000, -54, alt_m)





def METAR_time_str(utc):
	return '%02d%02d%02dZ' % (utc.day, utc.hour, utc.minute)


def temperature_str(temp):
	if temp < 0:
		return 'M%02d' % -temp
	else:
		return '%02d' % temp

def temperature_int(s):
	if s[0] == 'M': # negative temperature
		return -int(s[1:])
	else:
		return int(s)


def mkWeather(station, time, wind='00000KT', vis=9999, clouds='NSC', qnh=1013, temp=15, dp=None):
	if dp is None:
		dp = temp - 5
	metar = '%s %s %s' % (station, METAR_time_str(time), wind)
	if vis >= 9999 and clouds == 'NSC':
		metar += ' CAVOK'
	else:
		metar += ' %04d %s' % (min(vis, 9999), clouds)
	metar += ' %s/%s' % (temperature_str(temp), temperature_str(dp))
	metar += ' Q%04d' % qnh
	return Weather(metar + '=')




class Weather:
	def __init__(self, metar):
		self.METAR_string = metar
	
	## ACCESS
	
	def METAR(self):
		return self.METAR_string
	
	def station(self):
		return self.METAR_string.split(' ', maxsplit=1)[0]

	def QNH(self):
		"""
		returns the sea level pressure (QNH) in hPa
		"""
		match = QNH_regexp.search(self.METAR_string)
		if match:
			if match.group(1) == 'Q': # num in hPa
				return int(match.group(2))
			elif match.group(1) == 'A': # num in 100*inch_Hg
				return int(match.group(2)) / 100 / hPa2inHg
		return None
	
	def mainWind(self):
		"""
		Returns a (heading, speed, gusts, unit) tuple, where:
		- heading & speed values are:
			- None, 0: wind is calm
			- None, int: wind is VRB
			- Heading, int: wind has dominant direction
		- and:
			- gusts is None or int speed
			- unit is either 'kt', 'm/s' or 'km/h', following that used in the METAR
		"""
		match = wind_regexp.search(self.METAR_string)
		if match:
			if match.group('gusts') is None:
				gusts = None
			else:
				gusts = int(match.group('gusts'))
			unit = {'KT': 'kt', 'MPS': 'm/s', 'KMH': 'km/h'}[match.group('unit')]
			vrbspd = match.group('speed_vrb')
			if vrbspd is None: # got dominant direction
				wind_speed = int(match.group('speed_nrm'))
				return (Heading(int(match.group('hdg')), True) if wind_speed != 0 else None), wind_speed, gusts, unit
			else: # wind is VRB
				return None, int(vrbspd), gusts, unit
		return None
	
	def prevailingVisibility(self):
		"""
		Returns one of the following pairs:
		- int metres, None     (vis. was in metres; 9999 means > 10 km, either from "9999" or "CAVOK")
		- float metres, str SM (vis. was in miles; float ignores the M/P prefix; str is the original matched string, incl. "SM" suffix)
		- None, None           (info not present)
		"""
		match = visibility_m_regexp.search(self.METAR_string.split('RMK', maxsplit=1)[0])
		if match:
			return int(match.group(1)), None
		match = visibility_SM_regex.search(self.METAR_string)
		if match:
			i = match.group('inof')
			f = match.group('fnoi')
			if i is None and f is None:
				i = match.group('iwithf')
				f = match.group('fwithi')
			vis = 0
			if i is not None:
				vis += int(i)
			if f is not None:
				f1, f2 = f.split('/', maxsplit=1)
				vis += int(f1) / int(f2)
			return vis / m2mi, match.group(0)[:-1]
		if 'CAVOK' in self.METAR_string:
			return 9999, None
		return None, None
	
	def windVariability(self):
		"""
		returns wind variability (Heading, Heading) if present, None otherwise
		"""
		match = wind_hdgvrb_regexp.search(self.METAR_string)
		if match:
			return Heading(int(match.group(1)), False), Heading(int(match.group(2)), False)
		return None
	
	def temperatures(self):
		"""
		returns (temperature, dew point) int pair, None if not available
		"""
		match = temperatures_regexp.search(self.METAR_string)
		if match:
			return temperature_int(match.group(1)), temperature_int(match.group(2))
		return None
	
	def updateTimeStr(self, timeOnlyIfShortBefore=None):
		"""
		If METAR recent enough before a given ref, returns a human-readable time string.
		Otherwise returns the 'Z'-suffixed METAR timestamp.
		"""
		match = update_time_regexp.search(self.METAR_string)
		if match:
			day_update = int(match.group(1))
			timestr_update = match.group(2)
			if timeOnlyIfShortBefore is not None:
				day_ref = timeOnlyIfShortBefore.day
				timestr_ref = '%02d%02d' % (timeOnlyIfShortBefore.hour, timeOnlyIfShortBefore.minute)
				if day_update == day_ref and timestr_update <= timestr_ref or day_update == day_ref - 1 and timestr_update > timestr_ref:
					return '%s:%s' % (timestr_update[:2], timestr_update[2:])
			return '%s%sZ' % (match.group(1), timestr_update)
		return None
	
	def isNewerThan(self, prev):
		"""
		NB: only time is compared (no check for identical source stations)
		"""
		match_A = update_time_regexp.search(self.METAR_string)
		match_B = update_time_regexp.search(prev.METAR_string)
		if match_A and match_B:
			day_A, time_A = int(match_A.group(1)), int(match_A.group(2))
			day_B, time_B = int(match_B.group(1)), int(match_B.group(2))
			if day_A == day_B:
				return time_A >= time_B and self.METAR_string != prev.METAR_string # consider self newer if times are equal (useful in tutorial sessions)
			else:
				return day_A < day_B and day_B - day_A > 15 or day_A > day_B and day_A - day_B < 15
		else:
			return match_A and not match_B # at least prefer a readable time stamp
	
	
	## Reading
	
	def readWind(self):
		wind_info = self.mainWind()
		if wind_info is None:
			return 'N/A'
		whdg, wspd, gusts, unit = wind_info
		if whdg is None and wspd == 0:
			txt = 'calm'
		else:
			if whdg is None:
				txt = 'variable'
			else: # got heading
				vrb = self.windVariability()
				if vrb is None:
					txt = '%s°' % whdg.rounded(False, step=10).read()
				else:
					txt = '%s-%s°' % (vrb[0].read(), vrb[1].read())
			txt += ', %d %s' % (wspd, unit)
		if gusts is not None:
			txt += ', gusting %d %s' % (gusts, unit)
		return txt
	
	def readVisibility(self):
		vis_m, str_SM = self.prevailingVisibility()
		if vis_m is None:
			return 'N/A'
		elif str_SM is None: # metres only
			if vis_m == 9999:
				return '> 10 km'
			elif vis_m % 1000 == 0:
				return '%d km' % (vis_m // 1000)
			elif vis_m < 1000:
				return '%d m' % vis_m
			else:
				return '%d,%03d m' % (vis_m // 1000, vis_m % 1000)
		else: # statute miles
			if str_SM[0] in 'MP':
				return '%s %s mi' % ('<>'[str_SM[0] == 'P'], str_SM[1:-2])
			else:
				return '%s mi' % str_SM[:-2]
