
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

from datetime import timedelta
from math import sqrt, cos, sin, atan2, radians, degrees

from base.utc import duration_str
from base.util import rounded, m2NM, m2mi

from session.config import settings


# ---------- Constants ----------

min_significant_TTF_speed = 10 # kt

# -------------------------------


#--------------------------------------------#
#                                            #
#                 Headings                   #
#                                            #
#--------------------------------------------#

class Heading:
	def __init__(self, deg, true_hdg):
		"""
		deg is float angle in degrees with 0/360 North and counting clockwise
		true_hdg is a bool to choose between a true heading (True) or a magnetic heading (False)
		"""
		self.is_true = true_hdg
		self.deg_angle = deg % 360
	
	def __add__(self, a):
		return Heading(self.deg_angle + a, self.is_true)
	
	def trueAngle(self):
		return self.deg_angle if self.is_true else self.deg_angle + settings.magnetic_declination
	
	def magneticAngle(self):
		return self.deg_angle - settings.magnetic_declination if self.is_true else self.deg_angle
	
	def opposite(self):
		return Heading(self.deg_angle + 180, self.is_true)
		
	def read(self):
		"""
		as would be read by/to a pilot, returns a string
		"""
		return '%03d' % ((self.magneticAngle() - 1) % 360 + 1)
		
	def readTrue(self):
		return '%03d' % ((self.trueAngle() - 1) % 360 + 1)
	
	def diff(self, other, tolerance=0):
		diff = (self.trueAngle() - other.trueAngle() + 180) % 360 - 180
		if abs(diff) <= tolerance:
			return 0
		else:
			return diff
	
	def rounded(self, true_hdg, step=5):
		return Heading(rounded((self.trueAngle() if true_hdg else self.magneticAngle()), step), true_hdg)
	
	def approxCardinal(self, true):
		h = (self.trueAngle() if true else self.magneticAngle()) % 360
		return ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][next((i for i in range(8) if h < 45 * i + 22.5), 0)]




#---------------------------------------------#
#                                             #
#                 Altitudes                   #
#                                             #
#---------------------------------------------#


class AltFlSpec:
	def __init__(self, is_FL, int_value):
		self.is_FL = is_FL
		self.int_value = int_value
	
	@staticmethod
	def fromStr(s):
		"""
		"FL" prefix (or "F") => flight level
		"ft" suffix => altitude AMSL
		digits only => FL if up to 3 digits, else altitude
		"""
		if len(s) > 0 and s[0].upper() == 'F' or len(s) <= 3 and s.isdigit():
			return AltFlSpec(True, int(s.lstrip('flFL '))) # can raise ValueError
		else: # altitude in ft
			return AltFlSpec(False, int(s.rstrip('ftFT '))) # can raise ValueError

	def __eq__(self, other):
		if isinstance(other, str):
			try:
				other = AltFlSpec.fromStr(other)
			except ValueError:
				return False
		return isinstance(other, AltFlSpec) and self.is_FL == other.is_FL and self.int_value == other.int_value
	
	def isFL(self):
		return self.is_FL
	
	def plusHundredsFt(self, hft):
		return AltFlSpec(self.is_FL, self.int_value + hft * (1 if self.is_FL else 100))
	
	def toStr(self, unit=True):
		return 'FL%03d' % self.int_value if self.is_FL else '%d%s' % (self.int_value, (' ft' if unit else ''))
	
	def toPressureAlt(self, qnh):
		return PressureAlt.fromFL(self.int_value) if self.is_FL else PressureAlt.fromAMSL(self.int_value, qnh)



class PressureAlt:
	"""
	The class for pressure-altitudes, as are reported by a transponder
	"""
	
	@staticmethod
	def fromFL(fl):
		return PressureAlt(100 * fl)
	
	@staticmethod
	def fromAMSL(ftAMSL, qnh):
		"""
		Gives the XPDR/std pressure-alt of an altitude above sea level, measured in given MSL pressure conditions.
		"""
		return PressureAlt(ftAMSL + 28 * (1013.25 - qnh))
	
	def __init__(self, ft):
		self._ft = ft
	
	def __add__(self, ft):
		return PressureAlt(self.ft1013() + ft)
	
	def __sub__(self, ft):
		return self + -ft
	
	def ft1013(self):
		return self._ft
	
	def ftAMSL(self, qnh):
		return self._ft + 28 * (qnh - 1013.25)
	
	def FL(self):
		return rounded(self._ft / 100)

	def diff(self, other, tolerance=0):
		"""
		'other' must be a PressureAlt
		"""
		diff = self.ft1013() - other.ft1013()
		if abs(diff) <= tolerance:
			return 0
		else:
			return diff




#------------------------------------------#
#                                          #
#                 Speeds                   #
#                                          #
#------------------------------------------#

class Speed:
	"""
	A class for HORIZONTAL speeds, typically measured in knots
	"""
	def __init__(self, v, unit='kt'):
		if unit == 'kt':
			self._kt = v
		elif unit == 'm/s':
			self._kt = m2NM * v * 3600
		elif unit == 'km/h':
			self._kt = m2NM * v * 1000
		elif unit == 'mi/h':
			self._kt = m2NM * v / m2mi
		else:
			raise ValueError('Unknown unit: %s' % unit)
		
	def __str__(self):
		return '%d kt' % rounded(self._kt)
	
	def __add__(self, d):
		return Speed(self._kt + d)
	
	def __sub__(self, d):
		return self + -d

	def __mul__(self, fact):
		return Speed(self._kt * fact)

	def __truediv__(self, fact):
		return self * (1 / fact)
	
	def __eq__(self, other):
		try:
			return self._kt == other._kt
		except AttributeError:
			return False

	def kt(self):
		return self._kt

	def mps(self):
		return self._kt / m2NM / 3600

	def inUnit(self, unit):
		if unit == 'km/h':
			return self._kt / m2NM / 1000
		elif unit == 'mi/h':
			return self._kt / m2NM * m2mi
		else:
			raise ValueError('Unknown unit: %s' % unit)
	
	def rounded(self, step=10):
		return Speed(rounded(self._kt, step))
	
	def diff(self, other, tolerance=0):
		diff = self._kt - other._kt
		if abs(diff) <= tolerance:
			return 0
		else:
			return diff
	
	def ias2tas(self, alt):
		return self * (1 + 2e-5 * alt.ft1013()) # 2% TAS increase per thousand ft AMSL
	
	def tas2ias(self, alt):
		return self / (1 + 2e-5 * alt.ft1013())




def wind_effect(acft_hdg, acft_tas, wind_from_hdg, wind_speed):
	"""
	return (course, ground speed) pair from ACFT heading and TAS
	"""
	hdg = radians(acft_hdg.magneticAngle())
	tas = acft_tas.kt()
	wd = radians(wind_from_hdg.magneticAngle())
	ws = wind_speed.kt()
	ground_speed = sqrt(ws*ws + tas*tas - 2 * ws * tas * cos(hdg - wd))
	wca = atan2(ws * sin(hdg - wd), tas - ws * cos(hdg - wd))
	return Heading(degrees(hdg + wca), False), Speed(ground_speed)
	


def time_to_fly(dist, speed):
	if speed.kt() < min_significant_TTF_speed:
		raise ValueError('Speed too low')
	return timedelta(hours=(dist / speed.kt()))



def distance_travelled(time, speed):
	"""
	time is a timedelta object
	"""
	return time / timedelta(hours=1) * speed.kt()



def TTF_str(dist, speed):
	return duration_str(time_to_fly(dist, speed))
