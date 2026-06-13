
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
from math import radians, degrees, pi, cos, sin, acos, asin, atan, atan2, sqrt

from PyQt5.QtCore import QPointF
from PyQt5.QtGui import QVector2D

from base.params import Heading
from base.util import m2NM, m2ft


# ---------- Constants ----------

Earth_radius_km = 6366.71
Earth_radius_NM = m2NM * 1000 * Earth_radius_km

# NOTE: d/m/s allowed below instead of °/'/'' to enable bypass of any encoding problems, esp. with '°'
degMinSec_regexp = re.compile('(\\d+)[°d]((\\d+)[\'m](([0-9.]+)(\'\'|"|s)?)?)?[NSEW]')

default_breakUp_segment_length = 8 # NM

# -------------------------------


def dist_str(distance):
	return '%s NM' % ('%.1f' if distance < 10 else '%d') % distance




def pitchLookAt(dist, alt_diff):
	"""
	Returns pitch angle in degrees when looking at a point.
	"""
	return degrees(atan((alt_diff / m2ft) / (dist / m2NM)))



def deg_min_sec(decimal):
	sgn = decimal >= 0
	decimal = abs(decimal)
	d = int(decimal)
	decimal -= d
	m = int(60 * decimal)
	decimal -= m / 60
	s = 3600 * decimal
	return sgn, d, m, s





def breakUpLine(p1, p2, segmentLength=default_breakUp_segment_length):
	length = p1.distanceTo(p2)
	if int(length / segmentLength) == 0:
		return [(p1, p2)]
	else:
		intermediate = p1.moved(p1.headingTo(p2), segmentLength)
		return [(p1, intermediate)] + breakUpLine(intermediate, p2, segmentLength)












#--------------------------------------------#
#                                            #
#       Cartesian positioning on radar       #
#                                            #
#--------------------------------------------#


class RadarCoords(QPointF):
	"""
	cartesian with NM unit, centre at radar position, and inverted Y axis
	"""
	def __init__(self, x, y):
		QPointF.__init__(self, x, y)
	
	@staticmethod
	def fromQPointF(p):
		return RadarCoords(p.x(), p.y())
	
	def toQPointF(self):
		return QPointF(self.x(), self.y())
	
	
	def headingTo(self, other):
		"""
		this gives the heading of route from self to other, using cartesian approx. of both radar coords
		"""
		dx = other.x() - self.x()
		dy = other.y() - self.y()
		theta = atan2(dx, dy)
		return Heading(degrees(pi - theta), True)
	
	def headingFrom(self, other):
		"""
		this gives the heading of route from other to self, using cartesian approx. of both radar coords
		"""
		return other.headingTo(self)
	
	def distanceTo(self, other):
		dx = other.x() - self.x()
		dy = other.y() - self.y()
		return sqrt(dx*dx + dy*dy)
	
	def moved(self, radial, distance):
		a = pi/2 - radians(radial.trueAngle())
		dx = distance * cos(a)
		dy = distance * sin(-a) # inverted Y axis
		return RadarCoords(self.x() + dx, self.y() + dy)

	def isBetween(self, p1, p2, max_offset, offsetBeyondEnds=False):
		if offsetBeyondEnds and any(self.distanceTo(p) <= max_offset for p in (p1, p2)):
			return True
		q = self.toQPointF()
		q1 = p1.toQPointF()
		q2 = p2.toQPointF()
		vp1p2 = QVector2D(q2 - q1)
		return QVector2D.dotProduct(vp1p2, QVector2D(q - q1)) >= 0 \
			and QVector2D.dotProduct(vp1p2, QVector2D(q - q2)) <= 0 \
			and QVector2D(q).distanceToLine(QVector2D(q1), vp1p2.normalized()) <= max_offset

	def orthProj(self, p1, p2):
		q1 = p1.toQPointF()
		q2 = p2.toQPointF()
		vdir = QVector2D(q2 - q1).normalized()
		vq1q = QVector2D(self.toQPointF() - q1)
		vq1h = QVector2D.dotProduct(vq1q, vdir) * vdir
		return RadarCoords.fromQPointF(q1 + vq1h.toPointF())







#---------------------------------------------#
#                                             #
#     Earth positioning (GPS-type coords)     #
#                                             #
#---------------------------------------------#

class Format:
	DEG_MIN_SEC, DEG_DECMIN, DECDEG = range(3)


def read_coord(s):
	"""
	Returns (deg, typ) where:
	- deg is the value in degrees
	- typ is either 'dec' (was decimal), 'lat' (ending in 'N' or 'S') or 'lon' ('E' or 'W')
	"""
	match = degMinSec_regexp.fullmatch(s)
	if match: # Deg-min-sec format
		decdeg = float(match.group(1)) # degrees
		if match.group(3) is not None: # minutes
			decdeg += float(match.group(3)) / 60
			if match.group(5) is not None: # seconds
				decdeg += float(match.group(5)) / 3600
		return (decdeg if s[-1] in 'NE' else -decdeg), ('lon' if s[-1] in 'EW' else 'lat')
	else: # Decimal degrees
		return float(s), 'dec'



class EarthCoords:
	preferred_format = Format.DEG_MIN_SEC
	# for conversions to radar coords, to be updated:
	ref_pos = None
	lat_1deg = None
	lon_1deg = None
	
	@staticmethod
	def setRadarPos(coords):
		EarthCoords.ref_pos = coords
		EarthCoords.lat_1deg = EarthCoords(coords.lat - .5, coords.lon).distanceTo(EarthCoords(coords.lat + .5, coords.lon))
		EarthCoords.lon_1deg = EarthCoords(coords.lat, coords.lon - .5).distanceTo(EarthCoords(coords.lat, coords.lon + .5))
	
	@staticmethod
	def clearRadarPos():
		EarthCoords.ref_pos = None
	
	@staticmethod
	def getRadarPos():
		return EarthCoords.ref_pos
	
	@staticmethod
	def fromRadarCoords(coords):
		orig = RadarCoords(0, 0)
		dist = orig.distanceTo(coords)
		hdg = orig.headingTo(coords)
		return EarthCoords.ref_pos.moved(hdg, dist)
	
	@staticmethod
	def fromString(s):
		tokens = s.split(',')
		if len(tokens) == 2:
			c1, t1 = read_coord(tokens[0])
			c2, t2 = read_coord(tokens[1])
			if t1 == 'dec' and t2 == 'dec' or t1 == 'lat' and t2 == 'lon':
				return EarthCoords(c1, c2)
			elif t1 == 'lon' and t2 == 'lat':
				return EarthCoords(c2, c1)
		raise ValueError('Bad coordinate format: %s' % s)

	def __init__(self, lat, lon):
		self.lat = lat
		self.lon = lon
	
	def __str__(self):
		"""
		Pretty and readable but might break if parsed with EarthCoords.fromString
		"""
		s = self.toString(fmt=EarthCoords.preferred_format)
		return s.replace(',', {Format.DEG_MIN_SEC: ' ', Format.DECDEG: ', '}[EarthCoords.preferred_format])
	
	def toString(self, fmt=Format.DECDEG):
		"""
		The result is reversible with EarthCoords.fromString
		"""
		if fmt == Format.DEG_MIN_SEC:
			lat_pos, lat_d, lat_m, lat_s = deg_min_sec(self.lat)
			lon_pos, lon_d, lon_m, lon_s = deg_min_sec(self.lon)
			n_s = 'SN'[lat_pos]
			e_w = 'WE'[lon_pos]
			return '%d°%d\'%.2f\'\'%c,%d°%d\'%.2f\'\'%c' % (lat_d, lat_m, lat_s, n_s, lon_d, lon_m, lon_s, e_w)
		elif fmt == Format.DECDEG:
			return '%.8f,%.8f' % (self.lat, self.lon)
		else:
			raise ValueError('Unimplemented format for coordinates')
	
	def toRadarCoords(self):
		x = EarthCoords.lon_1deg * (self.lon - EarthCoords.ref_pos.lon)
		y = EarthCoords.lat_1deg * (EarthCoords.ref_pos.lat - self.lat) # this inverts the Y axis for use with Qt coordinate system
		return RadarCoords(x, y)
	
	def toQPointF(self): # shortcut
		return self.toRadarCoords().toQPointF()
	
	def distanceTo(self, other):
		lat1 = radians(self.lat)
		lat2 = radians(other.lat)
		dlon = radians(other.lon - self.lon)
		try:
			return acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(dlon)) * Earth_radius_NM
		except ValueError: # Caught as every time was only acos of value just over 1, probably from non-critical approximations
			return 0 # acos(1)
	
	def headingTo(self, other):
		"""
		this gives the initial heading of route from self to other
		that follows the shortest path (on great circle, i.e. as the crow flies)
		"""
		lat1 = radians(self.lat)
		lat2 = radians(other.lat)
		dlon = radians(other.lon - self.lon)
		theta = atan2(sin(dlon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon))
		return Heading(degrees(theta), True)
	
	def headingFrom(self, other):
		"""
		this gives the final heading of route from other to self
		that follows the shortest path (on great circle, i.e. as the crow flies)
		"""
		return self.headingTo(other).opposite()
	
	def moved(self, radial, distance):
		"""
		this gives the final position of a crow's flight starting with a given heading
		"""
		lat1 = radians(self.lat)
		lon1 = radians(self.lon)
		a = radians(radial.trueAngle())
		d = distance / Earth_radius_NM
		lat2 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(a))
		lon2 = lon1 + atan2(sin(a) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat2))
		lat_res = (degrees(lat2) + 90) % 180 - 90
		lon_res = (degrees(lon2) + 180) % 360 - 180
		return EarthCoords(lat_res, lon_res)











## ======= WGS84 geodesy =======

# translated from simgear C++ sources

WGS84_equrad = 6378137
WGS84_squash = .9966471893352525192801545

ra2 = 1 / (WGS84_equrad * WGS84_equrad)
e2 = abs(1 - WGS84_squash * WGS84_squash)
e4 = e2 * e2

def WGS84_geodetic_to_cartesian_metres(geodetic, ft_amsl):
	"""
	Earth centred cartesian coordinates from geodetic coordinates on the WGS84 ellipsoid
	Translated from Simgear sources: simgear/math/SGGeodesy.cxx
	"""
	lam = radians(geodetic.lon)
	phi = radians(geodetic.lat)
	h = ft_amsl / m2ft
	sphi = sin(phi)
	n = WGS84_equrad / sqrt(1 - e2 * sphi * sphi)
	cphi = cos(phi)
	slambda = sin(lam)
	clambda = cos(lam)
	x = (h + n) * cphi * clambda
	y = (h + n) * cphi * slambda
	z = (h + n - e2 * n) * sphi
	return x, y, z
	
	
	
def cartesian_metres_to_WGS84_geodetic(x, y, z):
	"""
	Geodetic coordinates on the WGS84 ellipsoid from cartesian coordinates
	Translated from Simgear sources: simgear/math/SGGeodesy.cxx
	returns coordinates and sea level altitude in feet
	"""
	XXpYY = x*x + y*y
	if XXpYY + z*z < 25:
		return EarthCoords(0, 0), -WGS84_equrad
	sqrtXXpYY = sqrt(XXpYY)
	p = XXpYY * ra2
	q = z*z * (1 - e2) * ra2
	r = 1 / 6 * (p + q - e4)
	s = e4 * p * q / (4 * r * r * r)
	if -2 <= s <= 0:
		s = 0.0
	t = pow(1 + s + sqrt(s * (2 + s)), 1/3)
	u = r * (1 + t + 1/t)
	v = sqrt(u * u + e4 * q)
	w = e2 * (u + v - q) / (2 * v)
	k = sqrt(u + v + w*w) - w
	d = k * sqrtXXpYY / (k + e2)
	sqrtDDpZZ = sqrt(d*d + z*z)
	lon = 2 * atan2(y, x + sqrtXXpYY)
	lat = 2 * atan2(z, d + sqrtDDpZZ)
	m_amsl = (k + e2 - 1) * sqrtDDpZZ/k
	return EarthCoords(degrees(lat), degrees(lon)), m2ft * m_amsl
