
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

from base.params import Speed


# ---------- Constants ----------

take_off_speed_factor = 1.3 # mult. stall speed
touch_down_speed_factor = 1.1 # mult. stall speed
helos_touch_down_speed = Speed(15)
max_overspeed_prop = 10 # speed over cruising speed
max_overspeed_jet = 25 # speed over cruising speed

stall_speed_factors = {'heavy': .25, 'jets': .22, 'turboprops': .37, 'props': .52} # helos separate (no low-speed stall)
commercial_prob = {'heavy': 1, 'jets': .8, 'turboprops': .2, 'props': 0, 'helos': 0}

# -------------------------------


acft_db = {} # ICAO -> X-plane cat, WTC, cruise speed
acft_registration_formats = []


##  PRONUNCIATION DICTIONARIES: code -> (TTS str, SR phoneme list)  ##

phon_airlines = {}
phon_navpoints = {}


def get_TTS_string(dct, key):
	return dct[key][0]

def get_phonemes(dct, key):
	return dct[key][1]





def all_aircraft_types():
	return set(acft_db.keys())

def all_airline_codes():
	return list(phon_airlines)




def _get_info(t, i):
	try:
		return acft_db[t][i]
	except KeyError:
		return None

def acft_cat(t):
	return _get_info(t, 0)

def wake_turb_cat(t):
	return _get_info(t, 1)

def cruise_speed(t):
	return _get_info(t, 2)

def stall_speed(t):
	cat = acft_cat(t)
	if cat == 'helos':
		return Speed(0)
	else:
		fact = stall_speed_factors.get(cat)
		crspd = cruise_speed(t)
		return None if fact is None or crspd is None else crspd * fact

def maximum_speed(t):
	crspd = cruise_speed(t)
	return None if crspd is None else crspd + (max_overspeed_jet if acft_cat(t) in ['heavy', 'jets'] else max_overspeed_prop)

def take_off_speed(t):
	stall = stall_speed(t)
	return None if stall is None else stall * take_off_speed_factor

def touch_down_speed(t):
	if acft_cat(t) == 'helos':
		return helos_touch_down_speed
	else:
		stall = stall_speed(t)
		return None if stall is None else stall * touch_down_speed_factor
