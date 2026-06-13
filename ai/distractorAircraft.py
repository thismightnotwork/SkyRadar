
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

from ai.aircraft import AbstractAiAcft
from ai.status import Status

from base.params import distance_travelled


# ---------- Constants ----------

# -------------------------------


class DistractorAiAircraft(AbstractAiAcft):
	"""
	This class represents an AI aircraft NOT in radio contact (uncontrolled), that just flies around.
	Used as disctractors in solo sessions.
	"""
	def __init__(self, callsign, acft_type, init_params, ticks_to_live):
		AbstractAiAcft.__init__(self, callsign, acft_type, init_params, Status(airborne=True))
		self.ticks_to_live = ticks_to_live
	
	def doTick(self):
		self.params.position = self.params.position.moved(self.params.heading, distance_travelled(self.tick_interval, self.params.ias))
		self.ticks_to_live -= 1

	def outlived(self):
		return self.ticks_to_live <= 0
