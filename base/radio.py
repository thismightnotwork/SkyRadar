
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

from base.util import some, rounded

from session.config import settings
from session.env import env
from session.manager import SessionType


# ---------- Constants ----------

EMG_frequency_MHz = 121.5 # use EMG_comm_freq defined further down for CommFrequency instance
antenna_height = 50 # ft

# -------------------------------


class CommFrequency:
	spacing_MHz = .025 / 3 # 8.33 kHz
	
	def __init__(self, arg):
		"""
		STRING argument containing digits and optional decimal point:
			Creates an 8.33-spaced frequency. Channel names are recognised, including old and shortened names.
			Tunes to the closest otherwise. Decimal point can be left out even if decimal part is non-zero, as
			when digits are read out in order.
			Freq's created this way print (__str__) with 6 significant digits to be read like a properly tuned aviation freq/channel.
		NUMERICAL argument:
			Creates an exact frequency from given physical wave frequency value in MHz.
			Freq's created this way print (__str__) with 7 significant digits to ensure distinction with channel names.
		"""
		self.from_str = isinstance(arg, str)
		self.keep_833_channel_name = None
		if self.from_str:
			mhzstr = arg if '.' in arg else arg[:3] + '.' + arg[3:] # ensures a decimal point
			int_part, dec_part = mhzstr.rsplit('.', maxsplit=1)
			if len(dec_part) == 2:
				if dec_part[-1] in '27': # ._2 and ._7 endings are old shortened names for 25kHz-step freq's
					mhzstr += '5'
				elif dec_part[-1] not in '05':
					raise ValueError('invalid frequency')
			elif len(dec_part) == 3:
				last_two = dec_part[1:]
				if last_two in ['05', '10', '15', '30', '35', '40', '55', '60', '65', '80', '85', '90']: # recognise 8.33 channel name
					self.keep_833_channel_name = mhzstr
					mhzstr = mhzstr[:-2] + {'05':'00', '30':'25', '55':'50', '80':'75'}.get(last_two, last_two) # replace last 2 digits
				elif last_two not in ['00', '25', '50', '75']:
					raise ValueError('invalid channel or frequency')
			elif len(dec_part) >= 4:
				raise ValueError('too many decimal digits')
			self.mhz = rounded(float(mhzstr), CommFrequency.spacing_MHz)
		else: # numerical argument, physical wave frequency value
			self.mhz = arg
		if abs(self.mhz) < 1e-6: # freq not even 1 Hz
			raise ValueError('invalid near-zero comm frequency')
	
	def __str__(self):
		if self.from_str:
			return some(self.keep_833_channel_name, '%.3f' % self.mhz)
		else:
			return '%.4f' % self.mhz
	
	def MHz(self):
		return self.mhz
	
	def inTune(self, other):
		return abs(self.mhz - other.mhz) <= CommFrequency.spacing_MHz / 2


EMG_comm_freq = CommFrequency(EMG_frequency_MHz)




class AbstractRadio:
	"""
	Subclasses should redefine the following silent methods:
		- state "getters": isOn, frequency, isTransmitting, volume
		- state "setters": switchOnOff, setFrequency, setPTT, setVolume
	"""
	def __init__(self):
		self.RDF_monitored = False
		self.RDF_signal = None
	
	def isRdfMonitored(self):
		return self.RDF_monitored
	
	def setRdfMonitored(self, toggle):
		self.RDF_monitored = toggle
		if not toggle:
			self.RDF_signal = None

	def rdfSignal(self):
		return self.RDF_signal

	def setRdfSignal(self, signal_data):
		self.RDF_signal = signal_data
	
	# Methods to implement follow
	def isOn(self):
		raise NotImplementedError()
	
	def frequency(self):
		raise NotImplementedError()
	
	def isTransmitting(self):
		raise NotImplementedError()
	
	def volume(self):
		raise NotImplementedError()
	
	def switchOnOff(self, toggle):
		raise NotImplementedError()
	
	def setFrequency(self, new_frq):
		raise NotImplementedError()
	
	def setPTT(self, ptt):
		raise NotImplementedError()
	
	def setVolume(self, volume):
		raise NotImplementedError()






class RdfSignalData:
	def __init__(self, frequency, hdg, quality):
		self.frequency = frequency # CommFrequency, or None in the case of single dummy freq. (solo & tutoring sessions)
		self.direction = hdg # base.params.Heading
		self.quality = quality # float in [0, 1]



class RadioDirectionFinder:
	def __init__(self):
		self.supp_signal = None  # RdfSignalData or None, for publicised frequency (if any), and single dummy freq. in solo & tutoring sessions
		# NB: frequency-specific signals are stored in RDF-enabled radios (see AbstractRadio)
		self.latest_signal = None  # RdfSignalData or None
	
	def antennaPos(self):
		if env.airport_data is None:
			coords = env.radarPos()
			base_alt = env.elevation(coords)
		else: # radio antenna at top of tower
			coords, base_alt = env.viewpoint()
		return coords, base_alt + antenna_height

	def strongestSignal(self):
		lst = [radio.rdfSignal() for radio in settings.radios if radio.isRdfMonitored() and radio.rdfSignal() is not None]
		if self.supp_signal is not None:
			lst.append(self.supp_signal)
		return max(lst, key=lambda sig: sig.quality) if lst else None
	
	def latestSignal(self):
		return self.latest_signal
	
	def radioSignal(self, freq, direction, quality=1):
		if settings.radio_direction_finding:
			sig = RdfSignalData(freq, direction, quality)
			if freq is None or settings.publicised_frequency is not None and settings.publicised_frequency.inTune(freq) \
					or settings.session_manager.session_type == SessionType.PLAYBACK:
				self.supp_signal = sig
				settings.session_recorder.proposeRdfSignalUpdate(settings.session_manager.clockTime(), self.supp_signal)
				self.latest_signal = self.supp_signal
			else:
				for radio in settings.radios:
					if radio.isRdfMonitored() and radio.frequency().inTune(freq):
						radio.setRdfSignal(sig)
						settings.session_recorder.proposeRdfSignalUpdate(settings.session_manager.clockTime(), sig)
						self.latest_signal = sig

	def endOfSignal(self, freq):
		if freq is None:
			self.supp_signal = None
			settings.session_recorder.proposeRdfSignalEnd(settings.session_manager.clockTime(), None)
		else:
			for radio in settings.radios:
				if radio.frequency().inTune(freq):
					radio.setRdfSignal(None)
					settings.session_recorder.proposeRdfSignalEnd(settings.session_manager.clockTime(), freq)
	
	def resetSignals(self):
		for radio in settings.radios:
			radio.setRdfSignal(None)
		self.supp_signal = None
		self.latest_signal = None
