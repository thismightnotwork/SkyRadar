
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

import struct
from math import sqrt
from sys import stderr

from ext.audio import InOutAudioStreamer

from gui.misc import signals

from session.config import settings
from session.manager import SessionType


# ---------- Constants ----------

# -------------------------------

class PhoneLineStatus:
	IDLE, CALLING, RINGING, HELD_INCOMING, HELD_OUTGOING, IN_CALL = range(6)



def RMS_amplitude(block):
	count = len(block) / 2
	sum_squares = 0
	for sample in struct.unpack('%dh' % count, block): # iterate over 16-bit (short int) samples
		n = sample / 32768 # normalise short to 1
		sum_squares += n * n
	return sqrt(sum_squares / count)



class AbstractPhoneLineManager:
	"""
	Subclasses should redefine the following methods:
		- _startVoiceWith(str: ATC callsign)
		- _stopVoice()
		- _sendRequest(str: ATC callsign)
		- _sendDrop(str: ATC callsign)
	"""
	def __init__(self):
		self.line_status = {}

	def createPhoneLine(self, atc):
		if atc in self.line_status:
			print('WARNING: Phone line "%s" already exists.' % atc, file=stderr)
		self.line_status[atc] = PhoneLineStatus.IDLE
		signals.phoneLineStatusChanged.emit(atc)
		settings.session_recorder.proposeAtcPhoneStatusChange(settings.session_manager.clockTime(), atc, PhoneLineStatus.IDLE)

	def destroyPhoneLine(self, atc):
		if self.isOpenOutgoing(atc):
			self.dropPhoneLine(atc)
		try:
			del self.line_status[atc]
		except KeyError:
			print('WARNING: Phone line "%s" does not exist.' % atc, file=stderr)
		signals.phoneLineStatusChanged.emit(atc)
		settings.session_recorder.proposeAtcPhoneStatusChange(settings.session_manager.clockTime(), atc, PhoneLineStatus.IDLE)

	def setLineStatus(self, atc, status):
		self.line_status[atc] = status
		signals.phoneLineStatusChanged.emit(atc)
		settings.session_recorder.proposeAtcPhoneStatusChange(settings.session_manager.clockTime(), atc, status)

	def lineStatus(self, atc):
		return self.line_status.get(atc)

	def linesWithStatus(self, status):
		return [atc for atc, lls in self.line_status.items() if lls == status]

	def isOpenOutgoing(self, atc):
		try:
			return self.line_status[atc] in [PhoneLineStatus.CALLING, PhoneLineStatus.IN_CALL, PhoneLineStatus.HELD_OUTGOING]
		except KeyError:
			return False

	def isOpenIncoming(self, atc):
		try:
			return self.line_status[atc] in [PhoneLineStatus.RINGING, PhoneLineStatus.IN_CALL, PhoneLineStatus.HELD_INCOMING]
		except KeyError:
			return False

	def requestPhoneLine(self, atc):
		if atc not in self.line_status:
			print('WARNING: Phone line "%s" does not exist.' % atc, file=stderr)
		elif self.isOpenOutgoing(atc):
			print('WARNING: Phone line "%s" already requested.' % atc, file=stderr)
		else:
			if self.isOpenIncoming(atc):  # answering an incoming request
				if settings.session_manager.session_type != SessionType.TEACHER:
					for ll in self.linesWithStatus(PhoneLineStatus.IN_CALL): # currently active call (normally only one or zero)
						self.dropPhoneLine(ll)  # put on hold; status changes to HELD_INCOMING accordingly
				self._sendRequest(atc)
				self._startVoiceWith(atc)
				self.setLineStatus(atc, PhoneLineStatus.IN_CALL)
			else:  # placing a call
				self._sendRequest(atc)
				self.setLineStatus(atc, PhoneLineStatus.CALLING)
	
	def dropPhoneLine(self, atc):
		if atc not in self.line_status:
			print('WARNING: Phone line "%s" does not exist.' % atc, file=stderr)
		elif not self.isOpenOutgoing(atc):
			print('WARNING: Phone line "%s" not requested.' % atc, file=stderr)
		else:
			if self.isOpenIncoming(atc):  # hanging up (or putting on hold) the current call
				self._stopVoice()
			self._sendDrop(atc)
			if self.isOpenIncoming(atc):  # hanging up the current call
				self.setLineStatus(atc, PhoneLineStatus.HELD_INCOMING)
			else:  # cancelling my request
				self.setLineStatus(atc, PhoneLineStatus.IDLE)

	def incomingLineRequest(self, atc):
		if atc not in self.line_status:
			print('WARNING: Phone line "%s" does not exist.' % atc, file=stderr)
		elif self.isOpenIncoming(atc):
			print('WARNING: Phone line "%s" already incoming.' % atc, file=stderr)
		else:
			if self.isOpenOutgoing(atc):  # they answer my request
				if settings.session_manager.session_type != SessionType.TEACHER \
						and len(self.linesWithStatus(PhoneLineStatus.IN_CALL)) > 0:  # currently in (another) call; turn my prior request into theirs
					self.dropPhoneLine(atc)  # makes phone line HELD_INCOMING
					self.setLineStatus(atc, PhoneLineStatus.RINGING)
				else:  # they are picking up and we are ready to talk
					self._startVoiceWith(atc)
					self.setLineStatus(atc, PhoneLineStatus.IN_CALL)
					signals.phoneCallAnswered.emit(atc)
			else:  # they are calling us
				self.setLineStatus(atc, PhoneLineStatus.RINGING)
				signals.incomingPhoneCall.emit(atc)
	
	def incomingLineDrop(self, atc):
		if atc not in self.line_status:
			print('WARNING: Phone line "%s" does not exist.' % atc, file=stderr)
		elif not self.isOpenIncoming(atc):
			print('WARNING: Phone line "%s" had no incoming request.' % atc, file=stderr)
		else:
			if self.isOpenOutgoing(atc):  # they are hanging up
				self._stopVoice()
				self.setLineStatus(atc, PhoneLineStatus.HELD_OUTGOING)
				signals.phoneCallDropped.emit(atc)
			else:  # they are cancelling their line request
				self.setLineStatus(atc, PhoneLineStatus.IDLE)

	# Methods to implement in subclasses
	def _startVoiceWith(self, atc):
		raise NotImplementedError()
	
	def _stopVoice(self):
		raise NotImplementedError()
	
	def _sendRequest(self, atc):
		raise NotImplementedError()
	
	def _sendDrop(self, atc):
		raise NotImplementedError()



class AbstractVoipPhoneManager(AbstractPhoneLineManager, InOutAudioStreamer): # class is therefore also a QThread
	"""
	Subclasses should redefine the following methods:
		- sendSoundData(tuple: callee INET address, bytes: sound output picked up from mic)
		- _sendRequest(str: ATC callsign)
		- _sendDrop(str: ATC callsign)
	"""
	def __init__(self, gui):
		AbstractPhoneLineManager.__init__(self)
		InOutAudioStreamer.__init__(self, gui)
		self.phone_book = {} # str callsign -> INET address
		self.current_call_inet = None
		self.current_audio_index_in = 0  # to help us skip out-of-sequence received audio chunks
		self.current_audio_index_out = 0 # to help receiver skip out-of-sequence audio chunks

	def updatePhoneBook(self, atc, inet_addr):
		self.phone_book[atc] = inet_addr
		self.createPhoneLine(atc)

	def removePhoneBookEntry(self, atc):
		self.destroyPhoneLine(atc)
		try:
			del self.phone_book[atc]
		except KeyError:
			pass
	
	def receivePhoneData(self, phone_data):
		# TODO this assumed audio is from current call (maximum one phone line assumed open at a time); add sender check?
		seq = struct.unpack('!I', phone_data[:4])[0]
		if seq > self.current_audio_index_in:
			self.current_audio_index_in = seq
			self.receiveAudioData(phone_data[4:])
		else: # output chunk is out of sequence
			print('Ignored out-of-sequence audio packet from current phone call.', file=stderr)

	## Defining AbstractPhoneLineManager methods
	def _startVoiceWith(self, atc):
		try:
			self.current_call_inet = self.phone_book[atc]
			self.current_audio_index_in = 0
			self.current_audio_index_out = 0
			self.startProcessingMicAudio()
		except KeyError:
			print('Unknown phone number to start voice with %s' % atc, file=stderr)

	def _stopVoice(self):
		self.stopProcessingMicAudio()

	## Defining InOutAudioStreamer method
	def processMicAudioChunk(self, audio_data):
		self.current_audio_index_out += 1
		self.sendPhoneData(struct.pack('!I', self.current_audio_index_out) + audio_data, self.current_call_inet)

	## Methods to implement in subclasses below (in addition to inherited still abstract methods)
	def sendPhoneData(self, data, inet_addr):
		raise NotImplementedError()
