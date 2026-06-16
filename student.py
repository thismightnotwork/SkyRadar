
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

from PyQt5.QtNetwork import QTcpSocket, QAbstractSocket
from PyQt5.QtWidgets import QMessageBox

from base.cpdlc import CpdlcMessage
from base.fpl import FPL
from base.radio import CommFrequency
from base.strip import Strip, received_from_detail, handover_details
from base.text import TextMessage
from base.timeline import unknown_radio_msg_sender_str
from base.utc import VirtualClock, duration_str
from base.util import pop_all
from base.weather import Weather

from ext.fgfs import send_packet_to_views
from ext.fgms import update_FgmsAircraft_list

from gui.actions import register_weather_information, kill_aircraft
from gui.misc import signals

from session.config import settings, version_string
from session.env import env
from session.manager import SessionManager, SessionType, student_callsign, TextMsgBlocked, CpdlcOperationBlocked
from session.managers.teacher import TeachingMsg, TeachingSessionWire, pyaudio_available, TeachingPhoneRadioManager, \
		CPDLC_cmd_prefix_conn, CPDLC_cmd_prefix_xfr, CPDLC_cmd_prefix_msg


# ---------- Constants ----------

init_connection_timeout = 3000 # ms

# -------------------------------


class StudentSessionManager(SessionManager):
	def __init__(self, gui):
		SessionManager.__init__(self, gui, SessionType.STUDENT)
		self.phone_radio_manager = None
		self.teacher_socket = QTcpSocket() # this socket connects to the teacher
		self.traffic = [] # FgmsAircraft list
		self.clock = VirtualClock() # we rely on our real time when only real time flows; teacher sends SET_CLOCK messages otherwise
	
	def start(self):
		self.teacher_socket.connectToHost(settings.teaching_service_host, settings.teaching_service_port)
		if self.teacher_socket.waitForConnected(init_connection_timeout):
			self.teacher = TeachingSessionWire(self.teacher_socket)
			self.teacher_socket.disconnected.connect(self.socketDisconnected)
			self.teacher.messageArrived.connect(self.receiveMsgFromTeacher)
			print('Connected to teacher.')
			self.traffic.clear()
			signals.sessionStarted.emit(SessionType.STUDENT)
			self.teacher.sendMessage(TeachingMsg(TeachingMsg.ATCPIE_VERSION, data=version_string))
		else:
			QMessageBox.critical(self.gui, 'Connection error', 'Connection to teacher has failed.')
	
	def stop(self): # CAUTION called by self.socketDisconnected (not only by GUI)
		if self.phone_radio_manager:
			self.phone_radio_manager.deactivate()
		self.teacher.messageArrived.disconnect(self.receiveMsgFromTeacher)
		self.teacher_socket.disconnected.disconnect(self.socketDisconnected)
		self.teacher_socket.disconnectFromHost()
		signals.sessionEnded.emit(SessionType.STUDENT)
	
	def isRunning(self):
		return self.teacher_socket.state() == QAbstractSocket.ConnectedState

	def clockTime(self):
		return self.clock.readTime()
	
	def getAircraft(self):
		return self.traffic[:]
	
	
	## ACFT/ATC INTERACTION
	
	def instructAircraftByCallsign(self, callsign, instr):
		signals.textInstructionSuggestion.emit(callsign, instr.readOutStr(env.radarContactByCallsign(callsign)))
	
	def postTextRadioMsg(self, msg):
		assert not msg.isPrivate()
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.TEXT_MSG, data=msg.txtMsg()))
	
	def postAtcChatMsg(self, msg):
		if not msg.isPrivate():
			raise TextMsgBlocked('Public ATC messaging disabled in tutoring sessions.')
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.TEXT_MSG, data=(msg.recipient() + '\n' + msg.txtOnly())))
	
	def sendStrip(self, strip, sendto):
		msg_data = sendto + '\n' + strip.encodeDetails(handover_details)
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.STRIP_EXCHANGE, data=msg_data))
	
	def sendCpdlcMsg(self, callsign, msg):
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.CPDLC, data=('%s\n%s%s' % (callsign, CPDLC_cmd_prefix_msg, msg.toEncodedStr()))))
	
	def sendCpdlcTransferRequest(self, acft_callsign, atc_callsign, proposing):
		if not proposing:
			raise CpdlcOperationBlocked('Cannot abort a transfer as student.')
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.CPDLC,
				data=('%s\n%s%d %s' % (acft_callsign, CPDLC_cmd_prefix_xfr, proposing, atc_callsign))))
	
	def sendCpdlcTransferResponse(self, acft_callsign, atc_callsign, accept):
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.CPDLC,
				data=('%s\n%s%d' % (acft_callsign, CPDLC_cmd_prefix_xfr, accept))))
	
	def sendCpdlcDisconnect(self, callsign):
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.CPDLC, data=('%s\n%s0' % (callsign, CPDLC_cmd_prefix_conn))))
	
	
	## VOICE COMM'S
	
	def createRadio(self):
		QMessageBox.critical(self.gui, 'Create radio',
				'No radio boxes in student sessions. Radio transmissions all happen on a single virtual frequency.')

	def recordAtis(self, parent_dialog):
		pass

	def phoneLineManager(self):
		return self.phone_radio_manager # can be None
	
	
	## ONLINE SYSTEMS
	
	def weatherLookUpRequest(self, station):
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.WEATHER, data=station))
	
	def pushFplOnline(self, fpl):
		if fpl.isOnline():
			fpl.modified_details.clear()
		else:
			used_IDs = {got.online_id for got in env.FPLs.findAll(pred=FPL.isOnline)}
			i = 0
			while '%s-%X' % (student_callsign, i) in used_IDs:
				i += 1
			fpl.markAsOnline('%s-%X' % (student_callsign, i))
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.FLIGHT_PLAN, data=fpl.encode()))
		env.FPLs.refreshViews()
	
	def changeFplStatus(self, fpl, new_status):
		fpl.setOnlineStatus(new_status)
		self.teacher.sendMessage(TeachingMsg(TeachingMsg.FLIGHT_PLAN, data=fpl.encode()))
		env.FPLs.refreshViews()
	
	def syncOnlineFPLs(self):
		pass # online FPLs always in sync
	
	
	## MANAGER-SPECIFIC

	def killAircraft(self, acft):
		for acft in pop_all(self.traffic, lambda a: a is acft):
			acft.resetPtt()
	
	def socketDisconnected(self):
		self.stop()
		QMessageBox.critical(self.gui, 'Disconnected', 'Teacher connection dropped.')
	
	def receiveMsgFromTeacher(self, msg):
		#DEBUGif msg.type != TeachingMsg.TRAFFIC and msg.type != TeachingMsg.PHONE_AUDIO:
		#DEBUG	print('=== STUDENT RECEIVES ===\n%s\n=== End ===' % msg.output)
		if msg.type == TeachingMsg.ACFT_KILLED:
			callsign = msg.strData()
			try:
				kill_aircraft(next(acft for acft in self.traffic if acft.identifier == callsign))
			except StopIteration:
				print('Unknown ACFT %s ("kill" message from teacher).' % callsign, file=stderr)
		
		elif msg.type == TeachingMsg.TRAFFIC: # traffic update; contains FGMS packet
			fgms_packet = msg.binData()
			update_FgmsAircraft_list(self.traffic, fgms_packet)
			send_packet_to_views(fgms_packet)
			self.teacher.sendMessage(TeachingMsg(TeachingMsg.TRAFFIC))
		
		elif msg.type == TeachingMsg.SIM_PAUSED:
			self.clock.pause()
			signals.sessionPaused.emit()
			signals.statusBarMsg.emit('Simulation paused')
		
		elif msg.type == TeachingMsg.SIM_RESUMED:
			self.clock.resume()
			signals.sessionResumed.emit()
			signals.statusBarMsg.emit('Simulation resumed')

		elif msg.type == TeachingMsg.SET_CLOCK:
			old_time = self.clock.readTime()
			self.clock.setTimeEncoded(msg.strData())
			time_skip = self.clock.readTime() - old_time
			if time_skip.total_seconds() >= 0:
				signals.statusBarMsg.emit('Simulation time skipped %s forward' % duration_str(time_skip))
			else:
				if self.traffic:
					print('WARNING: Setting time backward with existing traffic.', file=stderr)
				signals.statusBarMsg.emit('Simulation set %s backward' % duration_str(-time_skip))
		
		elif msg.type == TeachingMsg.TEXT_MSG:
			lines = msg.strData().split('\n')
			if len(lines) == 1: # text radio msg
				if '\t' in lines[0]:
					sender, txt = lines[0].split('\t', maxsplit=1)
				else:
					sender = unknown_radio_msg_sender_str
					txt = lines[0]
				signals.incomingTextRadioMsg.emit(TextMessage(sender, txt))
			elif len(lines) == 2: # ATC private msg
				signals.incomingAtcTextMsg.emit(TextMessage(lines[0], lines[1], recipient=student_callsign, private=True))
			else:
				print('ERROR: Invalid format in received ATC text message from teacher.', file=stderr)
		
		elif msg.type == TeachingMsg.STRIP_EXCHANGE:
			line_sep = msg.strData().split('\n', maxsplit=1)
			fromATC = line_sep[0]
			strip = Strip.fromEncodedDetails('' if len(line_sep) < 2 else line_sep[1])
			strip.writeDetail(received_from_detail, fromATC)
			signals.receiveStrip.emit(strip)
		
		elif msg.type == TeachingMsg.FLIGHT_PLAN:
			fpl = FPL.fromEncoded(msg.strData())
			if fpl.isOnline():
				env.FPLs.updateFromOnlineDownload(fpl)
			else:
				print('ERROR: Received an offline FPL from teacher.', file=stderr)
		
		elif msg.type == TeachingMsg.ATC_LIST:
			to_remove = set(env.ATCs.knownAtcCallsigns())
			for line in msg.strData().split('\n'):
				if line != '': # last line is empty
					lst = line.rsplit('\t', maxsplit=1)
					try:
						frq = CommFrequency(lst[1]) if len(lst) == 2 else None
					except ValueError:
						frq = None
					env.ATCs.updateATC(lst[0], None, None, frq)
					to_remove.discard(lst[0])
			for atc in to_remove:
				env.ATCs.removeATC(atc)
				if self.phone_radio_manager is not None:
					self.phone_radio_manager.destroyPhoneLine(atc)
		
		elif msg.type == TeachingMsg.WEATHER:
			register_weather_information(Weather(msg.strData()))
		
		elif msg.type == TeachingMsg.CPDLC:
			# Teacher msg format in 2 lines, first being ACFT callsign, second is either of the following:
			#  - ACFT disconnects or logs on: CPDLC_cmd_prefix_conn + "0"/"1"
			#  - teacher proposes or cancels transfer: CPDLC_cmd_prefix_xfr + "0"/"1" + space + ATC callsign
			#  - teacher accepts or rejects our transfer proposal: CPDLC_cmd_prefix_xfr + "0"/"1"
			#  - other CPDLC message: CPDLC_cmd_prefix_msg + encoded message string
			try:
				acft_callsign, line2 = msg.strData().split('\n', maxsplit=1)
				link = env.cpdlc.lastDataLink(acft_callsign)
				if line2 == CPDLC_cmd_prefix_conn + '0': # ACFT disconnects output link
					if link is not None:
						link.terminate(False)
				elif line2 == CPDLC_cmd_prefix_conn + '1': # ACFT log-on
					if settings.controller_pilot_data_link:
						env.cpdlc.beginDataLink(acft_callsign)
					self.teacher.sendMessage(TeachingMsg(TeachingMsg.CPDLC,
							data=('%s\n%s%d' % (acft_callsign, CPDLC_cmd_prefix_conn, settings.controller_pilot_data_link))))
				elif line2.startswith(CPDLC_cmd_prefix_xfr):
					positive = line2[len(CPDLC_cmd_prefix_xfr)] == '1' # IndexError is guarded here
					if ' ' in line2: # teacher proposing or cancelling XFR
						signals.cpdlcTransferRequest.emit(acft_callsign, line2.split(' ', maxsplit=1)[1], positive)
					else: # teacher accepting or rejecting XFR
						if link is None or link.pendingTransferTo() is None:
							print('Ignored CPDLC transfer confirmed while none pending for %s.' % acft_callsign, file=stderr)
						else:
							signals.cpdlcTransferResponse.emit(acft_callsign, link.pendingTransferTo(), positive)
				elif line2.startswith(CPDLC_cmd_prefix_msg): # ACFT sending a message
					if link is not None:
						link.appendMessage(CpdlcMessage.fromEncodedStr(line2[len(CPDLC_cmd_prefix_msg):]))
				else:
					print('Error decoding CPDLC command from teacher:', line2, file=stderr)
			except (IndexError, ValueError):
				print('Error decoding CPDLC message from teacher', file=stderr)

		elif msg.type == TeachingMsg.RADIO_PTT: # msg format: "b acft" where b is '1' or '0' for PTT on/off; acft is caller's identifier
			line_sep = msg.strData().split(' ', maxsplit=1)
			try:
				ptt = bool(int(line_sep[0]))
				cs = line_sep[1]
				caller = next(acft for acft in self.getAircraft() if acft.identifier == cs)
				if ptt:
					caller.setPtt()
				else:
					caller.resetPtt()
			except StopIteration:
				print('Ignored PTT message from teacher (unknown ACFT %s).' % line_sep[1], file=stderr)
			except (ValueError, IndexError):
				print('Error decoding PTT message value from teacher.', file=stderr)

		elif msg.type == TeachingMsg.RADIO_AUDIO:
			if self.phone_radio_manager is not None and not settings.radios_silenced and not self.phone_radio_manager.radioPTT():
				self.phone_radio_manager.receiveAudioData(msg.binData())

		elif msg.type == TeachingMsg.PHONE_REQUEST:
			if self.phone_radio_manager is None:
				if msg.strData() == '' and pyaudio_available:
					self.phone_radio_manager = TeachingPhoneRadioManager(self.gui, self.teacher.sendMessage)
					self.phone_radio_manager.activate()
					self.teacher.sendMessage(TeachingMsg(TeachingMsg.PHONE_REQUEST))
					signals.phoneManagerAvailabilityChange.emit()
					QMessageBox.information(self.gui, 'Phone/radio audio active', 'Integrated phone and radio audio '
							'activated by the teacher for the session. Use keyboard PTT key to transmit on the radio.')
			else: # got a running phone/radio manager
				tokens = msg.strData().split(maxsplit=1)
				if len(tokens) == 2 and tokens[0] == '0':
					self.phone_radio_manager.incomingLineDrop(tokens[1])
				elif len(tokens) == 2 and tokens[0] == '1':
					self.phone_radio_manager.incomingLineRequest(tokens[1])
				else:
					print('Error decoding phone request/drop message from teacher', file=stderr)

		elif msg.type == TeachingMsg.PHONE_AUDIO:
			if self.phone_radio_manager is not None:
				self.phone_radio_manager.receiveAudioData(msg.binData())

		elif msg.type == TeachingMsg.ATCPIE_VERSION:
			teacher_version = msg.strData()
			if teacher_version != version_string:
				QMessageBox.warning(self.gui, 'ATC-Pie version mismatch', 'WARNING: Teacher is using ATC-Pie version %s, different from this one.' % teacher_version)

		else:
			print('Unhandled message type from teacher: %s' % msg.type, file=stderr)
