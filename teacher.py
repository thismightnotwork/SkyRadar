
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

from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtNetwork import QTcpServer
from PyQt5.QtWidgets import QMessageBox

from ai.controlledAircraft import ControlledAiAircraft

from base.cpdlc import CpdlcMessage
from base.db import wake_turb_cat
from base.fpl import FPL
from base.phone import RMS_amplitude, AbstractPhoneLineManager
from base.strip import Strip, handover_details, received_from_detail, sent_to_detail, student_ok_detail
from base.text import TextMessage
from base.utc import VirtualClock
from base.util import pop_all

from ext.audio import pyaudio_available, InOutAudioStreamer
from ext.fgfs import send_packet_to_views

from gui.actions import register_weather_information
from gui.misc import selection, signals
from gui.dialogs.createTraffic import CreateTrafficDialog
from gui.widgets.basicWidgets import Ticker

from session.config import settings
from session.env import env
from session.manager import SessionManager, SessionType, student_callsign, teacher_callsign, TextMsgBlocked, HandoverBlocked, CpdlcOperationBlocked


# ---------- Constants ----------

teacher_ticker_interval = 200 # ms
max_noACK_traffic = 20

CPDLC_cmd_prefix_conn = 'CX:' # connection/disconnection
CPDLC_cmd_prefix_xfr = 'XFR:' # transfers (proposed, cancelled, accepted, rejected)
CPDLC_cmd_prefix_msg = 'MSG:' # regular dialogue message

# -------------------------------



class TeachingMsg:
	msg_types = SIM_PAUSED, SIM_RESUMED, SET_CLOCK, ACFT_KILLED, TEXT_MSG, \
			STRIP_EXCHANGE, FLIGHT_PLAN, ATC_LIST, WEATHER, TRAFFIC, CPDLC, \
			RADIO_PTT, RADIO_AUDIO, PHONE_REQUEST, PHONE_AUDIO, ATCPIE_VERSION = range(16)
	
	def __init__(self, msg_type, data=None):
		self.type = msg_type
		self.data = b''
		if data is not None:
			self.appendData(data)
	
	def appendData(self, data):
		self.data += data if isinstance(data, bytes) else data.encode('utf8')
	
	def binData(self):
		return self.data
	
	def strData(self):
		return self.data.decode('utf8')






class TeachingSessionWire(QObject):
	messageArrived = pyqtSignal(TeachingMsg)
	
	def __init__(self, socket):
		QObject.__init__(self)
		self.socket = socket
		self.got_msg_type = None
		self.got_data_len = None
		self.socket.readyRead.connect(self.readAvailableBytes)

	def readAvailableBytes(self):
		if self.got_msg_type is None:
			if self.socket.bytesAvailable() < 1:
				return
			self.got_msg_type = int.from_bytes(self.socket.read(1), 'big')
		if self.got_data_len is None:
			if self.socket.bytesAvailable() < 4:
				return
			self.got_data_len = int.from_bytes(self.socket.read(4), 'big')
		if self.socket.bytesAvailable() < self.got_data_len:
			return
		self.messageArrived.emit(TeachingMsg(self.got_msg_type, data=self.socket.read(self.got_data_len)))
		self.got_msg_type = self.got_data_len = None
		if self.socket.bytesAvailable() > 0:
			self.socket.readyRead.emit()
	
	def sendMessage(self, msg):
		#DEBUGif msg.type != TeachingMsg.TRAFFIC and msg.type != TeachingMsg.PHONE_AUDIO:
		#DEBUG	print('Sending: %s' % msg.output)
		buf = msg.type.to_bytes(1, 'big') # message type code
		buf += len(msg.data).to_bytes(4, 'big') # length of output
		buf += msg.data # message output
		self.socket.write(buf)




class TeachingPhoneRadioManager(AbstractPhoneLineManager, InOutAudioStreamer): # class is therefore also a QThread
	msgToSend = pyqtSignal(TeachingMsg) # keep as signal (socket send operation from different thread is blocking)

	def __init__(self, gui, send_msg_function):
		AbstractPhoneLineManager.__init__(self)
		InOutAudioStreamer.__init__(self, gui)
		self.send_mg_function = send_msg_function
		self.running = False
		self.call_in_progress = False
		self.radio_PTT = False

	def activate(self): # use when a student connects
		self.start()
		for atc in env.ATCs.knownAtcCallsigns():
			self.createPhoneLine(atc)
		self.msgToSend.connect(self.send_mg_function)
		signals.newATC.connect(self.createPhoneLine)
		signals.kbdPTT.connect(self.setRadioPTT)

	def deactivate(self):
		self.msgToSend.disconnect(self.send_mg_function)
		signals.newATC.disconnect(self.createPhoneLine)
		signals.kbdPTT.disconnect(self.setRadioPTT)
		self.radio_PTT = False
		for atc in env.ATCs.knownAtcCallsigns():
			if self.lineStatus(atc) is not None:
				self.destroyPhoneLine(atc)
		self.stopAndWait(allowRestart=True)

	def radioPTT(self):
		return self.radio_PTT

	def setRadioPTT(self, toggle):
		self.radio_PTT = toggle
		if toggle:
			if not self.processingMicAudio():
				self.startProcessingMicAudio()
		else: # releasing PTT
			if not self.call_in_progress:
				self.stopProcessingMicAudio()

	## Defining AbstractPhoneLineManager methods below
	def _startVoiceWith(self, atc):
		self.call_in_progress = True
		if not self.processingMicAudio():
			self.startProcessingMicAudio()

	def _stopVoice(self):
		self.call_in_progress = None
		if not self.radioPTT():
			self.stopProcessingMicAudio()

	def _sendRequest(self, atc):
		self.msgToSend.emit(TeachingMsg(TeachingMsg.PHONE_REQUEST, data=('1 %s' % atc)))

	def _sendDrop(self, atc):
		self.msgToSend.emit(TeachingMsg(TeachingMsg.PHONE_REQUEST, data=('0 %s' % atc)))

	## Defining InOutAudioStreamer method
	def processMicAudioChunk(self, audio_data):
		if self.radioPTT():
			self.msgToSend.emit(TeachingMsg(TeachingMsg.RADIO_AUDIO, data=audio_data))
		elif RMS_amplitude(audio_data) > settings.phone_line_squelch:
			self.msgToSend.emit(TeachingMsg(TeachingMsg.PHONE_AUDIO, data=audio_data))




# -------------------------------

class TeacherSessionManager(SessionManager):
	def __init__(self, gui):
		SessionManager.__init__(self, gui, SessionType.TEACHER)
		self.session_ticker = Ticker(gui, self.tickSessionOnce)
		self.clock = VirtualClock()
		self.acft_transmitting = None
		self.phone_radio_manager = None
		self.server = QTcpServer(gui)
		self.student_socket = None
		self.server.newConnection.connect(self.studentConnects)
		self.aircraft_list = [] # ControlledAiAircraft list
		self.noACK_traffic_count = 0
	
	def start(self):
		self.aircraft_list.clear()
		self.acft_transmitting = None
		self.session_ticker.startTicking(teacher_ticker_interval)
		self.server.listen(port=settings.teaching_service_port)
		if pyaudio_available and settings.phone_lines_enabled:
			self.phone_radio_manager = TeachingPhoneRadioManager(self.gui, self.phoneRadioMsgToSend)
		print('Teaching server ready on port %d' % settings.teaching_service_port)
		signals.specialTool.connect(self.createNewTraffic)
		signals.newATC.connect(self.sendATCs)
		signals.kbdPTT.connect(self.sendPTT)
		signals.sessionStarted.emit(SessionType.TEACHER)
	
	def stop(self):
		if self.isRunning():
			self.session_ticker.stop()
			if self.studentConnected():
				self.shutdownStudentConnection()
			signals.kbdPTT.disconnect(self.sendPTT)
			signals.newATC.disconnect(self.sendATCs)
			signals.specialTool.disconnect(self.createNewTraffic)
			self.server.close()
			self.aircraft_list.clear()
			self.acft_transmitting = None
			signals.sessionEnded.emit(SessionType.TEACHER)

	def pause(self):
		if self.isRunning() and not self.clock.isPaused():
			self.session_ticker.stop()
			self.clock.pause()
			if self.studentConnected():
				self.student.sendMessage(TeachingMsg(TeachingMsg.SIM_PAUSED))
			signals.sessionPaused.emit()
	
	def resume(self):
		if self.clock.isPaused():
			self.clock.resume()
			self.session_ticker.startTicking(teacher_ticker_interval)
			if self.studentConnected():
				self.student.sendMessage(TeachingMsg(TeachingMsg.SIM_RESUMED))
			signals.sessionResumed.emit()

	def isRunning(self):
		return self.session_ticker.isActive() or self.clock.isPaused()

	def clockTime(self):
		return self.clock.readTime()
	
	def getAircraft(self):
		return self.aircraft_list[:]
	
	
	## ACFT/ATC INTERACTION
	
	def instructAircraftByCallsign(self, callsign, instr):
		print('INTERNAL ERROR: TeacherSessionManager.instructAircraftByCallsign called.', callsign, file=stderr)
	
	def postTextRadioMsg(self, msg):
		assert not msg.isPrivate()
		if self.studentConnected():
			s = msg.sender()
			t = msg.txtMsg()
			self.student.sendMessage(TeachingMsg(TeachingMsg.TEXT_MSG, data=(s + '\t' + t if s else t)))
		else:
			raise TextMsgBlocked('No student connected.')

	def postAtcChatMsg(self, msg):
		if not msg.isPrivate():
			raise TextMsgBlocked('Public ATC messaging disabled in tutoring sessions.')
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.TEXT_MSG, data=(msg.sender() + '\n' + msg.txtOnly())))
		else:
			raise TextMsgBlocked('No student connected.')

	# NOTE with teacher: ATC arg is who sends the strip to student (not who receives it)
	def sendStrip(self, strip, atc):
		if self.studentConnected():
			msg_data = atc + '\n' + strip.encodeDetails(handover_details)
			self.student.sendMessage(TeachingMsg(TeachingMsg.STRIP_EXCHANGE, data=msg_data))
		else:
			raise HandoverBlocked('No student connected.')
	
	def sendCpdlcMsg(self, callsign, msg):
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.CPDLC, data=('%s\n%s%s' % (callsign, CPDLC_cmd_prefix_msg, msg.toEncodedStr()))))
		else:
			raise CpdlcOperationBlocked('No student connected.')
	
	# NOTE with teacher: ATC is who is proposing/cancelling the transfer
	def sendCpdlcTransferRequest(self, acft_callsign, atc_callsign, proposing):
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.CPDLC,
					data=('%s\n%s%d %s' % (acft_callsign, CPDLC_cmd_prefix_xfr, proposing, atc_callsign))))
		else:
			raise CpdlcOperationBlocked('No student connected.')
	
	# NOTE with teacher: ATC is who is accepting/rejecting the transfer
	def sendCpdlcTransferResponse(self, acft_callsign, atc_callsign, accept):
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.CPDLC, data=('%s\n%s%d' % (acft_callsign, CPDLC_cmd_prefix_xfr, accept))))
		else:
			raise CpdlcOperationBlocked('No student connected.')
	
	# NOTE with teacher: this is ACFT disconnecting itself
	def sendCpdlcDisconnect(self, callsign):
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.CPDLC, data=('%s\n%s0' % (callsign, CPDLC_cmd_prefix_conn))))
		else:
			raise CpdlcOperationBlocked('No student connected.')
	
	
	## VOICE COMM'S
	
	def createRadio(self):
		QMessageBox.critical(self.gui, 'Create radio', 'No radio boxes in teaching sessions. '
				'Radio transmissions all happen on a single virtual frequency. Use keyboard PTT key to transmit. '
				'The signal source is the aircraft selected on PTT press, if any and if spawned (otherwise undetected).')
	
	def recordAtis(self, parent_dialog):
		pass

	def phoneLineManager(self):
		return self.phone_radio_manager # can be None
	
	
	## ONLINE SYSTEMS
	
	def weatherLookUpRequest(self, station):
		pass # weather never changes outside of teacher's action; no weather exists outside of primary station
	
	def pushFplOnline(self, fpl):
		if fpl.isOnline():
			fpl.modified_details.clear()
		else:
			used_IDs = {got.online_id for got in env.FPLs.findAll(pred=FPL.isOnline)}
			i = 0
			while '%s-%X' % (teacher_callsign, i) in used_IDs:
				i += 1
			fpl.markAsOnline('%s-%X' % (teacher_callsign, i))
		env.FPLs.refreshViews()
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.FLIGHT_PLAN, data=fpl.encode()))
	
	def changeFplStatus(self, fpl, new_status):
		fpl.setOnlineStatus(new_status)
		env.FPLs.refreshViews()
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.FLIGHT_PLAN, data=fpl.encode()))
	
	def syncOnlineFPLs(self):
		pass # teacher's online FPL list *is* the up-to-date online set
	
	
	## MANAGER-SPECIFIC
	
	def tickSessionOnce(self):
		for acft in pop_all(self.aircraft_list, lambda a: not env.pointInRadarRange(a.params.position)):
			self.killAircraft(acft) # this will send KILL to student
		send_traffic_this_tick = self.studentConnected() and self.noACK_traffic_count < max_noACK_traffic
		for acft in self.aircraft_list:
			acft.tickOnce()
			fgms_packet = acft.fgmsPositionPacket()
			send_packet_to_views(fgms_packet)
			if send_traffic_this_tick and acft.spawned:
				self.student.sendMessage(TeachingMsg(TeachingMsg.TRAFFIC, data=fgms_packet))
				self.noACK_traffic_count += 1

	def skipTimeForward(self, time_skipped):
		self.clock.offsetTime(time_skipped)
		for acft in self.getAircraft():
			acft.tickOnce()
		if self.studentConnected():
			self.sendCurrentTime()
		#FIXME instant radar sweep?
	
	
	# Teacher traffic/env. management
	
	def createNewTraffic(self, spawn_coords, spawn_hdg):
		dialog = CreateTrafficDialog(spawn_coords, spawn_hdg, parent=self.gui)
		dialog.exec()
		if dialog.result() > 0:
			params, status = dialog.acftInitParamsAndStatus()
			acft = ControlledAiAircraft(dialog.acftCallsign(), dialog.acftType(), params, status, None)
			acft.spawned = False
			acft.frozen = dialog.startFrozen()
			acft.tickOnce()
			self.aircraft_list.append(acft)
			if dialog.createStrip():
				strip = Strip()
				strip.writeDetail(FPL.CALLSIGN, acft.identifier)
				strip.writeDetail(FPL.ACFT_TYPE, acft.aircraft_type)
				strip.writeDetail(FPL.WTC, wake_turb_cat(acft.aircraft_type))
				strip.linkAircraft(acft)
				signals.receiveStrip.emit(strip)
			env.radar.scanSingleAcft(acft)
			selection.selectAircraft(acft)
	
	def killAircraft(self, acft):
		acft.resetPtt()
		pop_all(self.aircraft_list, lambda a: a is acft) # NOTE: might already be removed from list (in tickSessionOnce)
		if acft.spawned and self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.ACFT_KILLED, data=acft.identifier))
	
	def setWeather(self, weather): # NOTE: argument weather should be from primary station
		register_weather_information(weather)
		self.sendPrimaryWeather()
	
	def requestCpdlcLogOn(self, callsign): # NOTE: student must confirm log-on
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.CPDLC, data=('%s\n%s1' % (callsign, CPDLC_cmd_prefix_conn))))
		else:
			raise CpdlcOperationBlocked('No student connected.')
	
	
	# Snapshotting
	
	def situationSnapshot(self):
		return self.clock.readTime(), [acft.statusSnapshot() for acft in self.aircraft_list]
	
	def restoreSituation(self, situation_snapshot):
		time, traffic = situation_snapshot
		while self.aircraft_list:
			self.killAircraft(self.aircraft_list[0]) # sends info to student if ACFT known to them
		self.clock.setTime(time)
		if self.studentConnected():
			self.sendCurrentTime()
		for acft_snapshot in traffic:
			self.aircraft_list.append(ControlledAiAircraft.fromStatusSnapshot(acft_snapshot))
		self.tickSessionOnce()
	
	
	# Connection with student
	
	def studentConnected(self):
		return self.student_socket is not None
	
	def studentConnects(self):
		new_connection = self.server.nextPendingConnection()
		if new_connection:
			peer_address = new_connection.peerAddress().toString()
			print('Contacted by %s' % peer_address)
			if self.studentConnected():
				new_connection.disconnectFromHost()
				print('Client rejected. Student already connected.', file=stderr)
			else:
				self.student_socket = new_connection
				self.student_socket.disconnected.connect(self.studentDisconnects)
				self.student_socket.disconnected.connect(self.student_socket.deleteLater)
				self.student = TeachingSessionWire(self.student_socket)
				self.student.messageArrived.connect(self.receiveMsgFromStudent)
				self.noACK_traffic_count = 0
				self.sendCurrentTime()
				self.sendPrimaryWeather()
				self.sendATCs()
				for fpl in env.FPLs.findAll(pred=FPL.isOnline):
					self.student.sendMessage(TeachingMsg(TeachingMsg.FLIGHT_PLAN, data=fpl.encode()))
				if self.phone_radio_manager is not None:
					self.student.sendMessage(TeachingMsg(TeachingMsg.PHONE_REQUEST)) # with empty output = please answer if able
				self.tickSessionOnce()
				if self.clock.isPaused():
					self.student.sendMessage(TeachingMsg(TeachingMsg.SIM_PAUSED))
				QMessageBox.information(self.gui, 'Student connection', 'Student accepted from %s' % peer_address)
		else:
			print('WARNING: Connection attempt failed.', file=stderr)
	
	def studentDisconnects(self):
		self.shutdownStudentConnection()
		for strip in env.strips.listAll() + env.discarded_strips.listAll():
			strip.writeDetail(student_ok_detail, None)
		QMessageBox.information(self.gui, 'Student connection', 'Student has disconnected.')
	
	def shutdownStudentConnection(self):
		self.student_socket.disconnected.disconnect(self.studentDisconnects)
		if self.phone_radio_manager is not None:
			self.phone_radio_manager.deactivate()
		env.cpdlc.clearHistory()
		env.ATCs.removeATC(student_callsign)
		self.student.messageArrived.disconnect(self.receiveMsgFromStudent)
		self.student_socket.disconnectFromHost()
		self.student_socket = None

	def sendCurrentTime(self):
		if self.studentConnected():
			self.student.sendMessage(TeachingMsg(TeachingMsg.SET_CLOCK, data=self.clock.encodeTime()))

	def sendATCs(self):
		if self.studentConnected():
			msg = TeachingMsg(TeachingMsg.ATC_LIST)
			for atc in env.ATCs.knownAtcCallsigns():
				try:
					frq = env.ATCs.getATC(atc).frequency # CommFrequency or None
				except KeyError:
					frq = None
				msg.appendData(atc if frq is None else '%s\t%s' % (atc, frq))
				msg.appendData('\n')
			self.student.sendMessage(msg)
	
	def sendPrimaryWeather(self):
		w = env.primaryWeather()
		if self.studentConnected() and w is not None:
			self.student.sendMessage(TeachingMsg(TeachingMsg.WEATHER, data=w.METAR()))

	def sendPTT(self, on_off):
		if self.acft_transmitting is not None:
			self.acft_transmitting.resetPtt()
			if self.studentConnected():
				self.student.sendMessage(TeachingMsg(TeachingMsg.RADIO_PTT, data=('0 ' + self.acft_transmitting.identifier)))
			self.acft_transmitting = None
		if on_off and selection.acft is not None and selection.acft.spawned:
			self.acft_transmitting = selection.acft
			self.acft_transmitting.setPtt()
			if self.studentConnected():
				self.student.sendMessage(TeachingMsg(TeachingMsg.RADIO_PTT, data=('1 ' + self.acft_transmitting.identifier)))

	def phoneRadioMsgToSend(self, msg):
		if self.studentConnected():
			self.student.sendMessage(msg)
	
	def receiveMsgFromStudent(self, msg):
		#DEBUGif msg.type != TeachingMsg.TRAFFIC and msg.type != TeachingMsg.PHONE_AUDIO:
		#DEBUG	print('=== TEACHER RECEIVES ===\n%s\n=== End ===' % msg.output)
		if msg.type == TeachingMsg.TEXT_MSG:
			lines = msg.strData().split('\n')
			if len(lines) == 1: # text radio msg
				signals.incomingTextRadioMsg.emit(TextMessage(student_callsign, lines[0]))
			elif len(lines) == 2: # ATC private msg
				signals.incomingAtcTextMsg.emit(TextMessage(student_callsign, lines[1], recipient=lines[0], private=True))
			else:
				print('ERROR: Invalid format in received ATC text message from student.', file=stderr)
		
		elif msg.type == TeachingMsg.STRIP_EXCHANGE:
			line_sep = msg.strData().split('\n', maxsplit=1)
			toATC = line_sep[0]
			strip = Strip.fromEncodedDetails('' if len(line_sep) < 2 else line_sep[1])
			strip.writeDetail(received_from_detail, student_callsign)
			strip.writeDetail(sent_to_detail, toATC)
			signals.receiveStrip.emit(strip)
		
		elif msg.type == TeachingMsg.FLIGHT_PLAN:
			fpl = FPL.fromEncoded(msg.strData())
			if fpl.isOnline():
				env.FPLs.updateFromOnlineDownload(fpl)
			else:
				print('ERROR: Received an offline FPL from student.', file=stderr)
		
		elif msg.type == TeachingMsg.WEATHER: # requesting weather information
			if msg.strData() == settings.primary_METAR_station:
				self.sendPrimaryWeather()
		
		elif msg.type == TeachingMsg.TRAFFIC: # acknowledging a traffic message
			if self.noACK_traffic_count > 0:
				self.noACK_traffic_count -= 1
			else:
				print('ERROR: Student acknowledging unsent traffic?!', file=stderr)
		
		elif msg.type == TeachingMsg.CPDLC:
			# Student msg format in 2 lines, first being ACFT callsign, second is either of the following:
			#  - student disconnects or rejects output link: CPDLC_cmd_prefix_conn + "0"
			#  - student accepts ACFT log-on: CPDLC_cmd_prefix_conn + "1"
			#  - student proposes or cancels transfer: CPDLC_cmd_prefix_xfr + "0"/"1" + space + ATC callsign
			#  - student accepts or rejects our transfer proposal: CPDLC_cmd_prefix_xfr + "0"/"1"
			#  - other CPDLC message: CPDLC_cmd_prefix_msg + encoded message string
			try:
				acft_callsign, line2 = msg.strData().split('\n', maxsplit=1)
				link = env.cpdlc.lastDataLink(acft_callsign)
				if line2 == CPDLC_cmd_prefix_conn + '0':
					if link is None or link.isTerminated(): # student is rejecting a log-on
						QMessageBox.warning(self.gui, 'CPDLC connection failed', 'Student is not accepting CPDLC connections.')
					else: # ACFT disconnected by student
						link.markProblem('Student disconnected aircraft')
						link.terminate(True)
				elif line2 == CPDLC_cmd_prefix_conn + '1': # ACFT log-on confirmed
					env.cpdlc.beginDataLink(acft_callsign)
				elif line2.startswith(CPDLC_cmd_prefix_xfr):
					positive = line2[len(CPDLC_cmd_prefix_xfr)] == '1' # IndexError is guarded here
					if ' ' in line2: # student proposing or cancelling XFR
						atc = line2.split(' ', maxsplit=1)[1]
						if positive: # student proposing XFR
							signals.cpdlcDialogueRequest.emit(acft_callsign, False)
							if link is not None and link.isLive():
								accept = QMessageBox.question(self.gui, 'CPDLC transfer from student',
										'Accept output authority transfer for %s from student to %s?' % (acft_callsign, atc)) == QMessageBox.Yes
								if accept:
									link.setTransferTo(atc)
									link.terminate(True)
								try:
									self.sendCpdlcTransferResponse(acft_callsign, atc, accept)
								except CpdlcOperationBlocked as err:
									print('CPDLC error:', str(err), file=stderr)
							else:
								print('ERROR: Student proposing a transfer without output authority.', file=stderr)
						else:
							print('Student should not be able to abort a transfer.', file=stderr)
					else: # student accepting or rejecting XFR
						if link is None or link.pendingTransferFrom() is None:
							print('Ignored CPDLC transfer confirmed while none pending for %s.' % acft_callsign, file=stderr)
						elif positive:
							link.acceptIncomingTransfer()
						else: # student rejecting XFR
							link.markProblem('Student rejected transfer')
							link.terminate(False)
				elif line2.startswith(CPDLC_cmd_prefix_msg): # ACFT sending a message
					encoded_msg = line2[len(CPDLC_cmd_prefix_msg):]
					link = env.cpdlc.liveDataLink(acft_callsign)
					if link is None:
						print('Ignored CPDLC message sent from %s while not connected.' % acft_callsign, file=stderr)
					else:
						link.appendMessage(CpdlcMessage.fromEncodedStr(encoded_msg))
				else:
					print('Error decoding CPDLC command from student:', line2, file=stderr)
			except (IndexError, ValueError):
				print('Error decoding CPDLC message from student', file=stderr)

		elif msg.type == TeachingMsg.RADIO_AUDIO:
			if self.phone_radio_manager is not None and not settings.radios_silenced and not self.phone_radio_manager.radioPTT():
				self.phone_radio_manager.receiveAudioData(msg.binData())

		elif msg.type == TeachingMsg.PHONE_REQUEST:
			if self.phone_radio_manager is not None:
				tokens = msg.strData().split(maxsplit=1)
				if len(tokens) == 0: # student answering that audio is supported on their side
					self.phone_radio_manager.activate()
				elif len(tokens) == 2 and tokens[0] == '0':
					self.phone_radio_manager.incomingLineDrop(tokens[1])
				elif len(tokens) == 2 and tokens[0] == '1':
					self.phone_radio_manager.incomingLineRequest(tokens[1])
				else:
					print('Error decoding phone request/drop message from student', file=stderr)

		elif msg.type == TeachingMsg.PHONE_AUDIO:
			if self.phone_radio_manager is not None:
				self.phone_radio_manager.receiveAudioData(msg.binData())

		elif msg.type == TeachingMsg.ATCPIE_VERSION:
			txt = 'Student connected with using ATC-Pie version %s.' % msg.strData()
			signals.statusBarMsg.emit(txt)
			print(txt)

		else:
			print('Unhandled message type from student: %s' % msg.type, file=stderr)
