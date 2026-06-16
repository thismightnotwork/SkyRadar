
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
from random import random, randint, choice, uniform
from datetime import timedelta

from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer

from ai.controlledAircraft import ControlledAiAircraft, GS_alt, default_initial_climb_spec
from ai.distractorAircraft import DistractorAiAircraft
from ai.status import Status, FlightParams

from base.conflict import ground_separated
from base.cpdlc import CpdlcMessage, RspId
from base.db import all_aircraft_types, all_airline_codes, cruise_speed, wake_turb_cat, acft_cat
from base.fpl import FPL
from base.instr import Instruction, ApproachType
from base.nav import Navpoint, world_navpoint_db, world_routing_db
from base.params import Heading, PressureAlt, Speed, distance_travelled, time_to_fly
from base.strip import Strip, received_from_detail, assigned_SQ_detail, assigned_altitude_detail
from base.text import TextMessage
from base.utc import VirtualClock
from base.util import some, pop_all, bounded
from base.weather import mkWeather

from session.config import settings, SemiCircRule, XpdrAssignmentRange
from session.env import env, generate_unused_callsign, CallsignGenerationError
from session.manager import SessionManager, SessionType, TextMsgBlocked, HandoverBlocked, CpdlcOperationBlocked, OnlineFplActionBlocked

from ext.fgfs import send_packet_to_views, FGFS_model_liveries
from ext.sr import speech_recognition_available, InstructionRecogniser, radio_callsign_match, write_radio_callsign
from ext.tts import speech_synthesis_available, SpeechSynthesiser, speech_str2txt

from gui.actions import register_weather_information
from gui.graphics.airport import intercept_cone_half_angle
from gui.misc import signals
from gui.widgets.basicWidgets import Ticker


# ---------- Constants ----------

solo_ticker_interval = 50 # ms
cpdlc_recv_countdown_interval = 5000 # ms

exit_point_tolerance = 10 # NM
initial_climb_angle = 6 # %
TTF_separation = timedelta(minutes=2)
max_attempts_for_aircraft_spawn = 5
turn_off_options_for_GND_arrival = 3

XPDR_range_IFR_DEP = XpdrAssignmentRange('Auto-generated IFR DEP', 0o2101, 0o2177, None)
XPDR_range_IFR_ARR = XpdrAssignmentRange('Auto-generated IFR APP', 0o3421, 0o3477, None)
XPDR_range_IFR_transit = XpdrAssignmentRange('Auto-generated IFR transit', 0o3001, 0o3077, None)

# -------------------------------

class AcftSpawnError(Exception):
	pass



class SoloSessionManager(SessionManager):
	"""
	VIRTUAL!
	Subclass and define methods:
	- generateAircraftAndStrip(): return (ACFT, Strip, str) tuple of possibly None values if unsuccessful
	- handoverGuard(cs, atc): return str error msg if handover not OK
	"""
	def __init__(self, gui):
		SessionManager.__init__(self, gui, SessionType.SOLO)
		self.session_ticker = Ticker(gui, self.tickSessionOnce)
		self.weather_ticker = Ticker(gui, self.setNewWeather)
		self.spawn_timer = QTimer(gui)
		self.spawn_timer.setSingleShot(True)
		self.voice_instruction_recogniser = None # CAUTION accessed outside of class
		self.speech_synthesiser = None
		self.clock = VirtualClock()
		self.init_traffic_count = 0
		if speech_recognition_available:
			try:
				self.voice_instruction_recogniser = InstructionRecogniser(gui)
			except RuntimeError as err:
				settings.solo_voice_instructions = False
				QMessageBox.critical(self.gui, 'Sphinx error',
					'Error setting up the speech recogniser (check log): %s\nVoice instructions disabled.' % err)
		if speech_synthesis_available:
			try:
				self.speech_synthesiser = SpeechSynthesiser(gui)
			except Exception as err:
				settings.solo_voice_readback = False
				QMessageBox.critical(self.gui, 'Pyttsx error',
					'Error setting up the speech synthesiser: %s\nPilot read-back disabled.' % err)
		self.controlled_traffic = []
		self.uncontrolled_traffic = []
		self.cpdlc_msg_queue = [] # (acft, link, msg) list
		self.cpdlc_xfr_queue = [] # (acft_callsign, atc_callsign) list
		self.cpdlc_recv_countdown = 0 # to add some delay to CPDLC responses from ACFT
		self.spawn_timer.timeout.connect(self.spawnNewControlledAircraftIfNeeded)
		self.playable_aircraft_types = settings.solo_aircraft_types[:]
		self.uncontrolled_aircraft_types = [t for t in all_aircraft_types() if cruise_speed(t) is not None]
		pop_all(self.playable_aircraft_types, lambda t: t not in all_aircraft_types())
		pop_all(self.playable_aircraft_types, lambda t: cruise_speed(t) is None)

	def generateAircraftAndStrip(self):
		raise NotImplementedError()

	def handoverGuard(self, acft, atc_callsign):
		raise NotImplementedError()
	
	def start(self):
		if len(self.playable_aircraft_types) == 0:
			QMessageBox.critical(self.gui, 'Solo session error', 'Cannot start simulation: not enough playable aircraft types.')
			env.ATCs.clear()
			return
		if self.voice_instruction_recogniser is not None:
			self.voice_instruction_recogniser.startup()
			signals.kbdPTT.connect(self.voicePTT)
		if self.speech_synthesiser is not None:
			self.speech_synthesiser.startup()
			signals.voiceMsg.connect(self.speech_synthesiser.radioMsg)
		self.controlled_traffic.clear()
		self.uncontrolled_traffic.clear()
		for i in range(self.init_traffic_count):
			self.spawnNewControlledAircraft(isSessionStart=True)
		self.adjustDistractorCount()
		self.setNewWeather()
		self.session_ticker.startTicking(solo_ticker_interval)
		self.startStopWeatherTicker()
		signals.voiceMsgRecognised.connect(self.handleVoiceInstrMessage)
		signals.locationSettingsChanged.connect(self.setNewWeather) # in case primary weather station changed
		signals.soloRuntimeSettingsChanged.connect(self.startStopWeatherTicker)
		signals.soloRuntimeSettingsChanged.connect(self.adjustDistractorCount)
		signals.sessionStarted.emit(SessionType.SOLO)
		print('Solo simulation begins.')
	
	def stop(self):
		if self.isRunning():
			signals.voiceMsgRecognised.disconnect(self.handleVoiceInstrMessage)
			signals.locationSettingsChanged.disconnect(self.setNewWeather)
			signals.soloRuntimeSettingsChanged.disconnect(self.startStopWeatherTicker)
			signals.soloRuntimeSettingsChanged.disconnect(self.adjustDistractorCount)
			if self.voice_instruction_recogniser is not None:
				signals.kbdPTT.disconnect(self.voicePTT)
				self.voice_instruction_recogniser.shutdown()
				self.voice_instruction_recogniser.wait()
			if self.speech_synthesiser is not None:
				signals.voiceMsg.disconnect(self.speech_synthesiser.radioMsg)
				self.speech_synthesiser.shutdown()
				self.speech_synthesiser.wait()
			self.spawn_timer.stop()
			self.weather_ticker.stop()
			self.session_ticker.stop()
			self.controlled_traffic.clear()
			self.uncontrolled_traffic.clear()
			signals.sessionEnded.emit(SessionType.SOLO)
	
	def pause(self):
		if self.isRunning() and not self.clock.isPaused():
			self.session_ticker.stop()
			self.clock.pause()
			signals.sessionPaused.emit()
	
	def resume(self):
		if self.clock.isPaused():
			self.clock.resume()
			self.session_ticker.startTicking(solo_ticker_interval)
			signals.sessionResumed.emit()

	def isRunning(self):
		return self.session_ticker.isActive() or self.clock.isPaused()

	def clockTime(self):
		return self.clock.readTime()
	
	def getAircraft(self):
		return self.controlled_traffic + self.uncontrolled_traffic
	
	
	## ACFT/ATC INTERACTION
	
	def instructAircraftByCallsign(self, callsign, instr):
		if not self.instrExpectedByVoice(instr.type):
			self._instructSequence([instr], callsign, False)
	
	def postTextRadioMsg(self, msg):
		raise TextMsgBlocked('Text messages not supported in solo sessions.')
	
	def postAtcChatMsg(self, msg):
		raise TextMsgBlocked('ATC text messaging not available in solo sessions.')
	
	def sendStrip(self, strip, atc):
		if not self.instrExpectedByVoice(Instruction.HAND_OVER):
			cs = strip.callsign()
			try:
				acft = next(a for a in self.controlled_traffic if a.identifier == cs)
				guard = self.handoverGuard(acft, atc)
				if guard is not None:
					raise HandoverBlocked(guard)
			except StopIteration: # strip for which no ACFT makes sense; just let it be sent
				return
	
	def sendCpdlcMsg(self, callsign, msg):
		link = env.cpdlc.liveDataLink(callsign)
		if link is None:
			raise CpdlcOperationBlocked('No output link established with ' + callsign)
		try:
			acft = next(a for a in self.controlled_traffic if a.identifier == callsign) # uncontrolled traffic is not in contact
			if msg.expectsAnswer():
				self.cpdlc_msg_queue.append((acft, link, msg))
		except StopIteration: # ACFT not found or not connected
			print('ERROR: Aircraft %s not found for solo CPDLC message.' % callsign, file=stderr)
	
	def sendCpdlcTransferRequest(self, acft_callsign, atc_callsign, proposing):
		if proposing:
			try:
				acft = next(a for a in self.controlled_traffic if a.identifier == acft_callsign)
				guard = self.handoverGuard(acft, atc_callsign)
				if guard is None:
					self.cpdlc_xfr_queue.append((acft, atc_callsign))
				else:
					raise CpdlcOperationBlocked(guard)
			except StopIteration:
				raise CpdlcOperationBlocked('XFR of unknown aircraft "%s" in solo. Internal error?' % acft_callsign)
		else: # cancelling XFR
			raise CpdlcOperationBlocked('Cancelled XFR in solo. Internal error?')
	
	def sendCpdlcTransferResponse(self, acft_callsign, atc_callsign, accept):
		pass # nothing to do here
	
	def sendCpdlcDisconnect(self, callsign):
		pass # nothing to do here: now switch to voice/mouse
	
	
	## VOICE COMM'S
	
	def createRadio(self):
		QMessageBox.critical(self.gui, 'Create radio', 'No radio boxes in solo sessions. All transmissions happen '
				'on a single virtual frequency. When using voice instructions, use the keyboard PTT key to transmit.')
	
	def recordAtis(self, parent_dialog):
		pass
	
	
	## ONLINE SYSTEMS
	
	def weatherLookUpRequest(self, station):
		pass # useless: weather never changes outside of ticker's call to "setNewWeather" and no other weather exists than at primary station
	
	def pushFplOnline(self, fpl):
		raise OnlineFplActionBlocked('No online FPL system in solo sessions.')
	
	def changeFplStatus(self, fpl, new_status):
		raise OnlineFplActionBlocked('No online FPL system in solo sessions.')
	
	def syncOnlineFPLs(self):
		raise OnlineFplActionBlocked('No online FPL system in solo sessions.')
	
	
	## MANAGER-SPECIFIC
	
	def tickSessionOnce(self):
		if self.controlledAcftNeeded() and not self.spawn_timer.isActive():
			delay = randint(int(settings.solo_min_spawn_delay.total_seconds()), int(settings.solo_max_spawn_delay.total_seconds()))
			self.spawn_timer.start(1000 * delay)
		self.adjustDistractorCount()
		pop_all(self.controlled_traffic, lambda a: a.released or not env.pointInRadarRange(a.params.position))
		pop_all(self.uncontrolled_traffic, lambda a: a.outlived() or not env.pointInRadarRange(a.params.position))
		for acft in self.getAircraft():
			acft.tickOnce()
			send_packet_to_views(acft.fgmsPositionPacket())
		if self.cpdlc_recv_countdown == 0:
			self.digestCpdlcQueues()
			self.cpdlc_recv_countdown = cpdlc_recv_countdown_interval // solo_ticker_interval
		else:
			self.cpdlc_recv_countdown -= 1

	def skipTimeForward(self, time_skipped):
		self.clock.offsetTime(time_skipped)
		for acft in self.getAircraft():
			acft.tickOnce()
		#FIXME instant radar sweep?
	
	def digestCpdlcQueues(self):
		while len(self.cpdlc_msg_queue) > 0:
			acft, link, msg = self.cpdlc_msg_queue.pop(0)
			upinstr = msg.recognisedInstructions()
			if upinstr is None:
				if not msg.isAcknowledgement():
					link.appendMessage(CpdlcMessage([RspId.downlink_UNABLE, 'SUPD-1 COULD NOT INTERPRET, NO HUMAN ON BOARD.']))
			else:
				try:
					acft.instruct(upinstr, False)
				except Instruction.Error as err:
					link.appendMessage(CpdlcMessage([RspId.downlink_UNABLE, 'SUPD-1 ' + str(err).upper()]))
				else: # instruction sent and already accepted
					link.appendMessage(CpdlcMessage(RspId.downlink_WILCO))
		while len(self.cpdlc_xfr_queue) > 0:
			acft, atc_callsign = self.cpdlc_xfr_queue.pop(0)
			try:
				acft.instruct([env.ATCs.handoverInstructionTo(atc_callsign)], False)
				signals.cpdlcTransferResponse.emit(acft.identifier, atc_callsign, True)
			except Instruction.Error: # unlikely (new guard since last check before queuing)
				signals.cpdlcTransferResponse.emit(acft.identifier, atc_callsign, False)
	
	def startStopWeatherTicker(self):
		if settings.solo_weather_change_interval is None:
			self.weather_ticker.stop()
		else:
			self.weather_ticker.startTicking(settings.solo_weather_change_interval, immediate=False)
	
	def setNewWeather(self):
		weather = env.primaryWeather()
		wind_info = None if weather is None else weather.mainWind()
		if wind_info is None:
			# fixed for whole session
			self.qnh = randint(1005, 1025)
			self.temperature = randint(12, 25)
			self.dew_point = self.temperature - randint(5, 10)
			self.visibility = 1000 * randint(5, 15)
			self.clouds = choice(['NSC', 'FEW', 'SCT', 'BKN', 'OVC'])
			if self.clouds != 'NSC':
				self.clouds += '%03d' % randint(8, 80)
			# wind changes during session
			w1 = 10 * randint(1, 36)
			w2 = randint(5, 20) # avoid calm or VRB because main wind assumed not None below
			if env.airport_data is not None and \
					not any(rwy.inUse() and abs(w1 - rwy.orientation().trueAngle()) <= 90 for rwy in env.airport_data.directionalRunways()):
				w1 += 180
		else:
			whdg, wspd, gusts, unit = wind_info
			w1 = whdg.trueAngle() + 10 * randint(-1, 1) # whdg should not be None
			w2 = bounded(5, Speed(wspd, unit=unit).kt() + randint(-4, 4), 20)
		register_weather_information(mkWeather(settings.primary_METAR_station, self.clock.readTime(),
				wind=('%03d%02dKT' % ((w1 - 1) % 360 + 1, w2)), qnh=self.qnh,
				clouds=self.clouds, vis=self.visibility, temp=self.temperature, dp=self.dew_point))
	
	
	# Spawning aircraft
	
	def controlledAcftNeeded(self):
		return len(self.controlled_traffic) < settings.solo_max_aircraft_count
	
	def killAircraft(self, acft):
		if len(pop_all(self.controlled_traffic, lambda a: a is acft)) == 0:
			pop_all(self.uncontrolled_traffic, lambda a: a is acft)
	
	def adjustDistractorCount(self):
		while len(self.uncontrolled_traffic) > settings.solo_distracting_traffic_count: # too many uncontrolled ACFT
			self.killAircraft(self.uncontrolled_traffic[0])
		for i in range(settings.solo_distracting_traffic_count - len(self.uncontrolled_traffic)): # uncontrolled ACFT needed
			self.spawnNewUncontrolledAircraft()
	
	def spawnNewUncontrolledAircraft(self):
		rndpos = env.radarPos().moved(Heading(randint(1, 360), True), uniform(10, .8 * settings.radar_range))
		rndalt = PressureAlt(randint(1, 10) * 1000)
		if self.airbornePositionFullySeparated(rndpos, rndalt):
			acft_type = choice(self.uncontrolled_aircraft_types)
			params = FlightParams(rndpos, rndalt, Heading(randint(1, 360), True), cruise_speed(acft_type).tas2ias(rndalt), xpdrCode=settings.uncontrolled_VFR_XPDR_code)
			new_acft = self.mkAiAcft(acft_type, params, None, None)
			if new_acft is not None:
				self.uncontrolled_traffic.append(new_acft)
	
	def spawnNewControlledAircraftIfNeeded(self):
		if self.controlledAcftNeeded() and not self.clock.isPaused():
			self.spawnNewControlledAircraft()
	
	def spawnNewControlledAircraft(self, isSessionStart=False):
		new_acft = strip = radio_tts = None
		attempts = 0
		while new_acft is None and attempts < max_attempts_for_aircraft_spawn:
			try:
				new_acft, strip, radio_tts = self.generateAircraftAndStrip()
			except AcftSpawnError:
				attempts += 1
		if new_acft is not None and strip is not None and not self.clock.isPaused():
			self.controlled_traffic.append(new_acft)
			receiving_from = strip.lookup(received_from_detail)
			if settings.controller_pilot_data_link and receiving_from is not None and random() <= settings.solo_CPDLC_balance:
				if isSessionStart:
					env.cpdlc.beginDataLink(new_acft.identifier, transferFrom=receiving_from, autoAccept=True)
				else:
					signals.cpdlcTransferRequest.emit(new_acft.identifier, receiving_from, True)
			if isSessionStart:
				strip.linkAircraft(new_acft)
				strip.writeDetail(received_from_detail, None)
			signals.receiveStrip.emit(strip)
			if env.cpdlc.liveDataLink(new_acft.identifier) is None:
				new_acft.say('Hello, ' + radio_tts, False, initAddressee=settings.location_radio_name)
	
	def airbornePositionFullySeparated(self, pos, alt):
		return all(pos.distanceTo(acft.params.position) >= settings.horizontal_separation
				or abs(alt.diff(acft.params.altitude)) >= settings.vertical_separation for acft in self.getAircraft() if not acft.status.snapped_GND)
	
	def groundPositionFullySeparated(self, pos, t):
		return all(ground_separated(acft, pos, t) for acft in self.getAircraft() if acft.status.snapped_GND)
	
	def mkAiAcft(self, acft_type, params, status, goal):
		"""
		use with status and goal set to None to create a DistractorAiAircraft; otherwise ControlledAiAircraft
		returns None if something prevented fresh ACFT creation
		"""
		if acft_cat(acft_type) in ['jets', 'heavy']:
			params.XPDR_mode = 'S'
		airlines = all_airline_codes()
		if env.airport_data is not None: # might be rendering in tower view, prefer ACFT with known liveries
			liveries_for_acft = FGFS_model_liveries.get(acft_type, {})
			if len(liveries_for_acft) > 0 and settings.solo_restrict_to_available_liveries:
				pop_all(airlines, lambda al: al not in liveries_for_acft)
		try:
			callsign = generate_unused_callsign(acft_type, airlines)
			if status is None and goal is None:
				ms_to_live = 1000 * 60 * randint(10, 60 * 3)
				return DistractorAiAircraft(callsign, acft_type, params, ms_to_live // solo_ticker_interval)
			else:
				return ControlledAiAircraft(callsign, acft_type, params, status, goal)
		except CallsignGenerationError:
			raise AcftSpawnError()
	
	
	## Instructions
	
	def instrExpectedByVoice(self, itype):
		return settings.solo_voice_instructions \
			and itype in [Instruction.VECTOR_HDG, Instruction.VECTOR_ALT, Instruction.VECTOR_SPD, Instruction.HAND_OVER]
	
	def voicePTT(self, toggle):
		if self.voice_instruction_recogniser is not None and settings.solo_voice_instructions and not self.clock.isPaused():
			if toggle:
				self.voice_instruction_recogniser.keyIn()
			else:
				self.voice_instruction_recogniser.keyOut()
	
	def rejectInstruction(self, msg):
		if settings.solo_erroneous_instruction_warning:
			QMessageBox.warning(self.gui, 'Erroneous/rejected instruction', msg)
	
	def _instructSequence(self, instructions, callsign, is_voice):
		acft = next((a for a in self.controlled_traffic if a.identifier == callsign), None) # uncontrolled traffic is not in contact
		txt = instructions[0].readOutStr(acft) if len(instructions) == 1 else ' '.join(instr.readOutStr(acft) + '.' for instr in instructions)
		msg = TextMessage(settings.my_callsign, txt, recipient=callsign)
		if settings.solo_voice_instructions:
			msg.setDispPrefix('MV'[is_voice])
		signals.incomingTextRadioMsg.emit(msg) # not really "incoming" but this will collect in the radio msg history
		if acft is None:
			self.rejectInstruction('Nobody answering callsign %s' % callsign)
		else:
			try:
				acft.instruct(instructions, True)
				if settings.solo_wilco_beeps:
					signals.wilco.emit()
			except Instruction.Error as err:
				self.rejectInstruction('%s: "%s"' % (callsign, speech_str2txt(str(err))))
	
	def handleVoiceInstrMessage(self, radio_callsign_tokens, instructions):
		acft_matches = [acft for acft in self.getAircraft() if radio_callsign_match(radio_callsign_tokens, acft.identifier)]
		if len(acft_matches) < 2:
			if len(acft_matches) == 0: # nobody will respond but OK to send
				callsign_to_instruct = write_radio_callsign(radio_callsign_tokens) if radio_callsign_tokens else ''
			else: # perfect match; will respond to instruction
				callsign_to_instruct = acft_matches[0].identifier
			self._instructSequence(instructions, callsign_to_instruct, True)
		else: # too many matches; block instruction
			acft_matches[0].say('Sorry, was this for me?', True)
			self.rejectInstruction('Used callsign matches several: %s' % ', '.join(acft.identifier for acft in acft_matches))







# -----------------------------------------------------------------


def restrict_speed_under_ceiling(spd, alt, ceiling):
	if alt.diff(ceiling) <= 0:
		return Speed(min(spd.kt(), 250))
	else:
		return spd


def local_ee_point_closest_to(ad, exit_wanted):
	if exit_wanted:
		lst = world_routing_db.exitsFrom(env.airport_data.navpoint)
	else: # entry point wanted
		lst = world_routing_db.entriesTo(env.airport_data.navpoint)
	if len(lst) == 0:
		return None
	else:
		return min((p for p, legspec in lst), key=(lambda p: ad.coordinates.distanceTo(p.coordinates)))


def choose_dep_dest_AD(is_arrival):
	if settings.solo_prefer_entry_exit_ADs:
		ads = None
		if is_arrival and len(world_routing_db.entriesTo(env.airport_data.navpoint)) > 0: # pick a departure AD with exit points
			ads = world_routing_db.airfieldsWithExitPoints()
		elif not is_arrival and len(world_routing_db.exitsFrom(env.airport_data.navpoint)) > 0: # pick a dest. AD with entry points
			ads = world_routing_db.airfieldsWithEntryPoints()
		if ads is not None:
			try:
				return choice(list(ad for ad in ads if ad.code != env.airport_data.navpoint.code))
			except IndexError: # raised by random.choice on empty sequence
				pass # fall back on a random world airport
	return choice(world_navpoint_db.byType(Navpoint.AD))


def inTWRrange(params):
	return params.position.distanceTo(env.radarPos()) <= settings.solo_TWR_range_dist \
		and params.altitude.diff(PressureAlt.fromFL(settings.solo_TWR_ceiling_FL)) < 0


def rnd_dep_ldg_sfc(acft_type, dep=False, arr=False, requireILS=False):
	"""
	Picks a DepLdgSurface satisfying the given condition.
	"""
	if acft_cat(acft_type) == 'helos' and not requireILS: # prefer a helipad marked in use if ACFT is a helo
		hpads = [hpad for hpad in env.airport_data.helipads() if hpad.inUse()]
		if hpads:
			return choice(hpads)
	choose_from = [rwy for rwy in env.airport_data.directionalRunways() if dep and rwy.use_for_departures or arr and rwy.use_for_arrivals]
	if not choose_from: # Choose any from current wind
		w = env.primaryWeather()
		main_wind = None if w is None else w.mainWind()
		main_wind_hdg = Heading(360, True) if main_wind is None else main_wind[0]
		choose_from = [rwy for rwy in env.airport_data.directionalRunways() if abs(main_wind_hdg.diff(rwy.orientation())) <= 90]
	filtered = [rwy for rwy in choose_from if rwy.acceptsAcftType(acft_type) and (not requireILS or rwy.hasILS())]
	if not filtered and requireILS:
		filtered = [rwy for rwy in env.airport_data.directionalRunways() if rwy.acceptsAcftType(acft_type) and rwy.hasILS()]
	return choice(filtered) if filtered else None


def rnd_arrival(acceptable_acft_types):
	acft_type = choice(acceptable_acft_types)
	is_helo = acft_cat(acft_type) == 'helos'
	ils = (not is_helo or settings.solo_helos_request_ILS) and random() >= settings.solo_ILSvsVisual_balance
	sfc = rnd_dep_ldg_sfc(acft_type, arr=True, requireILS=ils)
	if sfc is None:
		raise AcftSpawnError()
	if ils:
		return acft_type, sfc, ApproachType.ILS
	else:
		return acft_type, sfc, ApproachType.STRAIGHT_IN if is_helo else ApproachType.VISUAL










# -----------------------------------------------------------------


class SoloSessionManager_AD(SoloSessionManager):
	def __init__(self, gui, init_traffic_count):
		SoloSessionManager.__init__(self, gui)
		self.init_traffic_count = init_traffic_count

	def start(self): # overrides (but calls) parent's
		self.parkable_aircraft_types = \
			[t for t in self.playable_aircraft_types if env.airport_data.ground_net.parkingPositions(acftType=t) != []]
		# Start errors (cancels start)
		if settings.solo_role_GND and self.parkable_aircraft_types == []:
			QMessageBox.critical(self.gui, 'Insufficient ground output', 'You cannot play solo GND with no parkable ACFT type.')
			return
		# Start warnings
		if (settings.solo_role_GND or settings.solo_role_TWR) and settings.radar_signal_floor_level > max(0, env.airport_data.field_elevation):
			QMessageBox.warning(self.gui, 'Radar visibility warning', 'You are assuming TWR/GND with radar signal floor above surface.')
		if settings.solo_role_DEP and settings.solo_ARRvsDEP_balance == 0:
			QMessageBox.warning(self.gui, 'No departures warning', 'You are assuming DEP with no departures set.')
		if settings.solo_role_APP and settings.solo_ARRvsDEP_balance == 1:
			QMessageBox.warning(self.gui, 'No arrivals warning', 'You are assuming APP with no arrivals set.')
		# Set up ATC neighbours
		env.ATCs.updateATC('CTR', env.radarPos(), 'En-route control centre', None)
		if settings.solo_role_GND:
			env.ATCs.updateATC('Ramp', None, 'Apron/gate services', None)
		else:
			env.ATCs.updateATC('GND', None, 'Airport ground', None)
		if not settings.solo_role_TWR:
			env.ATCs.updateATC('TWR', None, 'Tower', None)
		if not settings.solo_role_APP:
			env.ATCs.updateATC('APP', None, 'Approach', None)
		if not settings.solo_role_DEP:
			env.ATCs.updateATC('DEP', None, 'Departure', None)
		SoloSessionManager.start(self)

	def handoverGuard(self, acft, next_atc):
		# Bad or untimely handovers
		if next_atc == 'Ramp':
			if not acft.wantsToPark():
				return 'Aircraft does not want to park!'
			if not acft.canPark():
				return 'Bring aircraft close to parking position before handing over to ramp.'
		elif next_atc == 'GND':
			if not acft.status.snapped_GND or acft.status.ready_for_DEP or acft.status.rolling_TKOF_LDG:
				return 'Ground only accepts taxiing aircraft.'
		elif next_atc == 'TWR':
			if acft.isInboundGoal():
				if not inTWRrange(acft.params):
					return 'Not in TWR range.'
			elif not acft.status.ready_for_DEP:
				return 'Aircraft has not reported ready for departure.'
		elif next_atc == 'APP':
			if not acft.isInboundGoal():
				return 'Why hand over to APP?!'
			elif inTWRrange(acft.params):
				return 'This aircraft is in TWR range.'
		elif next_atc == 'DEP':
			if acft.isInboundGoal():
				return 'DEP only controls departures!'
			elif inTWRrange(acft.params):
				return 'TWR must keep control of aircraft until they fly out of tower range.'
		elif next_atc == 'CTR':
			if acft.isInboundGoal():
				return 'This aircraft is inbound your airport.'
			if settings.solo_role_DEP:
				point, alt, dest = acft.goal
				if point is None: # no specific exit point; check only (vaguely) direction
					if acft.params.position.distanceTo(dest.coordinates) > env.radarPos().distanceTo(dest.coordinates):
						return 'Not vectored towards destination airport %s.' % dest
				else: # exit navpoint specified; ACFT must be close enough for handoff
					if acft.params.position.distanceTo(point.coordinates) > exit_point_tolerance:
						return 'Not close enough to exit point %s.' % point
				if acft.params.altitude.diff(PressureAlt.fromFL(settings.solo_APP_ceiling_FL_min)) < 0:
					return 'Not high enough for CTR: reach FL%03d before handing over.' % settings.solo_APP_ceiling_FL_min
			else:
				return 'You should not be handing over to the centre directly.'
		else:
			print('INTERNAL ERROR: Please report unexpected ATC name "%s" in solo mode' % next_atc, file=stderr)


	def generateAircraftAndStrip(self):
		is_arrival = random() >= settings.solo_ARRvsDEP_balance
		if is_arrival:
			dep_ad = choose_dep_dest_AD(True)
			dest_ad = env.airport_data.navpoint
			midpoint = local_ee_point_closest_to(dep_ad, False) # None if none found
			if settings.solo_role_APP:
				new_acft, radio_tts = self.new_arrival_APP(midpoint) # may raise AcftSpawnError
				received_from = 'CTR'
			elif settings.solo_role_TWR:
				new_acft, radio_tts = self.new_arrival_TWR() # may raise AcftSpawnError
				received_from = 'APP'
			elif settings.solo_role_GND:
				new_acft, radio_tts = self.new_arrival_GND() # may raise AcftSpawnError
				received_from = 'TWR'
			else: # assuming only DEP; must not handle an arrival
				raise AcftSpawnError()
		else: # Create a departure
			dep_ad = env.airport_data.navpoint
			dest_ad = choose_dep_dest_AD(False)
			midpoint = local_ee_point_closest_to(dest_ad, True) # None if none found
			if settings.solo_role_GND:
				new_acft, radio_tts = self.new_departure_GND(midpoint, dest_ad) # may raise AcftSpawnError
				received_from = 'DEL'
			elif settings.solo_role_TWR:
				new_acft, radio_tts = self.new_departure_TWR(midpoint, dest_ad) # may raise AcftSpawnError
				received_from = 'GND'
			elif settings.solo_role_DEP:
				new_acft, radio_tts = self.new_departure_DEP(midpoint, dest_ad) # may raise AcftSpawnError
				received_from = 'TWR'
			else: # assuming only APP; must not handle a departure
				raise AcftSpawnError()
		strip = Strip()
		strip.writeDetail(FPL.CALLSIGN, new_acft.identifier)
		strip.writeDetail(FPL.ACFT_TYPE, new_acft.aircraft_type)
		strip.writeDetail(FPL.WTC, wake_turb_cat(new_acft.aircraft_type))
		strip.writeDetail(FPL.FLIGHT_RULES, 'IFR')
		strip.writeDetail(assigned_SQ_detail, new_acft.params.XPDR_code)
		strip.writeDetail(received_from_detail, received_from)
		if received_from == 'CTR':
			strip.writeDetail(assigned_altitude_detail, env.specifyAltFl(new_acft.params.altitude))
		elif received_from == 'TWR' and not settings.solo_role_GND: # receiving as DEP
			strip.writeDetail(assigned_altitude_detail, default_initial_climb_spec()) # CAUTION: also instructed to ACFT
		# routing details
		strip.writeDetail(FPL.ICAO_DEP, dep_ad.code)
		strip.writeDetail(FPL.ICAO_ARR, dest_ad.code)
		if is_arrival and midpoint is not None: # arrival with local entry point
			try:
				strip.writeDetail(FPL.ROUTE, world_routing_db.shortestRouteStr(dep_ad, midpoint) + ' ' + midpoint.code)
			except ValueError:
				strip.writeDetail(FPL.ROUTE, 'DCT %s' % midpoint.code)
		elif not is_arrival and midpoint is not None: # departure with local exit point
			try:
				strip.writeDetail(FPL.ROUTE, midpoint.code + ' ' + world_routing_db.shortestRouteStr(midpoint, dest_ad))
			except ValueError:
				strip.writeDetail(FPL.ROUTE, '%s DCT' % midpoint.code)
		return new_acft, strip, radio_tts


	## GENERATING DEPARTURES

	def new_departure_GND(self, goal_point, dest_AD):
		acft_type = choice(self.parkable_aircraft_types)
		pkglst = [p for p in env.airport_data.ground_net.parkingPositions(acftType=acft_type)
				if self.groundPositionFullySeparated(env.airport_data.ground_net.parkingPosition(p), acft_type)]
		if not pkglst: # nowhere to park
			raise AcftSpawnError()
		pkg = choice(pkglst)
		pkginfo = env.airport_data.ground_net.parkingPosInfo(pkg)
		params = FlightParams(pkginfo[0], env.groundPressureAlt(pkginfo[0]), pkginfo[1], Speed(0), xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_DEP))
		return self.mkAiAcft(acft_type, params, Status(airborne=False), (goal_point, None, dest_AD)), 'standing at \\SPELL_ALPHANUMS{%s}, ready to taxi' % pkg

	def new_departure_TWR(self, goal_point, dest_AD):
		acft_type = choice(self.parkable_aircraft_types if self.parkable_aircraft_types else self.playable_aircraft_types)
		sfc = rnd_dep_ldg_sfc(acft_type, dep=True)
		if sfc is None:
			raise AcftSpawnError()
		if sfc.isRunway():
			hdg = sfc.orientation() + 60
			pos = sfc.threshold(dthr=True).moved(hdg.opposite(), .04) # FUTURE use turn-offs backwards when ground net present
		else:
			hdg = sfc.param_preferred_DEP_course
			pos = sfc.centre.moved(hdg.opposite(), .04)
		params = FlightParams(pos, env.groundPressureAlt(pos), hdg, Speed(0), xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_DEP))
		return self.mkAiAcft(acft_type, params, Status.mkReadyForDep(sfc), (goal_point, None, dest_AD)), \
				'short of %s, ready for departure' % sfc.readOut(tts=True)

	def new_departure_DEP(self, goal_point, dest_AD):
		acft_type = choice(self.parkable_aircraft_types if self.parkable_aircraft_types else self.playable_aircraft_types)
		sfc = rnd_dep_ldg_sfc(acft_type, dep=True)
		if sfc is None:
			raise AcftSpawnError()
		if sfc.isRunway():
			hdg = sfc.orientation()
			tkofpt = sfc.threshold().moved(hdg, sfc.length() / 2) # midpoint
		else: # "sfc" is a helipad
			tkofpt = sfc.centre
			hdg = sfc.param_preferred_DEP_course
		dist_from_thr = max(3, settings.solo_TWR_range_dist - random())
		pos = tkofpt.moved(hdg, dist_from_thr)
		init_climb_spec = default_initial_climb_spec()
		init_climb_alt = env.pressureAlt(init_climb_spec)
		alt = GS_alt(env.elevation(tkofpt), initial_climb_angle, dist_from_thr) # pressure-alt. on upward flight path angle
		if alt.diff(init_climb_alt) > 0:
			alt = init_climb_alt
		ias = restrict_speed_under_ceiling(cruise_speed(acft_type).tas2ias(alt), alt, PressureAlt.fromFL(100))
		try: # Check for separation
			if time_to_fly(min(pos.distanceTo(acft.params.position) for acft in self.controlled_traffic if acft.isOutboundGoal()), ias) < TTF_separation:
				raise AcftSpawnError()
		except ValueError: # No departures in the sky yet
			pass
		params = FlightParams(pos, alt, hdg, ias, xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_DEP))
		acft = self.mkAiAcft(acft_type, params, Status(airborne=True), (goal_point, None, dest_AD))
		acft.ingestInstruction(Instruction(Instruction.VECTOR_ALT, arg=init_climb_spec)) # CAUTION: also on strip
		return acft, 'passing \\FL_ALT{%s} for \\FL_ALT{%s}' % (env.specifyAltFl(alt).toStr(), init_climb_spec.toStr())


	## GENERATING ARRIVALS

	def new_arrival_GND(self):
		acft_type, sfc, appgoal = rnd_arrival(self.parkable_aircraft_types)
		if sfc.isRunway():
			turn_off_lists = env.airport_data.ground_net.runwayTurnOffs(sfc, minRoll=(sfc.length(dthr=True) * .4))
			turn_offs = [rte for lst in turn_off_lists for rte in lst if self.groundPositionFullySeparated(env.airport_data.ground_net.nodePosition(rte[-1]), acft_type)]
			if not turn_offs:
				raise AcftSpawnError()
			turn_off_choice = choice(turn_offs[:turn_off_options_for_GND_arrival])
			pos = env.airport_data.ground_net.nodePosition(turn_off_choice[-1])
			hdg = env.airport_data.ground_net.nodePosition(turn_off_choice[-2]).headingTo(env.airport_data.ground_net.nodePosition(turn_off_choice[0][-1]))
		else: # LDG surface was a helipad
			n = env.airport_data.ground_net.closestNode(sfc.centre)
			assert n is not None, 'No closest ground net node found for helo spawn near %s' % sfc.readOut()
			hdg = sfc.centre.headingTo(env.airport_data.ground_net.nodePosition(n))
			pos = sfc.centre if n is None else sfc.centre.moved(hdg, .71 * sfc.width + .005)
		params = FlightParams(pos, env.groundPressureAlt(pos), hdg, Speed(0), xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_ARR))
		pk_request = choice(env.airport_data.ground_net.parkingPositions(acftType=acft_type))
		return self.mkAiAcft(acft_type, params, Status(airborne=False), pk_request), 'request parking at \\SPELL_ALPHANUMS{%s}' % pk_request

	def new_arrival_TWR(self):
		acft_type, sfc, appgoal = rnd_arrival(self.parkable_aircraft_types if self.parkable_aircraft_types else self.playable_aircraft_types)
		tdpt = sfc.touchDownPoint()
		if appgoal == ApproachType.STRAIGHT_IN: # ACFT necessarily a helo
			hdg = Heading(randint(1, 360), False)
			dist = uniform(.75, 1.5) * settings.solo_TWR_range_dist
			alt = PressureAlt.fromFL(settings.solo_TWR_ceiling_FL) + randint(0, 1000)
			while not self.airbornePositionFullySeparated(tdpt.moved(hdg.opposite(), dist), alt):
				alt += 500
		else: # LDG surface necessarily a runway, and ACFT on final leg
			hdg = sfc.appCourse()
			try:
				furthest = max(tdpt.distanceTo(acft.params.position) for acft in self.controlled_traffic
						if acft.isInboundGoal() and abs(acft.params.position.headingTo(tdpt).diff(hdg)) <= intercept_cone_half_angle)
				dist = max(furthest + uniform(1, 2) * distance_travelled(TTF_separation, cruise_speed(acft_type)), settings.solo_TWR_range_dist)
			except ValueError:
				dist = settings.solo_TWR_range_dist / 2
			alt = GS_alt(env.elevation(tdpt), sfc.param_FPA, max(2, dist if appgoal == ApproachType.ILS else dist - 2))
		if dist > min(settings.solo_TWR_range_dist * 1.5, settings.radar_range - 10):
			raise AcftSpawnError() # to protect from creating aircraft out of radar range
		params = FlightParams(tdpt.moved(hdg.opposite(), dist+3), alt, hdg, cruise_speed(acft_type).tas2ias(alt)/3, xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_ARR))
		acft = self.mkAiAcft(acft_type, params, Status(airborne=True), appgoal)
		#DEBUG print('Created TWR arrival.', appgoal, acft_type, sfc.name)
		acft.ingestInstruction(Instruction(Instruction.EXPECT_SFC, arg=sfc.name, arg2=appgoal))
		if appgoal != ApproachType.VISUAL:
			acft.ingestInstruction(Instruction(Instruction.CLEARED_APP))
		return acft, {
					ApproachType.ILS: 'established \\SPLIT_CHARS{ILS}',
					ApproachType.VISUAL: 'on visual approach for',
					ApproachType.STRAIGHT_IN: 'making approach straight in to'
				}[appgoal] + ' ' + sfc.readOut(tts=True)
	
	def new_arrival_APP(self, entry_point):
		acft_type, sfc, appgoal = rnd_arrival(self.parkable_aircraft_types if self.parkable_aircraft_types else self.playable_aircraft_types)
		if entry_point is None:
			hdg = Heading(randint(1, 360), True)
			pos = env.radarPos().moved(hdg.opposite(), uniform(.33 * settings.radar_range, .75 * settings.radar_range))
		else:
			pos = entry_point.coordinates
			hdg = pos.headingTo(env.radarPos())
		alt = PressureAlt.fromFL(10 * randint(settings.solo_APP_ceiling_FL_min // 10, settings.solo_APP_ceiling_FL_max // 10))
		if not self.airbornePositionFullySeparated(pos, alt):
			raise AcftSpawnError()
		ias = restrict_speed_under_ceiling(cruise_speed(acft_type).tas2ias(alt), alt, PressureAlt.fromFL(150)) # 5000-ft anticipation
		return self.mkAiAcft(acft_type, FlightParams(pos, alt, hdg, ias, xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_ARR)), Status(airborne=True), appgoal), \
				'\\FL_ALT{%s}, inbound for %s approach %s' % (env.specifyAltFl(alt).toStr(), ApproachType.tts(appgoal), sfc.readOut(tts=True))




# -----------------------------------------------------------------

class SoloSessionManager_CTR(SoloSessionManager):
	def __init__(self, gui, init_traffic_count):
		SoloSessionManager.__init__(self, gui)
		self.init_traffic_count = init_traffic_count
	
	def start(self): # overrides (but calls) parent's
		p = lambda d: env.radarPos().moved(Heading(d, True), 1.5 * settings.map_range)
		env.ATCs.updateATC('N', p(360), 'North', None)
		env.ATCs.updateATC('S', p(180), 'South', None)
		env.ATCs.updateATC('E', p(90), 'East', None)
		env.ATCs.updateATC('W', p(270), 'West', None)
		SoloSessionManager.start(self)

	def handoverGuard(self, acft, atc):
		if acft.coords().distanceTo(env.radarPos()) <= settings.solo_CTR_range_dist:
			return 'Aircraft is still in your airspace.'
		# Check if expected receiver
		dist_key_expected = lambda a: env.ATCs.getATC(a).position.distanceTo(acft.goal.coordinates)
		expected_receiver = min(env.ATCs.knownAtcCallsigns(), key=dist_key_expected)
		if atc != expected_receiver:
			return 'Destination is %s; hand over to %s.' % (acft.goal, expected_receiver)
		# Check if closest ATC
		dist_key_closest = lambda a: env.ATCs.getATC(a).position.distanceTo(acft.params.position)
		if atc != min(env.ATCs.knownAtcCallsigns(), key=dist_key_closest):
			return 'ACFT not near enough this neighbour\'s airspace.'

	
	def generateAircraftAndStrip(self):
		start_angle = uniform(0, 360)
		start_pos = env.radarPos().moved(Heading(start_angle, True), settings.solo_CTR_range_dist)
		end_pos = env.radarPos().moved(Heading(start_angle + 90 + uniform(1, 179), True), settings.solo_CTR_range_dist)
		transit_hdg = start_pos.headingTo(end_pos)
		dep_ad = world_navpoint_db.findClosest(env.radarPos().moved(transit_hdg.opposite(),
				uniform(1.2 * settings.map_range, 5000)), types=[Navpoint.AD])
		dest_ad = world_navpoint_db.findClosest(env.radarPos().moved(transit_hdg,
				uniform(1.2 * settings.map_range, 5000)), types=[Navpoint.AD])
		if env.pointOnMap(dep_ad.coordinates) or env.pointOnMap(dest_ad.coordinates):
			raise AcftSpawnError()
		
		candidate_midpoints = [p for code in settings.solo_CTR_routing_points
				for p in env.navpoints.findAll(code, types=[Navpoint.NDB, Navpoint.VOR, Navpoint.FIX])
				if start_pos.distanceTo(p.coordinates) < start_pos.distanceTo(end_pos)]
		midpoint = None if candidate_midpoints == [] else choice(candidate_midpoints)
		
		FLd10 = randint(settings.solo_CTR_floor_FL // 10, settings.solo_CTR_ceiling_FL // 10)
		if settings.solo_CTR_semi_circular_rule == SemiCircRule.E_W and (FLd10 % 2 == 0) != (transit_hdg.magneticAngle() >= 180) \
			or settings.solo_CTR_semi_circular_rule == SemiCircRule.N_S and (FLd10 % 2 == 1) != (90 <= transit_hdg.magneticAngle() < 270):
			FLd10 += 1
			if 10 * FLd10 > settings.solo_CTR_ceiling_FL:
				raise AcftSpawnError()
		p_alt = PressureAlt.fromFL(10 * FLd10)
		if not self.airbornePositionFullySeparated(start_pos, p_alt):
			raise AcftSpawnError()
		acft_type = choice(self.playable_aircraft_types)
		hdg = start_pos.headingTo(some(midpoint, dest_ad).coordinates)
		params = FlightParams(start_pos, p_alt, hdg, cruise_speed(acft_type).tas2ias(p_alt), xpdrCode=env.strips.nextSquawkCodeAssignment(XPDR_range_IFR_transit))
		new_acft = self.mkAiAcft(acft_type, params, Status(airborne=True), dest_ad)
		dist_key = lambda atc: env.ATCs.getATC(atc).position.distanceTo(start_pos)
		received_from = min(env.ATCs.knownAtcCallsigns(), key=dist_key)
		
		strip = Strip()
		strip.writeDetail(FPL.CALLSIGN, new_acft.identifier)
		strip.writeDetail(FPL.ACFT_TYPE, new_acft.aircraft_type)
		strip.writeDetail(FPL.WTC, wake_turb_cat(new_acft.aircraft_type))
		strip.writeDetail(FPL.FLIGHT_RULES, 'IFR')
		strip.writeDetail(FPL.ICAO_DEP, dep_ad.code)
		strip.writeDetail(FPL.ICAO_ARR, dest_ad.code)
		alt_spec = env.specifyAltFl(new_acft.params.altitude)
		strip.writeDetail(FPL.CRUISE_ALT, alt_spec)
		strip.writeDetail(assigned_altitude_detail, alt_spec)
		strip.writeDetail(assigned_SQ_detail, new_acft.params.XPDR_code)
		strip.writeDetail(received_from_detail, received_from)
		if midpoint is None:
			rtestr = ''
		else:
			strip.insertRouteWaypoint(midpoint)
			rtestr = strip.lookup(FPL.ROUTE).enRouteStr()

		new_acft.ingestInstruction(Instruction(Instruction.FOLLOW_ROUTE, arg=rtestr))
		return new_acft, strip, '\\FL_ALT{%s}' % env.specifyAltFl(new_acft.params.altitude).toStr()
