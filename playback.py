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
from sys import stderr

from base.acft import Aircraft, Xpdr
from base.cpdlc import CpdlcMessage
from base.phone import AbstractPhoneLineManager
from base.radio import CommFrequency
from base.text import TextMessage
from base.timeline import TimelineEvent
from base.utc import VirtualClock
from base.util import some, noNone, linear

from ext.fgfs import send_packet_to_views, FGFS_model_position
from ext.fgms import mk_fgms_position_packet

from gui.actions import register_weather_information
from gui.misc import signals
from gui.widgets.basicWidgets import Ticker

from session.config import settings
from session.env import env
from session.manager import SessionManager, SessionType, missing_client_type_str, \
    TextMsgBlocked, OnlineFplActionBlocked, HandoverBlocked, CpdlcOperationBlocked


# ---------- Constants ----------

live_ticker_interval_ms = 100
ACFT_TTL_without_snapshot = timedelta(seconds=8)

# -------------------------------

class PlaybackAircraft(Aircraft):
    def __init__(self, callsign, snapshot):
        Aircraft.__init__(self, callsign, snapshot.xpdrData.get(Xpdr.ACFT, missing_client_type_str), snapshot.coords, -999)
        self.radar_history = [snapshot] # hack to replace history with our own snapshot (and its own time stamp)
        ## Disabling Live status responsive updated by radar output
        self.live_update_time = NotImplemented  # no live status for playback session aircraft

    def isRadarVisible(self):
        return True

    def fgmsPositionPacket(self):
        """
        Raises ValueError if not able to depict at least some sensible ACFT position, altitude and heading.
        """
        heading = noNone(self.heading())
        model, coords, amsl = FGFS_model_position(self.aircraft_type, self.liveCoords(), self.liveRealAlt(), heading)
        return mk_fgms_position_packet(self.identifier, model, coords, amsl, hdg=heading.trueAngle(),
                pitch=linear(0, 0, 1000, 10, some(self.verticalSpeed(), 0)))

    ## Disable/replace live status operations
    def updateLiveStatus(self, pos, real_alt, xpdr_data):
        assert False, 'No live status for playback session aircraft.'

    def saveRadarSnapshot(self): # Ignore: future history already known
        return

    def liveCoords(self):
        t = settings.session_manager.clockTime()
        i_next = next((i for i, snap in enumerate(self.radar_history) if snap.time_stamp > t), len(self.radar_history))
        if 0 < i_next < len(self.radar_history):
            snap_prev = self.radar_history[i_next - 1]
            snap_next = self.radar_history[i_next]
            return snap_prev.coords.moved(snap_prev.coords.headingTo(snap_next.coords),
                    snap_prev.coords.distanceTo(snap_next.coords) * (t - snap_prev.time_stamp) / (snap_next.time_stamp - snap_prev.time_stamp))
        else:
            return self.radar_history[0 if i_next <= 0 else -1].coords

    def liveRealAlt(self):
        t = settings.session_manager.clockTime()
        i_next = next((i for i, snap in enumerate(self.radar_history) if snap.time_stamp > t), len(self.radar_history))
        if 0 < i_next < len(self.radar_history):
            snap_prev = self.radar_history[i_next - 1]
            snap_next = self.radar_history[i_next]
            alt_prev = snap_prev.xpdrData.get(Xpdr.ALT)
            alt_next = snap_next.xpdrData.get(Xpdr.ALT)
            if alt_prev is not None and alt_next is not None:
                return alt_prev.ft1013() + alt_next.diff(alt_prev) * (t - snap_prev.time_stamp) / (snap_next.time_stamp - snap_prev.time_stamp)
            else:
                raise ValueError('Missing altitude output around time %s' % t)
        else:
            return noNone(self.radar_history[0 if i_next <= 0 else -1].xpdrData.get(Xpdr.ALT)).ft1013()



class VoicelessPhoneLineManager(AbstractPhoneLineManager):
    def _startVoiceWith(self, atc):
        pass
    def _stopVoice(self):
        pass
    def _sendRequest(self, atc):
        pass
    def _sendDrop(self, atc):
        pass



def play_CPDLC_event(acft_callsign, spec):
    link = env.cpdlc.liveDataLink(acft_callsign)
    if spec is True and link is None: # logon accepted
        env.cpdlc.beginDataLink(acft_callsign)
    elif spec is False and link is not None: # ATC disconnect
        link.terminate(True)
    elif spec is None and link is not None: # ACFT disconnect
        link.terminate(False)
    elif isinstance(spec, str): # output authority transfer proposed/cancelled
        if link is None: # incoming proposal
            env.cpdlc.beginDataLink(acft_callsign, transferFrom=spec)
        elif link.pendingTransferFrom() is not None: # incoming cancellation
            link.terminate(True)
        elif link.pendingTransferTo() is not None: # we cancel
            link.appendMessage(CpdlcMessage('SYSU-2'))
            link.setTransferTo(None)
        else: # we propose
            link.appendMessage(CpdlcMessage('SYSU-2 ' + spec))
            link.setTransferTo(spec)
    elif isinstance(spec, tuple) and link is not None: # output authority transfer accepted/rejected
        atc_callsign, accept_flag = spec
        if link.pendingTransferFrom() is not None:
            if accept_flag: # we accept
                link.acceptIncomingTransfer()
            else: # we reject
                link.terminate(False)
        elif link.pendingTransferTo() is not None:
            if accept_flag: # they accept
                link.terminate(True)
            else: # they reject
                link.appendMessage(CpdlcMessage('SYSU-2'))
                link.setTransferTo(None)
        else: # error in pending XFR status
            print('CPDLC playback error: XFR accept/reject without pending proposal from/to %s' % atc_callsign, file=stderr)
    elif isinstance(spec, CpdlcMessage) and link is not None: # regular message
        link.appendMessage(spec)
    else: # error in link None status
        print('CPDLC playback error: message or system op not matching live link status for callsign %s' % acft_callsign, file=stderr)



class PlaybackSessionManager(SessionManager):
    def __init__(self, gui, timeline):
        SessionManager.__init__(self, gui, SessionType.PLAYBACK)
        self.timeline = timeline # Timeline (internal index during session points to the first event after playback time)
        # build ACFT output and radar histories
        self.acft_data = {} # str callsign -> PlaybackAircraft list
        self.phone_line_manager = VoicelessPhoneLineManager()
        # Scan timeline first to build useful persistent histories (messages, radar...)
        while not self.timeline.endReached():
            t, e, data = self.timeline.readEvent() # moves internal index forward on the timeline
            if e == TimelineEvent.ACFT_BLIP:
                callsign, snapshot = data
                try:
                    self.acft_data[callsign].appendToRadarHistory(snapshot)
                except KeyError: # create non-existing ACFT
                    self.acft_data[callsign] = PlaybackAircraft(callsign, snapshot)
            elif e == TimelineEvent.ATC_MSG:
                signals.incomingAtcTextMsg.emit(data)
            elif e == TimelineEvent.CPDLC_EVENT:
                play_CPDLC_event(*data)
            elif e == TimelineEvent.RADIO_MSG:
                signals.incomingTextRadioMsg.emit(data[0]) # contains the T/V display prefix
            elif e == TimelineEvent.NEW_ATIS:
                signals.incomingTextRadioMsg.emit(TextMessage('', 'ATIS "%s" recorded on %s.' % (data[0], data[1]), timeStamp=t)) # using text msg history as a timeline log
            elif e == TimelineEvent.GENERIC:
                signals.incomingTextRadioMsg.emit(TextMessage('', data, timeStamp=t)) # using text msg history as a timeline log
        self.timeline.reset()
        # simulation time and traffic
        self.live_clock = VirtualClock(startPausedAt=self.timeline.startTime()) # used when ticking for live playback
        self.live_clock_ticker = Ticker(self.gui, self._clockTick)
        self.session_started = False
        self.playback_time = self.timeline.firstEventTime()

    def start(self):
        self.playback_time = self.timeline.firstEventTime() # NOTE: usually different from startTime()
        self.session_started = True
        self._playTimelineUntil(self.timeline.startTime()) # ingest anything happening before start time
        print('Playback session ready.')
        signals.sessionStarted.emit(SessionType.PLAYBACK)

    def stop(self):
        self.live_clock_ticker.stop()
        signals.sessionEnded.emit(SessionType.PLAYBACK)
        self.session_started = False

    def pause(self):
        if self.isRunning() and not self.live_clock.isPaused():
            self.live_clock_ticker.stop()
            self.live_clock.pause()
            self.playback_time = self.live_clock.readTime()
            signals.playbackClockChanged.emit(self.playback_time)
            signals.sessionPaused.emit()

    def resume(self):
        if self.live_clock.isPaused():
            #FIXME useful?self.live_clock.setTime(self.playback_time)
            self.live_clock.resume()
            self.live_clock_ticker.startTicking(live_ticker_interval_ms, immediate=False)
            signals.sessionResumed.emit()

    def isRunning(self):
        return self.session_started

    def clockTime(self):
        return self.playback_time

    def getAircraft(self):
        return [acft for acft in self.acft_data.values()
                if acft.radar_history[0].time_stamp <= self.playback_time <= acft.radar_history[-1].time_stamp + ACFT_TTL_without_snapshot]

    # ACFT/ATC interaction
    def instructAircraftByCallsign(self, callsign, instr):
        pass

    def postTextRadioMsg(self, msg):
        raise TextMsgBlocked('Text radio panel in playback sessions is reserved for monitoring recorded messages and events.')

    def postAtcChatMsg(self, msg):
        raise TextMsgBlocked('Feature disabled in playback sessions.')

    def sendStrip(self, strip, atc):
        raise HandoverBlocked('Feature disabled in playback sessions.')

    def sendWhoHas(self, callsign):
        pass

    def sendCpdlcMsg(self, callsign, msg):
        raise CpdlcOperationBlocked('Feature disabled in playback sessions.')

    def sendCpdlcTransferRequest(self, acft_callsign, atc_callsign, proposing):
        raise CpdlcOperationBlocked('Feature disabled in playback sessions.')

    def sendCpdlcTransferResponse(self, acft_callsign, atc_callsign, accept):
        raise CpdlcOperationBlocked('Feature disabled in playback sessions.')

    def sendCpdlcDisconnect(self, acft_callsign):
        raise CpdlcOperationBlocked('Feature disabled in playback sessions.')

    # Voice communications
    def createRadio(self):
        return None # radio panel disabled anyway

    def recordAtis(self, parent_dialog):
        pass

    def phoneLineManager(self):
        return self.phone_line_manager

    # Online systems
    def weatherLookUpRequest(self, station):
        pass

    def pushFplOnline(self, fpl):
        raise OnlineFplActionBlocked('Feature disabled in playback sessions.')

    def changeFplStatus(self, fpl, new_status):
        raise OnlineFplActionBlocked('Feature disabled in playback sessions.')

    def syncOnlineFPLs(self):
        raise OnlineFplActionBlocked('Feature disabled in playback sessions.')


    ## MANAGER-SPECIFIC

    # Clock & time management
    def skipTimeForward(self, offset):
        self.offsetSessionTime(offset)

    def setTimeSpeedFactor(self, value):
        self.live_clock.setTimeFactor(value)

    def _clockTick(self):
        self._playTimelineUntil(self.live_clock.readTime())
        if self.timeline.endReached(): # end of timeline reached
            self.pause()

    def _playTimelineUntil(self, until_time): # inclusive
        while not self.timeline.endReached() and self.timeline.nextEventTime() <= until_time:
            self.playback_time, etyp, data = self.timeline.readEvent() # moves forward on the timeline
            if etyp == TimelineEvent.ACFT_BLIP:
                pass # ACFT history lookup by radar takes care of that
            elif etyp == TimelineEvent.RADIO_MSG:
                # NOTE: text radio message history already filled for whole session
                if data[1]: # audio file given
                    pass #TODO play if running time normally?
            elif etyp == TimelineEvent.RADIO_SIGNAL: # signalling on generic "None" freq because no radio boxes in playback sessions
                if data[0]: # new signal or update
                    env.rdf.radioSignal(data[1].frequency, data[1].direction, quality=data[1].quality)
                else: # signal dying
                    env.rdf.endOfSignal(None)
            elif etyp == TimelineEvent.CPDLC_EVENT:
                pass # CPDLC dialogues already filled for whole session
            elif etyp == TimelineEvent.ATC_MSG:
                pass # ATC message history already filled for whole session
            elif etyp == TimelineEvent.PHONE_LINE_STATUS:
                self.phone_line_manager.setLineStatus(data[0], data[1])
            elif etyp == TimelineEvent.WEATHER_INFO:
                register_weather_information(data)
            elif etyp == TimelineEvent.NEW_ATIS:
                settings.last_recorded_ATIS = data[0], self.playback_time, data[1], data[2]
            elif etyp == TimelineEvent.GENERIC:
                pass # text radio message history already filled for whole session
            else:
                print('Unhandled timeline event type %i' % etyp, file=stderr)
        if not self.timeline.endReached():
            self.playback_time = until_time
        if self.live_clock.isPaused():
            self.live_clock.setTime(until_time)
        signals.playbackClockChanged.emit(until_time)
        env.radar.instantSweep()
        for acft in self.getAircraft():
            try:
                send_packet_to_views(acft.fgmsPositionPacket())
            except ValueError:
                pass

    def setSessionTime(self, new_time):
        if new_time == self.playback_time:
            return
        if new_time < self.playback_time: # time wanted is prior to current; reset timeline and play back
            env.radar.resetContacts()
            env.rdf.resetSignals()
            env.weather_information.clear()
            settings.last_recorded_ATIS = None
            self.playback_time = self.timeline.firstEventTime()
            self.timeline.reset()
        self._playTimelineUntil(new_time)

    def offsetSessionTime(self, time_offset):
        self.setSessionTime(self.playback_time + time_offset)
