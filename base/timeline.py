
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
from datetime import datetime, timezone, timedelta
from sys import stderr

from base.acft import Xpdr, RadarSnapshot
from base.coords import EarthCoords
from base.cpdlc import CpdlcMessage
from base.params import Heading
from base.phone import PhoneLineStatus
from base.radio import CommFrequency, RdfSignalData
from base.text import TextMessage
from base.weather import Weather

from session.config import settings, version_string
from session.env import env
from gui.misc import signals


# ---------- Constants ----------

unknown_radio_msg_sender_str = '?'
missing_radio_msg_audio_str = '!! no audio'
missing_radio_msg_transcript_str = '!! no transcript'

header_kwd_start_time = 'utc_start'
header_kwd_loc_code = 'location_code'
header_kwd_loc_coords = 'location_coords'
header_kwd_recording_version = 'ATC_pie_version'

# -------------------------------


class TimelineEvent:
    events = ACFT_BLIP, RADIO_MSG, RADIO_SIGNAL, CPDLC_EVENT, ATC_MSG, PHONE_LINE_STATUS, WEATHER_INFO, NEW_ATIS, GENERIC = range(9)
    str_keys = {
        ACFT_BLIP: 'B', RADIO_MSG: 'R', RADIO_SIGNAL: 'D', CPDLC_EVENT: 'C',
        ATC_MSG: 'M', PHONE_LINE_STATUS: 'P', WEATHER_INFO: 'W', NEW_ATIS: 'A',
        GENERIC: '_'
    }
    assert len(str_keys) == len(events) == len(set(str_keys.values()))


phone_line_status_str = {
    PhoneLineStatus.IDLE: '-',
    PhoneLineStatus.CALLING: 'C',
    PhoneLineStatus.RINGING: 'R',
    PhoneLineStatus.HELD_INCOMING: 'I',
    PhoneLineStatus.HELD_OUTGOING: 'O',
    PhoneLineStatus.IN_CALL: '+'
}



class Timeline:
    def __init__(self, start_ref):
        self.start_time = start_ref
        self.timeline_data = [] # (datetime, TimelineEvent int, output tuple) list, ordered chronologically
        self.timeline_index = 0

    def startTime(self):
        return self.start_time

    def firstEventTime(self):
        return self.timeline_data[0][0]

    def lastEventTime(self):
        return self.timeline_data[-1][0]

    def duration(self):
        return self.lastEventTime() - self.startTime()

    def timeAfterStart(self, time):
        return time - self.startTime()

    ## READING/SEEKING
    def endReached(self):
        return self.timeline_index >= len(self.timeline_data)

    def nextEventTime(self):
        return self.timeline_data[self.timeline_index][0]

    def nextEventType(self):
        return self.timeline_data[self.timeline_index][1]

    def readEvent(self):
        restuple = self.timeline_data[self.timeline_index]
        self.timeline_index += 1
        return restuple

    def readEventData(self):
        restuple = self.timeline_data[self.timeline_index][2]
        self.timeline_index += 1
        return restuple

    def skipEvent(self):
        self.timeline_index += 1

    def reset(self):
        self.timeline_index = 0

    def resetAfterTime(self, time):
        self.timeline_index = next((i for i, s in enumerate(self.timeline_data) if s[0] > time), len(self.timeline_data))

    ## BUILDING
    def addEvent(self, time, etyp, data_tuple):
        if not self.timeline_data or self.timeline_data[-1][0] <= time:
            self.timeline_data.append((time, etyp, data_tuple))
        else:
            #print('WARNING: Time %s is before previous (event count before inserting: %i).' % (time, len(self.timeline_data)), file=stderr)
            self.timeline_data.insert(next(i for i, (t, _, _) in enumerate(self.timeline_data) if time <= t), (time, etyp, data_tuple))

    def finishBuilding(self):
        if not self.timeline_data:
            raise ValueError('empty timeline')
        for i in range(len(self.timeline_data) - 1):
            if self.timeline_data[i][0] > self.timeline_data[i+1][0]:
                assert False, 'Event %i (time %s) is not after previous' % (i+1, self.timeline_data[i+1][0])



class SessionRecorder:
    def __init__(self):
        self.output_file_name = None
        self.output_file_handle = None
        self.start_time = None
        self.record_traffic = True
        self.record_comms = True
        self.record_weather = True
        self.record_other = True

    def recordedEventsFlags(self):
        return self.record_traffic, self.record_comms, self.record_weather, self.record_other

    def setRecordedEvents(self, traffic, comms, weather, other):
        self.record_traffic = traffic
        self.record_comms = comms
        self.record_weather = weather
        self.record_other = other

    def isRecording(self):
        return self.output_file_handle is not None

    def startRecording(self, output_file_name):
        if self.isRecording():
            print('Internal error: already recording to file %s' % self.output_file_name, file=stderr)
            return
        try:
            self.output_file_handle = open(output_file_name, 'w', encoding='utf8')
            self.output_file_name = output_file_name
        except OSError as err:
            print('System error opening file for recording: %s' % err, file=stderr)
        else:
            self.start_time = settings.session_manager.clockTime()
            self._outputHeaderLine(header_kwd_loc_code, settings.location_code)
            self._outputHeaderLine(header_kwd_loc_coords, env.radarPos().toString())
            self._outputHeaderLine(header_kwd_recording_version, version_string)
            self._outputHeaderLine(header_kwd_start_time, '%d%02d%02d-%d%02d%02d' % (self.start_time.year, self.start_time.month,
                    self.start_time.day, self.start_time.hour, self.start_time.minute, self.start_time.second))
            ## BEGIN: record necessary environment
            # Open CPDLC dialogues
            for link in env.cpdlc.dataLinks(pred=lambda dm: not dm.isTerminated()):
                xfr_from = link.pendingTransferFrom()
                if xfr_from is None: # live link
                    settings.session_recorder.proposeCpdlcSys(self.start_time, link.acftCallsign(), connectFlag=True)
                if xfr_from is not None or link.pendingTransferTo() is not None:
                    settings.session_recorder.proposeCpdlcSys(self.start_time, link.acftCallsign(), xfr=xfr_from)
            # Phone line statuses
            llm = settings.session_manager.phoneLineManager()
            if llm is not None:
                for atc in env.knownAcftCallsigns():
                    lls = llm.lineStatus(atc)
                    if lls is not None and lls != PhoneLineStatus.IDLE:
                        self.proposeAtcPhoneStatusChange(self.start_time, atc, lls)
            # Current weather information
            for w in env.weather_information.values():
                self.proposeWeatherChange(self.start_time, w)
            ## END: record necessary environment
            signals.sessionRecorderStarted.emit()

    def stopIfRecording(self):
        if self.isRecording():
            to_close = self.output_file_handle
            self.output_file_handle = None # stops record* methods from writing
            try:
                to_close.close()
            except OSError as err:
                print('System error closing record file: %s' % err, file=stderr)
            self.output_file_name = None
            self.start_time = None
            signals.sessionRecorderStopped.emit()

    def _outputHeaderLine(self, kwd, datastr):
        print('%s=%s' % (kwd, datastr), file=self.output_file_handle)

    def _outputEventLine(self, time, event_type, data_fields):
        header_fields = [str((time - self.start_time).total_seconds()), TimelineEvent.str_keys[event_type]]
        print('\t'.join(header_fields + data_fields), file=self.output_file_handle)

    def proposeGenericEvent(self, t, text):
        if self.output_file_handle is not None and self.record_other:
            self._outputEventLine(t, TimelineEvent.GENERIC, [text])

    def proposeAcftBlip(self, identifier, snap):
        if self.output_file_handle is not None and self.record_traffic:
            fields = [identifier, snap.coords.toString()]
            fields.extend(Xpdr.encodeData(key, snap.xpdrData.get(key, None)) for key in Xpdr.keys)
            self._outputEventLine(snap.time_stamp, TimelineEvent.ACFT_BLIP, fields)

    def proposeTextRadioMsg(self, msg):
        if self.output_file_handle is not None and self.record_comms:
            self._outputEventLine(msg.timeStamp(), TimelineEvent.RADIO_MSG, ['-', msg.sender(), msg.txtMsg()])

    def proposeVoiceRadioMsg(self, t, audio_data, freqs=None): # FUTURE
        if self.output_file_handle is not None and self.record_comms:
            frqprefix = ':' if freqs is None else ','.join(str(frq) for frq in freqs) + ':'
            if audio_data is None:
                self._outputEventLine(t, TimelineEvent.RADIO_MSG, [frqprefix])
            else: # audio output provided
                try:
                    #TODO save audio output to file and use filename after prefix below
                    self._outputEventLine(t, TimelineEvent.RADIO_MSG, [frqprefix + 'filename'])
                except IOError:
                    self._outputEventLine(t, TimelineEvent.RADIO_MSG, [frqprefix, '!!error', 'audio output could not be saved'])

    def proposeRdfSignalUpdate(self, t, sigdata):
        if self.output_file_handle is not None and self.record_comms:
            self._outputEventLine(t, TimelineEvent.RADIO_SIGNAL,
                    [('-' if sigdata.frequency is None else str(sigdata.frequency)), str(sigdata.direction.magneticAngle()), str(sigdata.quality)])

    def proposeRdfSignalEnd(self, t, freq_or_none):
        if self.output_file_handle is not None and self.record_comms:
            self._outputEventLine(t, TimelineEvent.RADIO_SIGNAL, [('-' if freq_or_none is None else str(freq_or_none)), '-'])

    def proposeCpdlcSys(self, t, acft_callsign, connectFlag=None, xfr=None):
        """
        connectFlag without xfr: True = logon accepted; False = ATC disconnect; None = ACFT disconnect
        connectFlag with xfr (current/next ATC callsign): True = XFR accepted; False = XFR rejected; None = XFR proposed or cancelled
        """
        if self.output_file_handle is not None and self.record_comms:
            data = [acft_callsign, '!' if connectFlag is None else '-+'[connectFlag]]
            if xfr is not None:
                data.append(xfr)
            self._outputEventLine(t, TimelineEvent.CPDLC_EVENT, data)

    def proposeCpdlcMsg(self, acft_callsign, msg):
        if self.output_file_handle is not None and self.record_comms:
            self._outputEventLine(msg.timeStamp(), TimelineEvent.CPDLC_EVENT, [acft_callsign, msg.toEncodedStr()])

    def proposeAtcTextMsg(self, msg):
        if self.output_file_handle is not None and self.record_comms:
            if msg.isPrivate():
                data_fields = [msg.recipient() if msg.isFromMe() else msg.sender(), '-+'[msg.isFromMe()], msg.txtMsg()]
            else: # public message
                data_fields = [msg.sender(), '!', msg.txtMsg()]
            self._outputEventLine(msg.timeStamp(), TimelineEvent.ATC_MSG, data_fields)

    def proposeAtcPhoneStatusChange(self, t, atc_callsign, status):
        if self.output_file_handle is not None and self.record_comms:
            self._outputEventLine(t, TimelineEvent.PHONE_LINE_STATUS, [atc_callsign, phone_line_status_str[status]])

    def proposeWeatherChange(self, t, w):
        if self.output_file_handle is not None and self.record_weather:
            self._outputEventLine(t, TimelineEvent.WEATHER_INFO, [w.METAR()])

    def proposeNewAtis(self, t, letter, freq, txt):
        if self.output_file_handle is not None and self.record_comms:
            self._outputEventLine(t, TimelineEvent.NEW_ATIS, [letter, str(freq), txt.replace('\n', '  ')])



###  READ TIMELINE EVENT DATA  ###

# PHONE_LINE_STATUS event line output fields: ATC callsign, new status character (see phone_line_status_str)
# WEATHER_INFO event line output: METAR string
# NEW_ATIS event line output fields: letter char, frequency, notepad text
# GENERIC event line output: text description
# ACFT_BLIP, CPDLC_EVENT, RADIO_MSG, ATC_MSG, RADIO_SIGNAL event line output fields: see dedicated read functions below

header_line_regexp = re.compile(r'(\w+)=')
start_time_datum_regexp = re.compile(r'(\d+)(\d{2})(\d{2})-(\d{1,2})(\d{2})(\d{2})')

def read_timeline_data(data_file):
    """
    may raise: FileNotFoundError, ValueError
    """
    with open(data_file, encoding='utf8') as f:
        start_time = None
        meta_data = {}
        line = f.readline()
        while line != '':
            uncommented_line = line.split('#', maxsplit=1)[0].rstrip()
            if uncommented_line:
                match_header_line = header_line_regexp.match(uncommented_line)
                if match_header_line:
                    header_kwd = match_header_line.group(1)
                    header_str = uncommented_line[len(match_header_line.group(0)):]
                    if header_kwd == header_kwd_start_time:
                        match_utc_start = start_time_datum_regexp.fullmatch(header_str)
                        if match_utc_start:
                            start_time = datetime(*(int(match_utc_start.group(i)) for i in range(1, 7)), 0, timezone.utc)
                    elif header_kwd == header_kwd_loc_coords:
                        meta_data[header_kwd] = EarthCoords.fromString(header_str)
                    else:
                        meta_data[header_kwd] = header_str
                else: # first non-header content line found
                    break
            line = f.readline()
        if start_time is None:
            raise ValueError('missing "%s" header line or invalid date/time string' % header_kwd_start_time)
        result = Timeline(start_time)
        for line in f:
            spl = line.rstrip('\n').split('\t') # no comment allowed on an event line
            try:
                if len(spl) <= 1:
                    raise IndexError()
                time = start_time + timedelta(seconds=float(spl[0])) # may raise ValueError
                etyp = next(k for k, v in TimelineEvent.str_keys.items() if spl[1] == v) # may raise StopIteration
                data_fields = spl[2:]
                # read_* functions below may raise ValueError
                if etyp == TimelineEvent.ACFT_BLIP:
                    data = read_timeline_acft_blip(time, data_fields)
                elif etyp == TimelineEvent.RADIO_MSG:
                    data = read_timeline_radio_msg(time, data_fields)
                elif etyp == TimelineEvent.RADIO_SIGNAL:
                    data = read_timeline_radio_signal(data_fields)
                elif etyp == TimelineEvent.CPDLC_EVENT:
                    data = read_timeline_cpdlc_event(data_fields)
                elif etyp == TimelineEvent.ATC_MSG:
                    data = read_timeline_atc_msg(time, data_fields)
                elif etyp == TimelineEvent.PHONE_LINE_STATUS:
                    try:
                        data = data_fields[0], next(status for status, s in phone_line_status_str.items() if s == data_fields[1])
                    except StopIteration:
                        raise ValueError('invalid phone status string: ' + data_fields[1])
                elif etyp == TimelineEvent.WEATHER_INFO:
                    data = Weather(data_fields[0]) # may raise IndexError
                elif etyp == TimelineEvent.NEW_ATIS:
                    data = data_fields[0], CommFrequency(data_fields[1]), data_fields[2]  # may raise IndexError, ValueError
                elif etyp == TimelineEvent.GENERIC:
                    data = data_fields[0] # may raise IndexError
                result.addEvent(time, etyp, data)
            except StopIteration:
                print('Unrecognised timeline event type "%s" on line starting with "%s"' % (spl[1], spl[0]), file=stderr)
            except (ValueError, IndexError):
                print('Timeline output error on line starting with "%s"' % spl[0], file=stderr)
    result.finishBuilding() # may raise ValueError
    print('Recorded output sourced successfully from %s' % data_file)
    return result, meta_data



# ACFT_BLIP event line
# output fields: ID, coords, XPDR values in Xpdr.keys order
# returns pair: unique ACFT/flight ID, RadarSnapshot to add to its history
def read_timeline_acft_blip(time, data):
    if len(data) < 2:
        raise ValueError('insufficient output on ACFT radar snapshot line')
    tk_id = data[0]
    tk_coords = EarthCoords.fromString(data[1])
    xpdr_values = [Xpdr.decodeData(Xpdr.keys[i], tk) for i, tk in enumerate(data[2:len(Xpdr.keys)])]
    return tk_id, RadarSnapshot(time, tk_coords, {k: val for k, val in enumerate(xpdr_values) if val is not None})

# CPDLC_EVENT event line
# output fields: either format below [--> return values]
#   - logon accepted:         ACFT callsign, "+"                  -->  ACFT callsign, True
#   - ATC disconnect:         ACFT callsign, "-"                  -->  ACFT callsign, False
#   - ACFT disconnect:        ACFT callsign, "!"                  -->  ACFT callsign, None
#   - XFR proposed/cancelled: ACFT callsign, "!", ATC callsign    -->  ACFT callsign, str ATC callsign
#   - XFR accepted:           ACFT callsign, "+", ATC callsign    -->  ACFT callsign, (str ATC callsign, True) pair
#   - XFR rejected:           ACFT callsign, "-", ATC callsign    -->  ACFT callsign, (str ATC callsign, False) pair
#   - in-dialogue message:    ACFT callsign, str-encoded message  -->  ACFT callsign, CpdlcMessage
def read_timeline_cpdlc_event(data):
    if 2 <= len(data) <= 3:
        if data[1] in {'+', '-', '!'}: # CPDLC connection/transfer system
            if len(data) == 2: # logon/disconnect
                return data[0], {'+': True, '-': False, '!': None}[data[1]]
            else: # output authority transfer
                return data[0], data[2] if data[1] == '!' else (data[2], data[1] == '+')
        elif len(data) == 2: # regular message
            return data[0], CpdlcMessage.fromEncodedStr(data[1])
    raise ValueError('invalid output on CPDLC event line')

# RADIO_MSG event lines
# output fields: either format below
#   - text-only msg: "-", sender, text message
#   - voice msg: [frq/chan list]":"[audio file name], [sender, text message if transcript added]
# return value (tuple): TextMessage with "T" or "V" display prefix, audio file or None, list of declared frequencies
def read_timeline_radio_msg(time, data):
    if len(data) == 3 and data[0] == '-': # text-only message
        msg = TextMessage(data[1], data[2], timeStamp=time)
        msg.setDispPrefix('T')
        return msg, None, []
    elif (len(data) == 1 or len(data) == 3) and ':' in data[0]: # voice message
        tkfrq, tkfile = data[0].split(':', maxsplit=1)
        frqlst = [] if tkfrq == '' else [CommFrequency(frq) for frq in tkfrq.split(',')] # may raise ValueError
        faudio = tkfile if tkfile else None
        if len(data) == 3: # transcript provided
            msg = TextMessage(data[1], data[2], timeStamp=time)
        else: # len(output) == 1; no transcript
            txt = missing_radio_msg_audio_str if faudio is None else missing_radio_msg_transcript_str + ' ' + faudio
            msg = TextMessage(unknown_radio_msg_sender_str, txt, timeStamp=time)
        msg.setDispPrefix('V')
        return msg, faudio, frqlst
    raise ValueError('invalid output on radio message line')

# ATC_MSG event lines
# output fields: either format below
#   - public channel:  sender callsign, "!", text message
#   - private msg received: sender callsign, "-", text message
#   - private msg sent: recipient callsign, "+", text message
# returns a TextMessage
def read_timeline_atc_msg(time, data):
    if len(data) == 3: # audio file only
        if data[1] == '!': # public channel
            return TextMessage(data[0], data[2], private=False, timeStamp=time)
        elif data[1] == '-': # private msg received
            return TextMessage(data[0], data[2], recipient=settings.my_callsign, private=True, timeStamp=time)
        elif data[1] == '+': # private msg sent
            return TextMessage(settings.my_callsign, data[2], recipient=data[0], private=True, timeStamp=time)
    raise ValueError('invalid output on ATC message line')

# RADIO_SIGNAL event lines
# output fields: either format below [--> return values]
#   - signal update: frequency or "-", heading from, quality  -->  True, RdfSignalData
#   - signal dying: frequency or "-", "-"                     -->  False, CommFrequency or None
def read_timeline_radio_signal(data):
    if len(data) == 3: # signal update
        return True, RdfSignalData(None if data[0] == '-' else CommFrequency(data[0]), Heading(float(data[1]), False), float(data[2]))
    elif len(data) == 2 and data[1] == '-': # signal dies
        return False, None if data[0] == '-' else CommFrequency(data[0])
    else:
        raise ValueError('invalid output on radio signal line')
