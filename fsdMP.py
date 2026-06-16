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
from socket import socket, AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR

from PyQt5.QtCore import QMutex
from PyQt5.QtWidgets import QMessageBox

from base.acft import Xpdr
from base.coords import EarthCoords
from base.cpdlc import CPDLC_element_display_text
from base.fpl import FPL
from base.params import PressureAlt, Speed
from base.phone import AbstractVoipPhoneManager
from base.radio import CommFrequency
from base.strip import Strip, handover_details, received_from_detail
from base.text import TextMessage
from base.utc import realTime
from base.util import pop_all, INET_addr_str, INET_addr_from_str
from base.weather import Weather

from ext.audio import pyaudio_available
from ext.fgcom import FGComRadio, FGCom_tick_interval, send_FGCom_mumble_control_packet, receive_FGCom_mumble_packet, record_FGCom_Mumble_ATIS
from ext.fgfs import send_packet_to_views
from ext.fsd import FsdConnection, FsdAircraft, FPL_from_fields
from ext.hoppie import HoppieCommunicator
from ext.noaa import RealWeatherChecker

from gui.actions import register_weather_information
from gui.misc import signals
from gui.widgets.basicWidgets import Ticker

from session.config import settings, version_string
from session.env import env
from session.manager import SessionManager, SessionType, TextMsgBlocked, missing_client_type_str, \
    HandoverBlocked, CpdlcOperationBlocked, OnlineFplActionBlocked
from session.managers.flightGearMP import UdpSessionListener


# ---------- Constants ----------

position_update_interval = 5000 # ms
ATIS_ticker_interval = 15000 # ms
TM_escape_prefix = '___ATC-pie___'
RN_ACFT_type_separator = '//' # for pilots to declare their ACFT type in "real name" field

# -------------------------------


# ATC-Pie escaped commands in "#TM" packets:
#  - ATCPIE   Declare ATC-Pie version and social name
#  - STRIP    Strip exchange
#  - WHOHAS   Who-has request
#  - IHAVE    Who-has answer
#  - PHONE_NUMBER   Phone line number info for a client
#  - PHONE_REQUEST  Phone line opening
#  - PHONE_DROP     Phone line closing
#  - CPDLC_XFR_INIT    Data link transfer proposal
#  - CPDLC_XFR_CANCEL  Data link transfer proposal cancelled
#  - CPDLC_XFR_ACCEPT  Data link transfer accept
#  - CPDLC_XFR_REJECT  Data link transfer reject



class ClientInfoKey:
    info_keys = CID, SOCIAL_NAME, IS_ATCPIE, TYPE = range(4)


def dest_concerns_me(dest):
    # '@'-prefixed freq's are 5-digit strings containing value in kHz without the leading '1'
    if dest.startswith('@') and dest[1:].isdigit() and settings.publicised_frequency is not None:
        try:
            return CommFrequency('1' + dest[1:]).inTune(settings.publicised_frequency)
        except ValueError:
            return False
    else:
        return dest in ['*', '*A', '@499999', settings.my_callsign] # @499999 seems to be Euroscope's "all ATCs"




class FsdPhoneManager(AbstractVoipPhoneManager):
    def __init__(self, gui):
        AbstractVoipPhoneManager.__init__(self, gui)

    def setupComms(self, udp_socket, send_text_cmd_function):
        self.phone_socket = udp_socket
        self.send_text_cmd_function = send_text_cmd_function

    ## Defining AbstractVoipPhoneManager methods below
    def sendPhoneData(self, data, inet_addr):
        self.phone_socket.sendto(b'ATCPIE' + data, inet_addr)

    def _sendRequest(self, atc):
        self.send_text_cmd_function(atc, 'PHONE_REQUEST')

    def _sendDrop(self, atc):
        self.send_text_cmd_function(atc, 'PHONE_DROP')




class FsdSessionManager(SessionManager):
    def __init__(self, gui):
        SessionManager.__init__(self, gui, SessionType.FSD)
        self.socket = None # None here when simulation NOT running
        self.disconnect_on_purpose = False
        self.last_received_error = None
        self.client_table = {} # callsign -> info key -> value
        self.ACFT_list = [] # FsdAircraft list
        self.ACFT_list_mutex = QMutex() # Possibly critical: FSD connection modifying traffic vs. getAircraft
        self.FSD_connection = FsdConnection(gui)
        self.position_update_ticker = Ticker(gui, self.FSD_connection.sendPositionUpdate)
        self.weather_ticker = Ticker(gui, self.weatherTick)
        self.ATIS_ticker = Ticker(gui, self.atisTick)
        self.real_weather_checker = None if settings.FSD_weather_from_server else RealWeatherChecker(gui, register_weather_information)
        self.Hoppie_communicator = HoppieCommunicator(gui) if settings.FSD_Hoppie_enabled else None
        self.FSD_connection.cmdReceived.connect(self.fsdCmdReceived)
        self.FSD_connection.connectionDropped.connect(self._fsdDisconnected)
        if pyaudio_available and settings.phone_lines_enabled:
            self.phone_manager = FsdPhoneManager(gui)
        else:
            self.phone_manager = None

    def _fsdDisconnected(self):
        self.position_update_ticker.stop()
        self.weather_ticker.stop()
        self.ATIS_ticker.stop()
        signals.fastClockTick.disconnect(self.updateAllAcftLiveStatuses)
        if settings.FGCom_enabled:
            self.fgcom_ticker.stop()
            self.fgcom_listener.stop()
            self.fgcom_listener.wait()
            self.voice_socket = None
        if self.phone_manager is not None:
            self.phone_manager.stopAndWait()
        if self.Hoppie_communicator is not None:
            self.Hoppie_communicator.stopPolling()
        if self.real_weather_checker is not None:
            self.real_weather_checker.wait()
        self.ACFT_list.clear()
        self.client_table.clear()
        signals.sessionEnded.emit(SessionType.FSD)
        if not self.disconnect_on_purpose:
            msg = 'Connection dropped.'
            if self.last_received_error is not None:
                msg += '\nLast error message received: "%s"' % self.last_received_error
            QMessageBox.critical(self.gui, 'FSD error', msg)

    def start(self):
        self.disconnect_on_purpose = False
        self.last_received_error = None
        if self.FSD_connection.initOK():
            print('Connected to FSD server.')
            self.ACFT_list.clear()
            self.client_table.clear()
            if settings.FGCom_enabled:
                try:
                    self.voice_socket = socket(AF_INET, SOCK_DGRAM)
                    self.voice_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
                    self.voice_socket.bind(('', settings.FSD_voice_system_port))
                    self.fgcom_ticker = Ticker(self.gui, lambda: send_FGCom_mumble_control_packet(self.voice_socket, settings.FGCom_mumble_host, settings.FGCom_mumble_port, settings.FGCom_mumble_sound_effects))
                    self.fgcom_ticker.start(FGCom_tick_interval)
                    self.fgcom_listener = UdpSessionListener(self.gui, self.voice_socket, self.receiveUdpPacket)
                    self.fgcom_listener.start()
                except OSError as error:
                    self.socket = None
                    print('FGCom socket creation error: %s' % error, file=stderr)
                    return
            if self.phone_manager is not None:
                self.phone_manager.setupComms(self.voice_socket, lambda atc, cmd: self.sendAtcPieEscapedMsg(cmd, '', privateTo=atc))
                self.phone_manager.start()
            if self.Hoppie_communicator is not None:
                self.Hoppie_communicator.startPolling()
            signals.sessionStarted.emit(SessionType.FSD)
            signals.fastClockTick.connect(self.updateAllAcftLiveStatuses)
            if settings.FSD_METAR_update_interval is not None:
                self.weather_ticker.startTicking(settings.FSD_METAR_update_interval)
            self.ATIS_ticker.startTicking(ATIS_ticker_interval)
            self.position_update_ticker.startTicking(position_update_interval)
            self.sendAtcPieEscapedMsg('ATCPIE', version_string) # to all ATCs
            if self.phone_manager is not None:
                self.sendAtcPieEscapedMsg('PHONE_NUMBER', INET_addr_str(settings.reachable_phone_IP, settings.FSD_voice_system_port))
        else:
            QMessageBox.critical(self.gui, 'FSD error', 'Connection failed.')

    def stop(self):
        if self.isRunning():
            self.disconnect_on_purpose = True
            self.FSD_connection.shutdown() # emits a disconnection signal

    def isRunning(self):
        return self.FSD_connection.isConnected()

    def clockTime(self):
        return realTime()

    def getAircraft(self):
        self.ACFT_list_mutex.lock()
        result = self.ACFT_list[:]
        self.ACFT_list_mutex.unlock()
        return result


    ## ACFT/ATC INTERACTION

    def instructAircraftByCallsign(self, callsign, instr):
        signals.textInstructionSuggestion.emit(callsign, instr.readOutStr(env.radarContactByCallsign(callsign)))

    def postTextRadioMsg(self, msg):
        if settings.publicised_frequency is None:
            raise TextMsgBlocked('No publicised frequency to post radio message to.')
        self.FSD_connection.sendTextMsg(msg, frq=settings.publicised_frequency)

    def postAtcChatMsg(self, msg):
        if msg.txtOnly().startswith(TM_escape_prefix):
            raise TextMsgBlocked('Message starts with "%s".' % TM_escape_prefix)
        self.FSD_connection.sendTextMsg(msg)

    def sendStrip(self, strip, atc):
        if self.client_table.get(atc, {}).get(ClientInfoKey.IS_ATCPIE, False):
            self.sendAtcPieEscapedMsg('STRIP', strip.encodeDetails(handover_details), privateTo=atc)
        else: # sending to a *NON* ATC-Pie client
            cs = strip.callsign() # XPDR not normally squawking callsign anyway
            if cs is None:
                raise HandoverBlocked('A callsign must be on the strip when not sending to ATC-Pie.')
            QMessageBox.warning(self.gui, 'Hand-over warning', 'This ATC is not using ATC-Pie. '
                                'Handover will only be notified if callsign is connected to the server, '
                                'and all strip changes and assignments will be lost (recipient will likely rely on FPL).')
            self.FSD_connection.sendNonAtcPieHandover(atc, cs)

    def sendWhoHas(self, callsign):
        self.sendAtcPieEscapedMsg('WHOHAS', callsign) # to all ATCs

    def sendCpdlcMsg(self, callsign, msg):
        if self.Hoppie_communicator is None:
            raise CpdlcOperationBlocked('Hoppie sub-system must be enabled for CPDLC in FSD sessions.')
        encoded = '/'.join(CPDLC_element_display_text(elt.replace('@', '_'), varFmt='@%s@') for elt in msg.elements())
        ra_lst = ['WU', 'AN', 'R', 'Y', 'N'] if msg.isUplink() else ['Y', 'N']
        self.Hoppie_communicator.sendCpdlcData(callsign, encoded, ra_lst[msg.responseAttributePrecedence() - 1], incrPolling=msg.expectsAnswer())

    def sendCpdlcTransferRequest(self, acft_callsign, atc_callsign, proposing):
        if self.Hoppie_communicator is None:
            raise CpdlcOperationBlocked('Hoppie sub-system must be enabled for CPDLC in FSD sessions.')
        if proposing:
            self.sendAtcPieEscapedMsg('CPDLC_XFR_INIT', acft_callsign, privateTo=atc_callsign)
            self.Hoppie_communicator.sendCpdlcData(acft_callsign, 'HANDOVER @%s' % atc_callsign, 'NE', incrPolling=True)
        else:
            self.sendAtcPieEscapedMsg('CPDLC_XFR_CANCEL', acft_callsign, privateTo=atc_callsign)

    def sendCpdlcTransferResponse(self, acft_callsign, atc_callsign, accepting):
        if self.Hoppie_communicator is None:
            raise CpdlcOperationBlocked('Hoppie sub-system must be enabled for CPDLC in FSD sessions.')
        if accepting:
            self.sendAtcPieEscapedMsg('CPDLC_XFR_ACCEPT', acft_callsign, privateTo=atc_callsign)
            self.Hoppie_communicator.sendCpdlcData(acft_callsign, 'LOGON ACCEPTED', 'NE')
        else:
            self.sendAtcPieEscapedMsg('CPDLC_XFR_REJECT', acft_callsign, privateTo=atc_callsign)

    def sendCpdlcDisconnect(self, callsign):
        if self.Hoppie_communicator is None:
            raise CpdlcOperationBlocked('Hoppie sub-system must be enabled for CPDLC in FSD sessions.')
        self.Hoppie_communicator.sendCpdlcData(callsign, 'LOGOFF', 'NE')


    ## VOICE COMM'S

    def createRadio(self):
        if settings.FGCom_enabled:
            return FGComRadio()
        else:
            QMessageBox.information(self.gui, 'Create radio', 'FGCom-mumble sub-system must be enabled for integrated radios.')
            return None

    def recordAtis(self, parent_dialog):
        if settings.FGCom_enabled:
            record_FGCom_Mumble_ATIS(parent_dialog)

    def phoneLineManager(self):
        return self.phone_manager # can be None


    ## ONLINE SYSTEMS

    def weatherLookUpRequest(self, station):
        if settings.FSD_weather_from_server:
            self.FSD_connection.sendMetarRequest(station)
        else:
            self.real_weather_checker.lookupStation(station)

    def pushFplOnline(self, fpl):
        raise OnlineFplActionBlocked('FSD does not allow ATCs to file or amend FPLs online.')

    def changeFplStatus(self, fpl, new_status):
        raise OnlineFplActionBlocked('FSD does not implement FPL open/close.')

    def syncOnlineFPLs(self):
        for callsign in self.client_table:
            if callsign not in env.ATCs.knownAtcCallsigns():
                self.FSD_connection.sendQuery('SERVER', 'FP:%s' % callsign)


    ## MANAGER-SPECIFIC

    def registerClientInfo(self, callsign, info_key, value, reqFplIfNew=True):
        if callsign in self.client_table:
            self.client_table[callsign][info_key] = value
        else: # new client
            if reqFplIfNew:
                self.FSD_connection.sendQuery('SERVER', 'FP:%s' % callsign)
            self.client_table[callsign] = {info_key: value}

    def updateAllAcftLiveStatuses(self):
        self.ACFT_list_mutex.lock()
        for fsd_acft in self.ACFT_list:
            fsd_acft.updateLiveStatusWithEstimate()
            send_packet_to_views(fsd_acft.fgmsPositionPacket())
        self.ACFT_list_mutex.unlock()

    def weatherTick(self):
        if settings.FSD_weather_from_server:
            self.FSD_connection.sendMetarRequest(settings.primary_METAR_station)
            for station in settings.additional_METAR_stations:
                self.FSD_connection.sendMetarRequest(station)
        else:
            self.real_weather_checker.lookupSelectedStations()

    def atisTick(self):
        if settings.last_recorded_ATIS is not None:
            for line in settings.last_recorded_ATIS[3].split('\n'):
                if line.strip() != '': # avoid empty lines in text messages
                    self.FSD_connection.sendTextMsg(TextMessage(settings.my_callsign, line), frq=settings.last_recorded_ATIS[2])

    def sendAtcPieEscapedMsg(self, escaped_cmd, arg_str, privateTo=None):
        msg_txt = '%s%s %s' % (TM_escape_prefix, escaped_cmd, arg_str)
        if privateTo is None:
            msg = TextMessage(settings.my_callsign, msg_txt, private=False)
        else:
            msg = TextMessage(settings.my_callsign, msg_txt, recipient=privateTo, private=True)
        self.FSD_connection.sendTextMsg(msg)

    def processEscapedTM(self, sender, cmd, arg):
        if cmd == 'ATCPIE': # arg: version
            if not self.client_table.get(sender, {}).get(ClientInfoKey.IS_ATCPIE, False):
                self.registerClientInfo(sender, ClientInfoKey.IS_ATCPIE, True, reqFplIfNew=False)
                self.sendAtcPieEscapedMsg('ATCPIE', version_string, privateTo=sender)
                if self.phone_manager is not None: # send my phone number to anybody declaring their client as ATC-Pie
                    self.sendAtcPieEscapedMsg('PHONE_NUMBER', INET_addr_str(settings.reachable_phone_IP, settings.FSD_voice_system_port), privateTo=sender)
        elif cmd == 'STRIP': # arg: encoded strip details
            strip = Strip.fromEncodedDetails(arg)
            strip.writeDetail(received_from_detail, sender)
            signals.receiveStrip.emit(strip)
        elif cmd == 'WHOHAS': # arg: ACFT callsign
            if env.shouldAnswerWhoHas(arg):
                self.sendAtcPieEscapedMsg('IHAVE', arg, privateTo=sender)
        elif cmd == 'IHAVE': # arg: ACFT callsign
            signals.incomingContactClaim.emit(sender, arg)
        elif cmd == 'PHONE_NUMBER': # arg: publicised INET address
            if self.phone_manager is not None:
                try:
                    self.phone_manager.updatePhoneBook(sender, INET_addr_from_str(arg))
                except ValueError:
                    pass # ignore invalid phone number string
        elif cmd == 'PHONE_REQUEST': # no args
            if self.phone_manager is not None:
                self.phone_manager.incomingLineRequest(sender)
        elif cmd == 'PHONE_DROP': # no args
            if self.phone_manager is not None:
                self.phone_manager.incomingLineDrop(sender)
        elif cmd == 'CPDLC_XFR_INIT': # arg: ACFT callsign
            signals.cpdlcTransferRequest.emit(arg, sender, True)
        elif cmd == 'CPDLC_XFR_CANCEL': # arg: ACFT callsign
            signals.cpdlcTransferRequest.emit(arg, sender, False)
        elif cmd == 'CPDLC_XFR_ACCEPT': # arg: ACFT callsign
            signals.cpdlcTransferResponse.emit(arg, sender, True)
        elif cmd == 'CPDLC_XFR_REJECT': # arg: ACFT callsign
            signals.cpdlcTransferResponse.emit(arg, sender, False)
        else:
            print('Unhandled "%s" message from %s with arg "%s"' % (cmd, sender, arg), file=stderr)

    def fsdCmdReceived(self, cmd, fields):
        if cmd == '#AA' and len(fields) == 6: # Add ATC client
            callsign, srv, social_name, cid, wtf, rating = fields
            self.registerClientInfo(callsign, ClientInfoKey.CID, cid, reqFplIfNew=False)
            self.registerClientInfo(callsign, ClientInfoKey.SOCIAL_NAME, social_name, reqFplIfNew=False)

        elif cmd == '#AP' and len(fields) == 7: # Add pilot client
            callsign, srv, cid, wtf1, wtf2, wtf3, wtf4 = fields
            self.registerClientInfo(callsign, ClientInfoKey.CID, cid)

        elif cmd == '#DA' and len(fields) == 2: # Remove ATC client
            callsign, cid = fields
            if callsign in self.client_table:
                del self.client_table[callsign]
            try:
                env.ATCs.removeATC(callsign)
            except KeyError:
                pass

        elif cmd == '#DL' and len(fields) == 4: # Heart beat from server
            pass

        elif cmd == '#DP' and len(fields) == 2: # Remove pilot client
            callsign, cid = fields
            if callsign in self.client_table:
                del self.client_table[callsign]
            self.ACFT_list_mutex.lock()
            pop_all(self.ACFT_list, lambda acft: acft.identifier == callsign)
            self.ACFT_list_mutex.unlock()

        elif cmd == '#TM' and len(fields) == 3: # Text message
            src, dest_conjunction, msg = fields
            for dest in dest_conjunction.split('&'):
                if dest_concerns_me(dest):
                    if src == 'server':
                        signals.statusBarMsg.emit('Server message: ' + msg)
                        print('FSD server sends message:', msg)
                    elif dest.startswith('@'):
                        signals.incomingTextRadioMsg.emit(TextMessage(src, msg, private=False))
                    elif msg.startswith(TM_escape_prefix):
                        split = msg[len(TM_escape_prefix):].split(' ', maxsplit=1)
                        self.processEscapedTM(src, split[0], ('' if len(split) == 1 else split[1]))
                    elif dest == settings.my_callsign:
                        signals.incomingAtcTextMsg.emit(TextMessage(src, msg, recipient=settings.my_callsign, private=True))
                    else:
                        signals.incomingAtcTextMsg.emit(TextMessage(src, msg, recipient=None, private=False))

        elif cmd == '$AR' and len(fields) == 4:
            if fields[2] == 'METAR':
                register_weather_information(Weather(fields[3]))
            else:
                print('Unhandled $AR "%s" packet' % fields[2], file=stderr)

        elif cmd == '$CQ' and len(fields) == 3: # Query
            src, dest, req = fields
            if dest_concerns_me(dest):
                if req == 'RN':
                    self.FSD_connection.sendQueryResponse(src, 'RN', '%s::%d' % (settings.MP_social_name, settings.FSD_rating))

        elif cmd == '$CR' and len(fields) == 4: # Query response
            src, dest, req, answer = fields
            if dest_concerns_me(dest):
                if req == 'RN':
                    real_name_field = answer.split(':', maxsplit=1)[0]
                    if RN_ACFT_type_separator in real_name_field:
                        lft, rgt = real_name_field.split(RN_ACFT_type_separator, maxsplit=1)
                        self.registerClientInfo(src, ClientInfoKey.TYPE, lft)
                        self.registerClientInfo(src, ClientInfoKey.SOCIAL_NAME, rgt)
                    else:
                        self.registerClientInfo(src, ClientInfoKey.TYPE, missing_client_type_str)
                        self.registerClientInfo(src, ClientInfoKey.SOCIAL_NAME, real_name_field)
                else:
                    print('Unexpected $CR response "%s" from %s' % (req, src), file=stderr)

        elif cmd == '$ER' and len(fields) == 5: # Error message
            src, dest, errcode, wtf, msg = fields
            self.last_received_error = msg
            if not (errcode.isdigit() and int(errcode) == 8):
                print('FSD error packet received:', ':'.join(fields), file=stderr)

        elif cmd == '$FP' and len(fields) == 17: # Flight plan
            fpl = FPL_from_fields(*fields)
            fpl.markAsOnline(fields[0])
            env.FPLs.updateFromOnlineDownload(fpl)

        elif cmd == '$HO' and len(fields) == 3: # Handover
            src, dest, acft_callsign = fields
            strip = Strip()
            strip.writeDetail(FPL.CALLSIGN, acft_callsign)
            got_online = env.FPLs.findAll(lambda fpl: fpl.online_id == acft_callsign)
            if len(got_online) == 0:
                print('Received strip from non ATC-Pie client while no FPL was found online for %s' % acft_callsign, file=stderr)
            else:
                strip.fillFromFPL(useFpl=got_online)
                strip.writeDetail(received_from_detail, src)
                signals.receiveStrip.emit(strip)

        elif cmd == '%' and len(fields) == 8: # ATC update
            callsign, frq, wtf1, vis, rating, lat, lon, wtf2 = fields
            try:
                social_name = self.client_table[callsign][ClientInfoKey.SOCIAL_NAME]
            except KeyError:
                social_name = None
                self.FSD_connection.sendQuery(callsign, 'RN')
            try:
                coords = EarthCoords(float(lat), float(lon))
                comm_freq = None if frq == '' or frq == '0' else CommFrequency('1' + frq)
            except ValueError as err:
                print('Value error in position update from "%s": %s' % (callsign, err), file=stderr)
            else:
                env.ATCs.updateATC(callsign, coords, social_name, comm_freq)

        # ---------- FIXED PILOT POSITION HANDLER ----------

        elif cmd == '@' and len(fields) == 10: # Pilot update
            sqmode, callsign, sqcode, rating, lat, lon, amsl, spd, wtf1, wtf2 = fields
            try:
                coords = EarthCoords(float(lat), float(lon))
                real_alt = float(amsl)
                gnd_speed = Speed(float(spd))
                xpdr = {}
                if sqmode != 'S': # not "standby"
                    xpdr[Xpdr.CODE] = int(sqcode, base=8)
                    xpdr[Xpdr.IDENT] = sqmode == 'Y'
                    xpdr[Xpdr.ALT] = PressureAlt(real_alt)
            except ValueError as err:
                print('Value error in position update from "%s": %s' % (callsign, err), file=stderr)
            else:
                self.ACFT_list_mutex.lock()
                try:
                    fsd_acft = next(acft for acft in self.ACFT_list if acft.identifier == callsign)
                    fsd_acft.updatePdStatus(coords, real_alt, gnd_speed, xpdr)
                except StopIteration: # new callsign
                    # NEW: always create an aircraft, even if #AP not seen yet
                    acft_type = self.client_table.get(callsign, {}).get(ClientInfoKey.TYPE, '')
                    fsd_acft = FsdAircraft(callsign, acft_type, coords, real_alt, gnd_speed, xpdr)
                    self.ACFT_list.append(fsd_acft)
                    if callsign not in self.client_table:
                        self.registerClientInfo(callsign, ClientInfoKey.TYPE, '', reqFplIfNew=True)
                        self.FSD_connection.sendQuery(callsign, 'RN')
                self.ACFT_list_mutex.unlock()
                if fsd_acft is not None:
                    send_packet_to_views(fsd_acft.fgmsPositionPacket())

        else:
            print('Ignoring unhandled "%s" packet.' % cmd, file=stderr)

    def receiveUdpPacket(self, datagram):
        if datagram[:5] == b'FGCOM':
            receive_FGCom_mumble_packet(datagram)
        elif datagram[:6] == b'ATCPIE':
            if self.phone_manager is not None:
                self.phone_manager.receivePhoneData(datagram[6:])
        else:
            print('Unrecognised or unexpected packet type received on port %d.' % settings.FGMS_client_port, file=stderr)
