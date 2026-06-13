
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

from base.instr import Instruction

from session.config import settings
from session.manager import SessionType


# ---------- Constants ----------

cpdlc_msg_element_separator = '|' # keep non-alpha-num and without spaces

# -------------------------------


class CpdlcMessage:
	def __init__(self, contents, timeStamp=None):
		"""
		contents: str for a single element, or str list for multiple elements, each with element identifier as first word
		time: defaults to current session clock time
		"""
		self.time_stamp = timeStamp if timeStamp else settings.session_manager.clockTime()
		self.msg_elements = [contents] if isinstance(contents, str) else contents
		try:
			assert all(elt.split(' ')[0] in CPDLC_element_formats for elt in self.msg_elements)
			assert all(elt[3] == self.msg_elements[0][3] for elt in self.msg_elements)
		except (IndexError, AssertionError):
			raise ValueError('Empty CPDLC message or invalid element identifier.')
		try:
			self.recognised_instructions = [Instruction.fromCpdlcMsgElement(instr) for instr in self.msg_elements]
		except ValueError:
			self.recognised_instructions = None

	@staticmethod
	def fromEncodedStr(encoded):
		sep = encoded.split(' ', maxsplit=1)[0]
		if len(sep) > 0 and not sep[0].isalnum():  # custom separator defined (to be surrounded by spaces between elements)
			elt_split = encoded[:len(sep) + 1].split(' %s ' % sep)
		else:  # simple separation
			elt_split = encoded.split(cpdlc_msg_element_separator)
		return CpdlcMessage(elt_split)
	
	def timeStamp(self):
		return self.time_stamp

	def elements(self):
		return self.msg_elements
	
	def isFromMe(self):
		return self.isUplink() == (settings.session_manager.session_type != SessionType.TEACHER)
	
	def isUplink(self):
		return self.msg_elements[0][3] == 'U'
	
	def isDownlink(self):
		return not self.isUplink()
	
	def isAcknowledgement(self):  # single ROGER or WILCO, uplink or downlink
		return len(self.msg_elements) == 1 \
				and self.msg_elements[0] in [RspId.uplink_ROGER, RspId.downlink_ROGER, RspId.downlink_WILCO]
	
	def isStandby(self):  # single STANDBY, uplink or downlink
		return len(self.msg_elements) == 1 \
				and (self.msg_elements[0] == RspId.uplink_STANDBY or self.msg_elements[0] == RspId.downlink_STANDBY)
	
	def containsUnable(self):  # contains UNABLE, uplink or downlink
		return any(elt == RspId.uplink_UNABLE or elt == RspId.downlink_UNABLE for elt in self.msg_elements)
	
	def expectsAnswer(self):
		return self.responseAttributePrecedence() < (N if self.isUplink() else DN)

	def responseAttributePrecedence(self): # see ICAO doc. 4444, 16th edition, page 14-5
		return min(element_response_attribute_precedence(elt.split(' ')[0]) for elt in self.msg_elements)
	
	def recognisedInstructions(self): # ATC instructions if msg is uplink; requested instructions if downlink
		return self.recognised_instructions
	
	def displayText(self, sepStr='\n'):
		return sepStr.join(CPDLC_element_display_text(elt) for elt in self.msg_elements)
	
	def toEncodedStr(self):
		sep = cpdlc_msg_element_separator
		if len(self.msg_elements) >= 2 and any(sep in elt for elt in self.msg_elements):
			while any(sep in elt for elt in self.msg_elements):
				sep += cpdlc_msg_element_separator
		if sep == cpdlc_msg_element_separator:
			return sep.join(self.msg_elements)
		else:
			return '%s ' % sep + (' %s ' % sep).join(self.msg_elements)


class RspId:
	uplink_UNABLE = 'RSPU-1'
	uplink_STANDBY = 'RSPU-2'
	uplink_ROGER = 'RSPU-4'
	uplink_AFFIRM = 'RSPU-5'
	uplink_NEGATIVE = 'RSPU-6'
	downlink_WILCO = 'RSPD-1'
	downlink_UNABLE = 'RSPD-2'
	downlink_STANDBY = 'RSPD-3'
	downlink_ROGER = 'RSPD-4'
	downlink_AFFIRM = 'RSPD-5'
	downlink_NEGATIVE = 'RSPD-6'



uplink_prec_levels = WU, AN, R, Y, N = range(1, 6)
downlink_prec_levels = DY, DN = range(1, 3)

def element_response_attribute_precedence(elt_id): # see ICAO doc. 4444, 16th edition, page 14-5
	n = int(elt_id.split('-')[1]) # CAUTION: element ID *must* be valid
	# UPLINKS: return value in 1..5
	if elt_id.startswith('RTEU'):
		return WU if n <= 12 else R if n <= 14 else Y
	elif elt_id.startswith('LATU'):
		return R if n == 7 or n == 8 else WU
	elif elt_id.startswith('LVLU'):
		return R if n <= 4 or n == 22 else Y if n == 25 or 27 <= n <= 30 else AN if n >= 31 else WU
	elif elt_id.startswith('CSTU'):
		return WU
	elif elt_id.startswith('SPDU'):
		return R if n <= 3 or n == 14 else Y if n >= 15 else WU
	elif elt_id.startswith('ADVU'):
		return N if n == 6 else R if n <= 7 else Y if n == 14 else WU
	elif elt_id.startswith('COMU'):
		return R if n == 4 else N if n >= 8 else WU
	elif elt_id.startswith('SPCU'):
		return N
	elif elt_id.startswith('EMGU'):
		return {1: Y, 2: N, 3: AN}[n]
	elif elt_id.startswith('RSPU'):
		return N
	elif elt_id.startswith('SUPU'):
		return N
	elif elt_id.startswith('TXTU'):
		return {1: R, 2: N, 3: N, 4: WU, 5: AN}[n]
	elif elt_id.startswith('SYSU'):
		return N
	# DOWNLINKS: return value in 1..2
	if elt_id.startswith('RTED'):
		return DN if n == 5 or n >= 9 else DY
	elif elt_id.startswith('LATD'):
		return DN if n == 3 or n == 4 or n == 8 else DY
	elif elt_id.startswith('LVLD'):
		return DN if n >= 8 else DY
	elif elt_id.startswith('SPDD'):
		return DN if n >= 3 else DY
	elif elt_id.startswith('ADVD'):
		return DN
	elif elt_id.startswith('COMD'):
		return DN if n == 2 else DY
	elif elt_id.startswith('SPCD'):
		return DN
	elif elt_id.startswith('EMGD'):
		return DY
	elif elt_id.startswith('RSPD'):
		return DN
	elif elt_id.startswith('SUPD'):
		return DN
	elif elt_id.startswith('TXTD'):
		return DN if n == 2 else DY
	elif elt_id.startswith('SYSD'):
		return DN


args_allowing_spaces = ['FUEL', 'LEGTYPE', 'MINUTES', 'REASON', 'ROUTE', 'TEXT', 'VSPEED']
# CAUTION: There should be a maximum of one arg allowing contained spaces in any message element format
CPDLC_element_formats = {
	'RTEU-1': '{TEXT}',
	'RTEU-2': 'PROCEED DIRECT TO {POINT}',
	'RTEU-3': 'AT TIME {TIME} PROCEED DIRECT TO {POINT}',
	'RTEU-4': 'AT {POINT} PROCEED DIRECT TO {POINT}',
	'RTEU-5': 'AT {FL_ALT} PROCEED DIRECT TO {POINT}',
	'RTEU-6': 'CLEARED TO {POINT} VIA {ROUTE}',
	'RTEU-7': 'CLEARED {ROUTE}',
	'RTEU-8': 'CLEARED {PROCEDURE}',
	'RTEU-9': 'AT {POINT} CLEARED {ROUTE}',
	'RTEU-10': 'AT {POINT} CLEARED {PROCEDURE}',
	'RTEU-11': 'AT {POINT} HOLD INBOUND TRACK {DEGREES} {DIRECTION} TURNS {LEGTYPE} LEGS',
	'RTEU-12': 'AT {POINT} HOLD AS PUBLISHED',
	'RTEU-13': 'EXPECT FURTHER CLEARANCE AT {TIME}',
	'RTEU-14': 'EXPECT {CLRTYPE}',
	'RTEU-15': 'CONFIRM ASSIGNED ROUTE',
	'RTEU-16': 'REQUEST POSITION REPORT',
	'RTEU-17': 'ADVISE ETA {POINT}',

	'RTED-1': 'REQUEST DIRECT TO {POINT}',
	'RTED-2': 'REQUEST {TEXT}',
	'RTED-3': 'REQUEST CLEARANCE {ROUTE}',
	'RTED-4': 'REQUEST {CLRTYPE} CLEARANCE',
	'RTED-5': 'POSITION REPORT {TEXT}',
	'RTED-6': 'REQUEST HEADING {DEGREES}',
	'RTED-7': 'REQUEST GROUND TRACK {DEGREES}',
	'RTED-8': 'WHEN CAN WE EXPECT BACK ON ROUTE',
	'RTED-9': 'ASSIGNED ROUTE {ROUTE}',
	'RTED-10': 'ETA {POINT} TIME {TIME}',

	'LATU-1': 'OFFSET {HDIST} {DIRECTION} OF ROUTE',
	'LATU-2': 'AT {POINT} OFFSET {HDIST} {DIRECTION} OF ROUTE',
	'LATU-3': 'AT TIME {TIME} OFFSET {HDIST} {DIRECTION} OF ROUTE',
	'LATU-4': 'REJOIN ROUTE',
	'LATU-5': 'REJOIN ROUTE BEFORE PASSING {POINT}',
	'LATU-6': 'REJOIN ROUTE BEFORE TIME {TIME}',
	'LATU-7': 'EXPECT BACK ON ROUTE BEFORE PASSING {POINT}',
	'LATU-8': 'EXPECT BACK ON ROUTE BEFORE TIME {TIME}',
	'LATU-9': 'RESUME OWN NAVIGATION',
	'LATU-10': 'CLEARED TO DEVIATE UP TO {HDIST} OF ROUTE',
	'LATU-11': 'TURN {DIRECTION} HEADING {DEGREES}',
	'LATU-12': 'TURN {DIRECTION} GROUND TRACK {DEGREES}',
	'LATU-13': 'TURN {DIRECTION} {NDEG} DEGREES',
	'LATU-14': 'CONTINUE PRESENT HEADING',
	'LATU-15': 'AT {POINT} FLY HEADING {DEGREES}',
	'LATU-16': 'FLY HEADING {DEGREES}',
	'LATU-17': 'REPORT CLEAR OF WEATHER',
	'LATU-18': 'REPORT BACK ON ROUTE',
	'LATU-19': 'REPORT PASSING {POINT}',

	'LATD-1': 'REQUEST OFFSET {HDIST} {DIRECTION} OF ROUTE',
	'LATD-2': 'REQUEST WEATHER DEVIATION UP TO {HDIST} OF ROUTE',
	'LATD-3': 'CLEAR OF WEATHER',
	'LATD-4': 'BACK ON ROUTE',
	'LATD-5': 'DIVERTING TO {POINT} VIA {ROUTE}',
	'LATD-6': 'OFFSETTING {HDIST} {DIRECTION} OF ROUTE',
	'LATD-7': 'DEVIATING {HDIST} {DIRECTION} OF ROUTE',
	'LATD-8': 'PASSING {POINT}',

	'LVLU-1': 'EXPECT HIGHER AT TIME {TIME}',
	'LVLU-2': 'EXPECT HIGHER AT {POINT}',
	'LVLU-3': 'EXPECT LOWER AT TIME {TIME}',
	'LVLU-4': 'EXPECT LOWER AT {POINT}',
	'LVLU-5': 'MAINTAIN {FL_ALT}',
	'LVLU-6': 'CLIMB TO {FL_ALT}',
	'LVLU-7': 'AT TIME {TIME} CLIMB TO {FL_ALT}',
	'LVLU-8': 'AT {POINT} CLIMB TO {FL_ALT}',
	'LVLU-9': 'DESCEND TO {FL_ALT}',
	'LVLU-10': 'AT TIME {TIME} DESCEND TO {FL_ALT}',
	'LVLU-11': 'AT {POINT} DESCEND TO {FL_ALT}',
	'LVLU-12': 'CLIMB TO REACH {FL_ALT} BEFORE TIME {TIME}',
	'LVLU-13': 'CLIMB TO REACH {FL_ALT} BEFORE PASSING {POINT}',
	'LVLU-14': 'DESCEND TO REACH {FL_ALT} BEFORE TIME {TIME}',
	'LVLU-15': 'DESCEND TO REACH {FL_ALT} BEFORE PASSING {POINT}',
	'LVLU-16': 'STOP CLIMB AT {FL_ALT}',
	'LVLU-17': 'STOP DESCENT AT {FL_ALT}',
	'LVLU-18': 'CLIMB AT {VSPEED} OR GREATER',
	'LVLU-19': 'CLIMB AT {VSPEED} OR LESS',
	'LVLU-20': 'DESCEND AT {VSPEED} OR GREATER',
	'LVLU-21': 'DESCEND AT {VSPEED} OR LESS',
	'LVLU-22': 'EXPECT {FL_ALT} {MINUTES} AFTER DEPARTURE',
	'LVLU-23': 'REPORT LEAVING {FL_ALT}',
	'LVLU-24': 'REPORT MAINTAINING {FL_ALT}',
	'LVLU-25': 'REPORT PRESENT LEVEL',
	'LVLU-26': 'REPORT REACHING BLOCK {FL_ALT} TO {FL_ALT}',
	'LVLU-27': 'CONFIRM ASSIGNED LEVEL',
	'LVLU-28': 'ADVISE PREFERRED LEVEL',
	'LVLU-29': 'ADVISE TOP OF DESCENT',
	'LVLU-30': 'WHEN CAN YOU ACCEPT {FL_ALT}',
	'LVLU-31': 'CAN YOU ACCEPT {FL_ALT} AT {POINT}',
	'LVLU-32': 'CAN YOU ACCEPT {FL_ALT} AT TIME {TIME}',

	'LVLD-1': 'REQUEST LEVEL {FL_ALT}',
	'LVLD-2': 'REQUEST CLIMB TO {FL_ALT}',
	'LVLD-3': 'REQUEST DESCENT TO {FL_ALT}',
	'LVLD-4': 'AT {POINT} REQUEST {FL_ALT}',
	'LVLD-5': 'AT TIME {TIME} REQUEST {FL_ALT}',
	'LVLD-6': 'WHEN CAN WE EXPECT LOWER LEVEL',
	'LVLD-7': 'WHEN CAN WE EXPECT HIGHER LEVEL',
	'LVLD-8': 'LEAVING LEVEL {FL_ALT}',
	'LVLD-9': 'MAINTAINING LEVEL {FL_ALT}',
	'LVLD-10': 'REACHING BLOCK {FL_ALT} TO {FL_ALT}',
	'LVLD-11': 'ASSIGNED LEVEL {FL_ALT}',
	'LVLD-12': 'PREFERRED LEVEL {FL_ALT}',
	'LVLD-13': 'CLIMBING TO {FL_ALT}',
	'LVLD-14': 'DESCENDING TO {FL_ALT}',
	'LVLD-15': 'WE CAN ACCEPT {FL_ALT} AT TIME {TIME}',
	'LVLD-16': 'WE CAN ACCEPT {FL_ALT} AT {POINT}',
	'LVLD-17': 'WE CANNOT ACCEPT {FL_ALT}',
	'LVLD-18': 'TOP OF DESCENT {POINT} TIME {TIME}',

	'CSTU-1': 'CROSS {POINT} AT {FL_ALT}',
	'CSTU-2': 'CROSS {POINT} AT OR ABOVE {FL_ALT}',
	'CSTU-3': 'CROSS {POINT} AT OR BELOW {FL_ALT}',
	'CSTU-4': 'CROSS {POINT} AT TIME {TIME}',
	'CSTU-5': 'CROSS {POINT} BEFORE TIME {TIME}',
	'CSTU-6': 'CROSS {POINT} AFTER TIME {TIME}',
	'CSTU-7': 'CROSS {POINT} BETWEEN TIME {TIME} AND TIME {TIME}',
	'CSTU-8': 'CROSS {POINT} AT {SPEED}',
	'CSTU-9': 'CROSS {POINT} AT {SPEED} OR LESS',
	'CSTU-10': 'CROSS {POINT} AT {SPEED} OR GREATER',
	'CSTU-11': 'CROSS {POINT} AT TIME {TIME} AT {FL_ALT}',
	'CSTU-12': 'CROSS {POINT} BEFORE TIME {TIME} AT {FL_ALT}',
	'CSTU-13': 'CROSS {POINT} AFTER TIME {TIME} AT {FL_ALT}',
	'CSTU-14': 'CROSS {POINT} AT {FL_ALT} AT {SPEED}',
	'CSTU-15': 'CROSS {POINT} AT TIME {TIME} AT {FL_ALT} AT {SPEED}',

	'SPDU-1': 'EXPECT SPEED CHANGE AT TIME {TIME}',
	'SPDU-2': 'EXPECT SPEED CHANGE AT {POINT}',
	'SPDU-3': 'EXPECT SPEED CHANGE AT {FL_ALT}',
	'SPDU-4': 'MAINTAIN {SPEED}',
	'SPDU-5': 'MAINTAIN PRESENT SPEED',
	'SPDU-6': 'MAINTAIN {SPEED} OR GREATER',
	'SPDU-7': 'MAINTAIN {SPEED} OR LESS',
	'SPDU-8': 'MAINTAIN {SPEED} TO {SPEED}',
	'SPDU-9': 'INCREASE SPEED TO {SPEED}',
	'SPDU-10': 'INCREASE SPEED TO {SPEED} OR GREATER',
	'SPDU-11': 'REDUCE SPEED TO {SPEED}',
	'SPDU-12': 'REDUCE SPEED TO {SPEED} OR LESS',
	'SPDU-13': 'RESUME NORMAL SPEED',
	'SPDU-14': 'NO SPEED RESTRICTION',
	'SPDU-15': 'REPORT {SPDTYPE} SPEED',
	'SPDU-16': 'CONFIRM ASSIGNED SPEED',
	'SPDU-17': 'WHEN CAN YOU ACCEPT {SPEED}',

	'SPDD-1': 'REQUEST {SPEED}',
	'SPDD-2': 'WHEN CAN WE EXPECT {SPEED}',
	'SPDD-3': '{SPDTYPE} SPEED {SPEED}',
	'SPDD-4': 'ASSIGNED SPEED {SPEED}',
	'SPDD-5': 'WE CAN ACCEPT {SPEED} AT TIME {TIME}',
	'SPDD-6': 'WE CANNOT ACCEPT {SPEED}',

	'ADVU-1': '{CALLSIGN} PRESSURE {PRESSURE}',
	'ADVU-2': 'SERVICE TERMINATED',
	'ADVU-3': 'IDENTIFIED {POINT}',
	'ADVU-4': 'IDENTIFICATION LOST',
	'ADVU-5': 'ATIS {ATIS}',
	'ADVU-6': 'REQUEST AGAIN WITH NEXT ATC UNIT',
	'ADVU-7': 'TRAFFIC IS {TEXT}',
	'ADVU-8': 'REPORT SIGHTING AND PASSING OPPOSITE DIRECTION {TEXT}',
	'ADVU-9': 'SQUAWK {XPDR}',
	'ADVU-10': 'STOP SQUAWK',
	'ADVU-11': 'STOP ADS-B TRANSMISSION',
	'ADVU-12': 'SQUAWK MODE C',
	'ADVU-13': 'STOP SQUAWK MODE C',
	'ADVU-14': 'CONFIRM SQUAWK CODE',
	'ADVU-15': 'SQUAWK IDENT',
	'ADVU-16': 'ACTIVATE ADS-C',
	'ADVU-17': 'ADS-C OUT OF SERVICE REVERT TO VOICE POSITION REPORTS',
	'ADVU-18': 'RELAY TO {CALLSIGN} {TEXT}',
	'ADVU-19': '{DEVTYPE} DEVIATION DETECTED. VERIFY AND ADVISE',

	'ADVD-1': 'SQUAWKING {XPDR}',
	'ADVD-2': 'TRAFFIC {TEXT}',

	'COMU-1': 'CONTACT {CALLSIGN} {FREQ}',
	'COMU-2': 'AT {POINT} CONTACT {CALLSIGN} {FREQ}',
	'COMU-3': 'AT TIME {TIME} CONTACT {CALLSIGN} {FREQ}',
	'COMU-4': 'SECONDARY FREQUENCY {FREQ}',
	'COMU-5': 'MONITOR {CALLSIGN} {FREQ}',
	'COMU-6': 'AT {POINT} MONITOR {CALLSIGN} {FREQ}',
	'COMU-7': 'AT TIME {TIME} MONITOR {CALLSIGN} {FREQ}',
	'COMU-8': 'CHECK STUCK MICROPHONE {FREQ}',
	'COMU-9': 'CURRENT ATC UNIT {TEXT}',

	'COMD-1': 'REQUEST VOICE CONTACT {FREQ}',
	'COMD-2': 'RELAY FROM {TEXT}',

	'SPCU-1': 'ITP BEHIND {CALLSIGN}',
	'SPCU-2': 'ITP AHEAD OF {CALLSIGN}',
	'SPCU-3': 'ITP BEHIND {CALLSIGN} AND BEHIND {CALLSIGN}',
	'SPCU-4': 'ITP AHEAD OF {CALLSIGN} AND AHEAD OF {CALLSIGN}',
	'SPCU-5': 'ITP BEHIND {CALLSIGN} AND AHEAD OF {CALLSIGN}',

	'SPCD-1': 'ITP {HDIST} BEHIND {CALLSIGN}',
	'SPCD-2': 'ITP {HDIST} AHEAD OF {CALLSIGN}',
	'SPCD-3': 'ITP {HDIST} BEHIND {CALLSIGN} AND {HDIST} BEHIND {CALLSIGN}',
	'SPCD-4': 'ITP {HDIST} AHEAD OF {CALLSIGN} AND {HDIST} AHEAD OF {CALLSIGN}',
	'SPCD-5': 'ITP {HDIST} BEHIND {CALLSIGN} AND {HDIST} AHEAD OF {CALLSIGN}',

	'EMGU-1': 'REPORT FUEL AND PERSONS ON BOARD',
	'EMGU-2': 'IMMEDIATELY',
	'EMGU-3': 'CONFIRM ADS-C EMERGENCY',

	'EMGD-1': 'PAN PAN PAN',
	'EMGD-2': 'MAYDAY MAYDAY MAYDAY',
	'EMGD-3': '{FUEL} FUEL AND {POB} POB',
	'EMGD-4': 'CANCEL EMERGENCY',

	'RSPU-1': 'UNABLE',
	'RSPU-2': 'STANDBY',
	'RSPU-3': 'REQUEST DEFERRED',
	'RSPU-4': 'ROGER',
	'RSPU-5': 'AFFIRM',
	'RSPU-6': 'NEGATIVE',
	'RSPU-7': 'REQUEST FORWARDED',
	'RSPU-8': 'CONFIRM REQUEST',

	'RSPD-1': 'WILCO',
	'RSPD-2': 'UNABLE',
	'RSPD-3': 'STANDBY',
	'RSPD-4': 'ROGER',
	'RSPD-5': 'AFFIRM',
	'RSPD-6': 'NEGATIVE',

	'SUPU-1': 'WHEN READY',
	'SUPU-2': 'DUE TO {REASON}',
	'SUPU-3': 'EXPEDITE',
	'SUPU-4': 'REVISED {REASON}',

	'SUPD-1': 'DUE TO {REASON}',

	'TXTU-1': '{TEXT}',
	'TXTU-2': '{TEXT}',
	'TXTU-3': '{TEXT}',
	'TXTU-4': '{TEXT}',
	'TXTU-5': '{TEXT}',

	'TXTD-1': '{TEXT}',
	'TXTD-2': '{TEXT}',

	'SYSU-1': 'ERROR {TEXT}',
	'SYSU-2': 'NEXT DATA AUTHORITY {CALLSIGN}', # arg is optional in doc-4444 spec
	'SYSU-3': 'MESSAGE NOT SUPPORTED BY THIS ATC UNIT',
	'SYSU-4': 'LOGICAL ACKNOWLEDGEMENT',
	'SYSU-5': 'USE OF LOGICAL ACKNOWLEDGEMENT PROHIBITED',
	'SYSU-6': 'LATENCY TIME VALUE {TEXT}', # should have its own type, but we're not handling this
	'SYSU-7': 'MESSAGE RECEIVED TOO LATE, RESEND MESSAGE OR CONTACT BY VOICE',

	'SYSD-1': 'ERROR {TEXT}',
	'SYSD-2': 'LOGICAL ACKNOWLEDGEMENT',
	'SYSD-3': 'NOT CURRENT DATA AUTHORITY',
	'SYSD-4': 'CURRENT DATA AUTHORITY',
	'SYSD-5': 'NOT AUTHORIZED NEXT DATA AUTHORITY {CALLSIGN} {CALLSIGN}', # one arg is optional in doc-4444 spec
	'SYSD-6': 'MESSAGE RECEIVED TOO LATE, RESEND MESSAGE OR CONTACT BY VOICE',
	'SYSD-7': 'AIRCRAFT CPDLC INHIBITED'
}

def CPDLC_element_display_text(msg_element, varFmt=None):
	elt_split = msg_element.split(' ', maxsplit=1)
	try:
		res = CPDLC_element_formats[elt_split[0]]
		expected_args = re.findall(r'\{(\w+)}', res)
		argstr = '' if len(elt_split) < 2 else elt_split[1] # to consume
		args = []
		if len(elt_split) > 1:
			skipped_allowing_spaces = None
			while expected_args and skipped_allowing_spaces is None: # take from left, stop if an arg. can contain spaces
				if expected_args.pop(0) in args_allowing_spaces:
					skipped_allowing_spaces = len(args) # index of arg. with spaces (to insert last)
				else:
					arg_split = argstr.split(' ', maxsplit=1)
					args.append(arg_split[0])
					argstr = '' if len(arg_split) == 1 else arg_split[1]
			while expected_args: # take from right, skipped_allowing_spaces is not None
				assert expected_args.pop(-1) not in args_allowing_spaces, 'There should not be more than one arg. allowing spaces in CPDLC element format'
				# first expected argument does NOT allow spaces
				arg_split = argstr.rsplit(' ', maxsplit=1)
				args.insert(skipped_allowing_spaces, arg_split[-1])
				argstr = '' if len(arg_split) == 1 else arg_split[0]
			if skipped_allowing_spaces is None:
				if argstr: # this should have been entirely consumed
					return '!!err ' + msg_element
			else: # remainder of argstr is the skipped argument
				args.insert(skipped_allowing_spaces, argstr)
		for arg in args:
			res = re.sub(r'\{\w+}', (arg if varFmt is None else varFmt % arg), res, count=1)
		return res
	except KeyError: # unknown msg elt ID
		return '!!err ' + msg_element
