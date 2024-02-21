# Combination of AIS messages relevant to marine robotics.
# The use of a single combined message allows AIS data to be 
# sent using a single message topic.

# This is a lower-level ROS message designed to represent individual
# AIS messages.

# The timestamp in the header is the time the data 
# was received by the ROS driver.
# The frame_id refers to the AIS receiver antenna location.

std_msgs/Header header 

# The following are the defined AIS messages.
# Some data members are only valid for some message ids.

# 1 = Scheduled position report; (Class A shipborne mobile equipment)
# 2 = Assigned scheduled position report; (Class A shipborne mobile equipment)
# 3 = Special position report, response to interrogation; (Class A shipborne mobile equipment)
# 4 = Position, UTC, date and current slot number of base station
# 5 = Scheduled static and voyage related vessel data report; (Class A shipborne mobile equipment)
# 6 = Binary data for addressed communication
# 7 = Acknowledgement of received addressed binary data
# 8 = Binary data for broadcast communication
# 9 = Position report for airborne stations involved in SAR operations, only
# 10 = Request UTC and date
# 11 = Current UTC and date if available
# 12 = Safety related data for addressed communication
# 13 = Acknowledgement of received addressed safety related message
# 14 = Safety related data for broadcast communication
# 15 = Request for a specific message type (can result in multiple responses from one or several stations)
# 16 = Assignment of a specific report behaviour by competent authority using a Base station
# 17 = DGNSS corrections provided by a base station
# 18 = Standard position report for Class B shipborne mobile equipment to be used instead of Messages 1, 2, 3
# 19 = No longer required; Extended position report for Class B shipborne mobile equipment; contains additional static information
# 20 = Reserve slots for Base station(s)
# 21 = Position and status report for aids-to-navigation
# 22 = Management of channels and transceiver modes by a Base station
# 23 = Assignment of a specific report behaviour by competent authority using a Base station to a specific group of mobiles
# 24 = Additional data assigned to an MMSI
#      Part A: Name
#      Part B: Static Data
# 25 = Short unscheduled binary data transmission (Broadcast or addressed)
# 26 = Scheduled binary data transmission (Broadcast or addressed)
# 27 = Class A and Class B “SO” shipborne mobile equipment outside base station coverage
uint8 POSITION_REPORT_SCHEDULED        =  1   
uint8 POSITION_REPORT_ASSIGNED         =  2
uint8 POSITION_REPORT_SPECIAL          =  3
uint8 BASE_STATION_REPORT              =  4 
uint8 STATIC_AND_VOYAGE_RELATED_DATA   =  5
uint8 BINARY_ADDRESSED_MESSAGE         =  6
uint8 BINARY_ACKNOWLEDGEMENT_MESSAGE   =  7
uint8 BINARY_BROADCAST_MESSAGE         =  8
uint8 POSITION_REPORT_SAR              =  9
uint8 UTC_DATE_INQUIRY                 = 10
uint8 UTC_DATE_RESPONSE                = 11
uint8 ADDRESSED_SAFETY_RELATED_MESSAGE = 12
uint8 SAFETY_RELATED_ACKNOWLEDGEMENT   = 13
uint8 SAFETY_RELATED_BROADCAST_MESSAGE = 14
uint8 INTEROGATION                     = 15
uint8 ASSIGNMENT_MODE_COMMAND          = 16
uint8 DGNSS_BROADCAST_BINARY_MESSAGE   = 17
uint8 STANDARD_CLASS_B_POSITION_REPORT = 18
uint8 EXTENDED_CLASS_B_POSITION_REPORT = 19
uint8 DATA_LINK_MANAGEMENT_MESSAGE     = 20
uint8 AIDS_TO_NAVIGATION_REPORT        = 21
uint8 CHANNEL_MANAGEMENT               = 22
uint8 GROUP_ASSIGNMENT_COMMAND         = 23
uint8 STATIC_DATA_REPORT               = 24
uint8 SINGLE_SLOT_BINARY_MESSAGE       = 25
uint8 MULTIPLE_SLOT_BINARY_MESSAGE     = 26
uint8 POSITION_REPORT_LONG_RANGE       = 27

uint8  message_id        # Identifier for this Message

# all messages
# Used by the repeater to indicate how many times
# a message has been repeated. See § 4.6.1, Annex 2;
# 0-3; 0 = default; 3 = do not repeat any more

uint8  repeat_indicator  

# messages 1,2,3,4,5,9,11,18,19
# User ID
# Unique identifier such as MMSI number

# message 6,7,8,12,13,14,15,16,17,20,22,23
# Source station ID
# MMSI number of base station

# message 21
# ID
# MMSI number, (see Article 19 of the RR and Recommendation ITU-R M.585)

uint32 id

Navigation navigation

Static static_info

Voyage voyage

Addressed addressed

Binary binary

# message 18, 24
ClassB class_b

# message 20
DataLinkReservationBlock[] data_link_reservation_blocks

# message 21
AidsToNavigation aton

Communication communication

# message 22
ChannelManagement channel_management


# messages 4,11
builtin_interfaces/Time utc_time

# message 4,11
# Transmission control for long range broadcast message
# 0 = default – Class-A AIS station stops transmission of Message 27 within
# an AIS base station coverage area.
# 1 = Request Class-A station to transmit Message 27 within an AIS base
# station coverage area.
# Base station coverage area should be defined by Message 23; If Message 23
# is not received, the AIS station which is allowed to transmit on CH75 and 76
# (see 3.2, Annex 4) should ignore this bit and transmit Message 27.

bool long_range_transmission_control

# message 14
# Safety related text
# 6-bit ASCII as defined in Table 47
# The AIS-SART should use Message 14, and the safety related text should be:
# 1) For the active SART, the text should be “SART ACTIVE”.
# 2) For the SART test mode, the text should be “SART TEST”.
# 3) For the active MOB, the text should be “MOB ACTIVE”.
# 4) For the MOB test mode, the text should be “MOB TEST”.
# 5) For the active EPIRB, the text should be “EPIRB ACTIVE”.
# 6) For the EPIRB test mode, the text should be “EPIRB TEST”.

string safety_related_text

# Metadata
#
# Channel reported in nmea sentence (A or B)
string channel
# Reassembled nmea payload string (optional)
string nmea_payload
