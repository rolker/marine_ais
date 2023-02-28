# This message is a submessage of Ais and is not intended
# to be used separately.

# message 5
# AIS version indicator
# 0 = station compliant with Recommendation ITU-R M.1371-1
# 1 = station compliant with Recommendation ITU-R M.1371-3 (or later)
# 2 = station compliant with Recommendation ITU-R M.1371-5 (or later)
# 3 = station compliant with future editions 

uint8 ais_version

# message 5
# IMO number
# 0 = not available = default – Not applicable to SAR aircraft
# 0000000001-0000999999 not used
# 0001000000-0009999999 = valid IMO number;
# 0010000000-1073741823 = official flag state number.
uint32 imo_number

# message 5, 24B
# Call sign
# 7 x 6 bit ASCII characters, @@@@@@@ = not available = default.
# Craft associated with a parent vessel, should use “A” followed by the last
# 6 digits of the MMSI of the parent vessel. Examples of these craft include
# towed vessels, rescue boats, tenders, lifeboats and liferafts.

string callsign

# message 5,19,21
# Name
# Maximum 20 characters 6 bit ASCII, as defined in Table 47
# “@@@@@@@@@@@@@@@@@@@@” = not available = default.
# The Name should be as shown on the station radio license. For SAR aircraft,
# it should be set to “SAR AIRCRAFT NNNNNNN” where NNNNNNN
# equals the aircraft registration number.

# message 21
# Name of Aid-toNavigation Extension
# This parameter of up to 14 additional 6-bit-ASCII characters for a
# 2-slot message may be combined with the parameter “Name of Aid-toNavigation”
# at the end of that parameter, when more than 20 characters are
# needed for the name of the AtoN. This parameter should be omitted when no
# more than 20 characters for the name of the A-to-N are needed in total. Only
# the required number of characters should be transmitted, i.e. no @-character
# should be used

# message 24A
# Name of the MMSI-registered vessel.

string name

# message 5,19
# Type of ship and cargo type
# 0 = not available or no ship = default
# 1-99 = as defined in § 3.3.2
# 100-199 = reserved, for regional use
# 200-255 = reserved, for future use
# Not applicable to SAR aircraft
#
# 3.3.2 Type of ship
#
# Special craft
#
# 50 Pilot vessel
# 51 Search and rescue vessels
# 52 Tugs
# 53 Port tenders
# 54 Vessels with anti-pollution facilities or equipment
# 55 Law enforcement vessels
# 56 Spare - for assignments to local vessels
# 57 Spare - for assignments to local vessels
# 58 Medical transports (as defined in the 1949 Geneva Conventions and Additional Protocols)
# 59 Ships and aircraft of States not parties to an armed conflict
#
# Other ships
#
# First digit
# 1 – Reserved for future use
# 2 – WIG
# 3 – Vessel
# 4 – HSC
# 6 – Passenger ships
# 7 – Cargo ships
# 8 – Tanker(s) 
# 9 – Other types of ship
#
# Second digit (not for first digits 3 or 5)
#
# 0 – All ships of this type
# 1 – Carrying DG, HS, or MP, IMO hazard or pollutant category X
# 2 – Carrying DG, HS, or MP, IMO hazard or pollutant category Y
# 3 – Carrying DG, HS, or MP, IMO hazard or pollutant category Z
# 4 – Carrying DG, HS, or MP, IMO hazard or pollutant category OS
# 5 – Reserved for future use
# 6 – Reserved for future use
# 7 – Reserved for future use
# 8 – Reserved for future use
# 9 – No additional information

# Second digit (for first digit 3)
# 0 – Fishing
# 1 – Towing
# 2 – Towing and length of the tow exceeds 200 m or breadth exceeds 25 m
# 3 – Engaged in dredging or underwater operations
# 4 – Engaged in diving operations
# 5 – Engaged in military operations
# 6 – Sailing
# 7 – Pleasure craft
# 8 – Reserved for future use
# 9 – Reserved for future use

uint8 ship_and_cargo_type

# message 5, 19, 24B
# Reference point for reported position.
# Also indicates the dimension of ship (m)
# For SAR aircraft, the use of this field may be decided by the responsible
# administration. If used it should indicate the maximum dimensions of the
# craft. As default should A = B = C = D be set to “0”

# A 0-511 meters from ref to bow
# B 0-511 meters from ref to stern
# C 0-63 meters from ref to port (left)
# D 0-63 meters from ref to starboard (right)

# The dimension A should be in the direction of the transmitted
# heading information (bow)
# Reference point of reported position not available, but
# dimensions of ship are available: A = C = 0 and B != 0 and D != 0.
# Neither reference point of reported position nor dimensions of
# ship available; A = B = C = D = 0 (= default).

# message 21
# – For fixed Aids-to-Navigation, virtual AtoN, and for off-shore structures,
#   the orientation established by the dimension A should point to true north.
# – For floating aids larger than 2 m * 2 m the dimensions of the AtoN should
#   always be given approximated to a circle, i.e. the dimensions should always
#   be as follows A = B = C = D ≠ 0. (This is due to the fact that the
#   orientation of the floating Aid to Navigation is not transmitted. The
#   reference point for reported position is in the centre of the circle.)
# – A = B = C = D = 1 should indicate objects (fixed or floating) smaller than
#   or equal to 2 m * 2 m. (The reference point for reported position is in the
#   centre of the circle.)
# – Floating off shore structures that are not fixed, such as rigs, should be
#   considered as Code 31 type from Table 74 AtoN. These structures should have
#   their “Dimension/reference for position” parameter as determined above in
#   Note (1).
# For fixed off shore structures, Code 3 type from Table 74, should have their
# “Dimension/reference for position” parameter as determined above in Note (1).
# Hence, all off shore AtoN and structures have the dimension determined in
# the same manner and the actual dimensions are contained in Message 21.
#
# When transmitting virtual AtoN information, i.e. the virtual/pseudo AtoN
# Target Flag is set to one (1), the dimensions should be set to A=B=C=D=0
# (default). This should also be the case, when transmitting “reference
# point” information

uint16 reference_to_bow_distance       # A
uint16 reference_to_stern_distance     # B
uint8  reference_to_port_distance      # C
uint8  reference_to_starboard_distance # D

# message 5
# Maximum present static draught
# In 1/10 m, 255 = draught 25.5 m or greater, 0 = not available = default;
# in accordance with IMO Resolution A.851
# Not applicable to SAR aircraft, should be set to 0

float32 static_draught # meters

# message 5,9,19
# DTE
# Data terminal equipment (DTE) ready
# (0 = available, 1 = not available = default)

bool dte_ready # true if available, false otherwise
