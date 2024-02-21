# Aids to Navigation specific data

# Type of aids-to navigation
# 0 = not available = default; refer to appropriate definition set up by IALA;

# 0 Default, Type of AtoN not specified
# 1 Reference point
# 2 RACON
# 3 Fixed structures off-shore, such as oil platforms, wind farms.
#   (NOTE 1 – This code should identify an obstruction that is fitted
#   with an AtoN AIS station)
# 4 Emergency Wreck Marking Buoy
#
# Fixed AtoN
#
# 5 Light, without sectors
# 6 Light, with sectors
# 7 Leading Light Front
# 8 Leading Light Rear
# 9 Beacon, Cardinal N
# 10 Beacon, Cardinal E
# 11 Beacon, Cardinal S
# 12 Beacon, Cardinal W
# 13 Beacon, Port hand
# 14 Beacon, Starboard hand
# 15 Beacon, Preferred Channel port hand
# 16 Beacon, Preferred Channel starboard hand
# 17 Beacon, Isolated danger
# 18 Beacon, Safe water
# 19 Beacon, Special mark
#
# Floating AtoN
#
# 20 Cardinal Mark N
# 21 Cardinal Mark E
# 22 Cardinal Mark S
# 23 Cardinal Mark W
# 24 Port hand Mark
# 25 Starboard hand Mark
# 26 Preferred Channel Port hand
# 27 Preferred Channel Starboard hand
# 28 Isolated danger
# 29 Safe Water
# 30 Special Mark
# 31 Light Vessel/LANBY/Rigs

uint8 type

# message 21
# Off-position indicator
# For floating AtoN, only: 0 = on position; 1 = off position.
# NOTE 1 – This flag should only be considered valid by receiving station,
# if the AtoN is a floating aid, and if time stamp is equal to or below 59.
# For floating AtoN the guard zone parameters should be set on installation

bool off_position

# message 21
# AtoN status
# Reserved for the indication of the AtoN status
# 00000000 = default

uint8 status

# Virtual AtoN flag
# 0 = default = real AtoN at indicated position;
# 1 = virtual AtoN, does not physically exist(2).

bool virtual_aton

