Header header            # The frame_id
                         # refers to the AIS receiver antenna.


uint32 id           # Unique identifier such as MMSI number


uint32 imo_number

string callsign
string name

uint8 ship_and_cargo_type

# Reference point for reported position.
# Also indicates the dimension of ship (m)

# A  9 bits, bit 21 - bit 29, 0-511 meters from ref to bow
# B  9 bits, bit 12 - bit 20, 0-511 meters from ref to stern
# C  6 bits, bit 6  - bit 11, 0-63 meters from ref to port (left)
# D  6 bits, bit 0  - bit 5,  0-63 meters from ref to starboard (right)

# The dimension A should be in the direction of the transmitted
# heading information (bow)
# Reference point of reported position not available, but
# dimensions of ship are available: A = C = 0 and B 0 and D 0.
# Neither reference point of reported position nor dimensions of
# ship available; A = B = C = D = 0 (= default).
# For use in the message table, A = most significant field,
# D = least significant field.

# reference point and dimensions should converted to an outline
# following REP103 conventions. So A is positive x, B is negative x,
# C is positive y and D is negative y.
# The outline should look like a simple ship, with the bow being
# "pointy". 
geometry_msgs/Polygon outline

# Type of electronic position fixing device (4 bits)
# 0 = undefined (default)
# 1 = GPS
# 2 = GLONASS
# 3 = combined GPS/GLONASS
# 4 = Loran-C
# 5 = Chayka
# 6 = integrated navigation system
# 7 = surveyed
# 8 = Galileo,
# 9-14 = not used
# 15 = internal GNSS

uint8 position_fix_device_type

time estimated_time_of_arrival

# Maximum present static draught (8 bits)
# In 1/10 m, 255 = draught 25.5 m or greater, 0 = not available = default;
# in accordance with IMO Resolution A.851
# Not applicable to SAR aircraft, should be set to 0

float32 static_draught # meters

string destination

# DTE (1 bit)
# Data terminal equipment (DTE) ready (0 = available, 1 = not available =
# default)

bool dte_ready # true if available, false otherwise
