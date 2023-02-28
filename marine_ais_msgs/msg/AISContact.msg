# This is a higer level message for tracked AIS contacts.

# It combines the latest position report with previously seen
# static and voyage reports if available.

Header header 

# The AIS message type of the position report used to populate
# the position parts of this message
uint8 position_message_id 

# Sender id, often the MMSI number
uint32 id

# GeoPoseWithCovarariance is not part of the noetic
# distribution at this moment (2023-02-27).
# geographic_msgs/GeoPoseWithCovariance pose

geographic_msgs/GeoPose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (Lat, Lon, Alt, rotation about E (East) axis, rotation about N (North) axis, rotation about U (Up) axis)
float64[36] covariance

geometry_msgs/TwistWithCovariance twist

NavigationalStatus navigational_status

Static static_info

geometry_msgs/Polygon footprint

Voyage voyage
