# Channel A and B
# Channel number of 25 kHz simplex or simplex use of 25 kHz duplex in
# accordance with Recommendation ITU-R M.1084.

uint16 channel_a
uint16 channel_b


# Power
# 0 = high (default), 1 = low

bool high_power


# Channel bandwidth
# 0 = default (as specified by channel number);
# 1 = spare (formerly 12.5 kHz bandwidth in Recommendation
# ITU-R M.1371-1 now obsolete)

bool channel_a_bandwidth_spare
bool channel_b_bandwidth_spare

# Transitional zone size
# The transitional zone size in nautical miles should be calculated by adding 1
# to this parameter value. The default parameter value should be 4, which
# translates to 5 nautical miles; see § 4.1.5, Annex 2

# (Value reported here has 1 already added) 

uint8 transitional_zone_size # nautical miles 

# message 22,23
# Longitude 1 18 Longitude of area to which the group assignment applies; upper right corner
# (north-east); in 1/10 min
# (180, East = positive, West = negative)
# Latitude 1 17 Latitude of area to which the group assignment applies; upper right corner
# (north-east); in 1/10 min
# (90, North = positive, South = negative)
# Longitude 2 18 Longitude of area to which the group assignment applies; lower left corner
# (south-west); in 1/10 min
# (180, East = positive, West = negative)
# Latitude 2 17 Latitude of area to which the group assignment applies; lower left corner
# (south-west); in 1/10 min
# (90, North = positive, South = negative)

geographic_msgs/BoundingBox area

# message 22, 23
# Tx/Rx mode
# 0 = Tx A/Tx B, Rx A/Rx B (default)
# 1 = Tx A, Rx A/Rx B
# 2 = Tx B, Rx A/Rx B
# 3-15: not used
# When the dual channel transmission is suspended by Tx/Rx mode command
# 1 or 2, the required reporting interval should be maintained using the
# remaining transmission channel

uint8 tx_rx_mode

# message 23
# Station type
# 0 = all types of mobiles (default); 1 = Class A mobile stations only;
# 2 = all types of Class B mobile stations; 3 = SAR airborne mobile station;
# 4 = Class B “SO” mobile stations only; 5 = Class B “CS” shipborne mobile
# station only;
# 6 = inland waterways; 7 to 9 = regional use; 10 = base station coverage area
# (see Message 4 and Message 27); 11 to 15 = for future use

uint8 station_type

# message 23
# Reporting interval
# This parameter commands the respective stations to the reporting interval
# given below
#
# 0 As given by the autonomous mode
# 1 10 min
# 2 6 min
# 3 3 min
# 4 1 min
# 5 30 s
# 6 15 s
# 7 10 s
# 8 5 s
# 9 Next shorter reporting interval
# (only applicable if in autonomous mode)
# 10 Next longer reporting interval
# (only applicable if in autonomous mode)
# 11 2 s (not applicable to the Class B “CS”
# and Class B “SO”)
# 12-15 Reserved for future use

uint8 reporting_interval
