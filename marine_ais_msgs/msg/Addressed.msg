# This message is a submessage of Ais and is not intended
# to be used separately.

Destination[] destinations

# message 6,12
# Retransmit flag
# Retransmit flag should be set upon retransmission:
# 0 = no retransmission = default; 1 = retransmitted

bool retransmitted


# message 22, 25, 26
# Addressed or broadcast message indicator
# 0 = broadcast geographical area message = default;
# 1 = addressed message (to individual station(s))
# 
# Above destinations array left empty if a broadcast message


