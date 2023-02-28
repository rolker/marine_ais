# Message 20: Data link management reservation block
#
# The parameter offset number should denote the offset from
# the slot in which Message 20 was received
# to the first slot to be reserved.
# The parameter number of slots should denote the number of consecutive
# slots to be reserved starting with the first reserved slot.
# This defines a reservation block.

# Offset number
# Reserved offset number; 0 = not available(1)

uint16 offset_number

# Number of slots
# Number of reserved consecutive slots: 1-15;
# 0 = not available(1)

uint8 number_of_slots

# Time-out
# Time-out value in minutes; 0 = not available(1)

duration timeout

# Increment
# Increment to repeat reservation block;
# 0 = one reservation block per frame(1)

uint16 increment

# (1) If interrogated and no data link management information is available, only Offset number 1, number of slots 1, timeout 1, and increment 1 should be sent. These fields should all be set to zero.
