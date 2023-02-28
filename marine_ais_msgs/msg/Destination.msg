# This message is a submessage of Ais and is not intended
# to be used separately.

# message 6,7,12,13,15,16,22,26
# Destination ID
# MMSI number of destination station

uint32 id


# message 6,7,12,13
# Sequence number
# 0-3; refer to § 5.3.1, Annex 2

uint8 sequence_number

# message 15
# - The parameter slot offset should be set to zero, if slot should autonomously be allocated by the responding station.
# An interrogating mobile station should always set the parameter “slot offset” to zero. Slot assignments for the reply
# to an interrogation should only be used by a base station. If a slot offset is given, it should be relative to the start slot
# of this transmission. A mobile station should be able to process a minimum slot offset of 10 slots. There should be
# the following four (4) possibilities to use this message:
# - One (1) station is interrogated one (1) message: The parameters destination ID1, message ID1.1 and slot offset 1.1
# should be defined. All other parameters should be omitted.
# - One (1) station is interrogated two (2) messages: The parameters destination ID1, message ID1.1, slot offset 1.1,
# message ID1.2, and slot offset 1.2 should be defined. The parameters destination ID2, message ID2.1, and slot offset
# 2.1 should be omitted. See § 3.3.7, Annex 2 for byte boundaries.
# – The first station and the second station are interrogated one (1) message each: The parameters destination ID1,
# message ID1.1, slot offset 1.1, destination ID2, message ID2.1, and slot offset 2.1 should be defined. The parameters
# message ID1.2 and slot offset 1.2 should be set to zero (0).
# – The first station is interrogated two (2) messages, and the second station is interrogated one (1) message:
# All parameters should be defined.

# message 15
# requested message type from interrogated station

uint8 requested_message_id

# message 15
# Response slot offset for requested messages from interogated station

# message 16
# Offset
# Offset from current slot to first assigned slot
# To assign a reporting rate for a station, the parameter increment should be set to zero. The parameter offset should
# then be interpreted as the number of reports in a time interval of 10 min.

uint8 slot_offset

# message 16
# Increment
# Increment to next assigned slot

uint16 increment

