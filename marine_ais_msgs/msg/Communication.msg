# This message is a submessage of Ais and is not intended
# to be used separately.

# messages 9,18,26
# Communication state selector flag
# 0 = SOTDMA communication state follows
# 1 = ITDMA communication state follows

# message 1,2,3
# Communication state (19 bits)
# Message 1 and 2: SOTDMA communication state as described in § 3.3.7.2.2, Annex 2
# Message 3: ITDMA communication state as described in § 3.3.7.3.2, Annex 2

uint8 COMMUNICATION_STATE_SOTDMA = 0
uint8 COMMUNICATION_STATE_ITDMA = 1

uint8 state

# messages 1,2,3,9,18,26
# Sync state
# 0 UTC direct (see § 3.1.1.1)
# 1 UTC indirect (see § 3.1.1.2)
# 2 Station is synchronized to a base station (base direct)
# (see § 3.1.1.3)
# 3 Station is synchronized to another station based on the highest
# number of received stations or to another mobile station,
# which is directly synchronized to a base station
# (see § 3.1.1.3 and § 3.1.1.4)

uint8 COMMUNICATION_SYNC_STATE_UTC_DIRECT   = 0
uint8 COMMUNICATION_SYNC_STATE_UTC_INDIRECT = 1
uint8 COMMUNICATION_SYNC_STATE_BASE_DIRECT  = 2
uint8 COMMUNICATION_SYNC_STATE_INDIRECT     = 3

uint8 sync_state

# messages 1,2,4,9,11,18
# 3.3.7.2.2 Self-organizing time division multiple access communication state
# The communication state provides the following functions:
# – it contains information used by the slot allocation algorithm in the SOTDMA concept;
# – it also indicates the synchronization state.

# The SOTDMA communication state should apply only to the slot in the channel where the relevant
# transmission occurs.

# Slot time-out
# Specifies frames remaining until a new slot is selected
# 0 means that this was the last transmission in this slot
# 1-7 means that 1 to 7 frames respectively are left until slot change

uint8 sotdma_slot_timeout

# Sub message
# The sub message depends on the current value in slot time-out

# Slot time-out 3, 5, 7:  Received stations
#   Number of other stations (not own station)
#   which the station currently is receiving (between 0 and 16383).
# Slot time-out 2, 4, 6: Slot number
#   Slot number used for this transmission (between 0 and 2 249).
# Slot time-out 1: UTC hour and minute
#   If the station has access to UTC, the hour and minute should be
#   indicated in this sub message. Hour (0-23) should be coded in bits
#   13 to 9 of the sub message (bit 13 is MSB). Minute (0-59) should be
#   coded in bit 8 to 2 (bit 8 is MSB). Bit 1 and bit 0 are not used.
# Slot time-out 0: Slot offset
#   If the slot time-out value is 0 (zero) then the slot offset should
#   indicate the offset to the slot in which transmission will occur
#   during the next frame. If the slot offset is zero, the slot should be
#   de-allocated after transmission.
uint16 sotdma_received_stations
uint16 sotdma_slot_number
uint8  sotdma_utc_hour
uint8  sotdma_utc_minute
uint16 sotdma_slot_offset


# message 3,9,18
# 3.3.7.3.2 Incremental time division multiple access communication state
# The communication state provides the following functions:
# – it contains information used by the slot allocation algorithm in the ITDMA concept;
# – it also indicates the synchronization state.

# The ITDMA communication state should apply only to the slot in the channel where the relevant
# transmission occurs.

# Slot increment
# Offset to next slot to be used, or zero (0) if no more transmissions

uint16 itdma_slot_increment

# Number of slots
# Number of consecutive slots to allocate.
# 0 = 1 slot,
# 1 = 2 slots,
# 2 = 3 slots,
# 3 = 4 slots,
# 4 = 5 slots,
# 5 = 1 slot; offset = slot increment + 8 192,
# 6 = 2 slots; offset = slot increment + 8 192,
# 7 = 3 slots; offset = slot increment + 8 192.
# Use of 5 to 7 removes the need for RATDMA broadcast for scheduled
# transmissions up to 6 min intervals

uint8 itdma_number_of_slots

# Keep flag
# Set to TRUE = 1 if the slot remains allocated for one additional frame

bool itdma_keep
