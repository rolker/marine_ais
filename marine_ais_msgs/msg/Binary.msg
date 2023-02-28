# This message is a submessage of Ais and is not intended
# to be used separately.

# message 6,8,25,26
# Binary data
# Application identifier
#   Should be as described in § 2.1, Annex 5

# 16-bit application identifier (AI = DAC + FI), consisting of:
# – 10-bit designated area code (DAC) – based on the MID;
# – 6-bit function identifier (FI) – allows for 64 unique application specific messages.

uint16 designated_area_code
uint8 function_identifier

# message 25, 26
# Binary data flag
# 0 = unstructured binary data (no Application Identifier bits used)
# 1 = binary data coded as defined by using the
#     16-bit Application identifier

bool unstructured # true for unstructured, false if application id is used

# message 6,8,25,26
# Application data
#   Application specific data

# message 17
# Data
# Differential correction data (see below). If interrogated and differential
# correction service not available, the data field should remain empty (zero
# bits). This should be interpreted by the recipient as DGNSS data words set
# to zero

uint8[]  data

