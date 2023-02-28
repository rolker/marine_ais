# This message is a submessage of Ais and is not intended
# to be used separately.

# message 5
# Estimated time of arrival; MMDDHHMM UTC
# Bits 19-16: month; 1-12; 0 = not available = default
# Bits 15-11: day; 1-31; 0 = not available = default
# Bits 10-6: hour; 0-23; 24 = not available = default
# Bits 5-0: minute; 0-59; 60 = not available = default
#For SAR aircraft, the use of this field may be decided by the responsible
# administration

time estimated_time_of_arrival


# message 5
# Destination 120 Maximum 20 characters using 6-bit ASCII;
# @@@@@@@@@@@@@@@@@@@@ = not available
# For SAR aircraft, the use of this field may be decided by the responsible
# administration

string destination

