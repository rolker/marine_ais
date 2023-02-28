# This message is a submessage of Ais and is not intended
# to be used separately.

# message 18
# Class B unit flag
# 0 = Class B SOTDMA unit
# 1 = Class B “CS” unit

bool cs_unit

# message 18
# Class B display flag
# 0 = No display available; not capable of displaying Message 12 and 14
# 1 = Equipped with integrated display displaying Message 12 and 14

bool display_equipped

# message 18
# Class B DSC flag
# 0 = Not equipped with DSC function
# 1 = Equipped with DSC function (dedicated or time-shared)

bool dsc_equipped

# message 18
# Class B band flag
# 0 = Capable of operating over the upper 525 kHz band of the marine band
# 1 = Capable of operating over the whole marine band
# (irrelevant if “Class B Message 22 flag” is 0)

bool whole_band

# message 18
# Class B Message 22 flag
# 0 = No frequency management via Message 22, operating on AIS 1, AIS 2
# only
# 1 = Frequency management via Message 22

bool message_22_frequency_management

# message 24
# Part number
# Identifier for the message part number;
# 0 for Part A
# 1 for Part B

uint8 part_number

# Vendor ID
# Unique identification of the Unit by a number as defined by the manufacturer
# (option; “@@@@@@@” = not available = default)
# 
# Manufacturer’s ID The Manufacturer’s ID bits indicate the manufacture’s mnemonic code consisting of three 6 bit ASCII characters(1)

string manufacturer_id 

# Unit Model Code
# The Unit Model Code bits indicate the binary coded series number of
# the model. The first model of the manufacture uses “1” and the
# number is incremented at the release of a new model. The code
# reverts to “1” after reaching to “15”. The “0” is not used

uint8 unit_model_code

# Unit Serial Number
# The Unit Serial Number bits indicate the manufacture traceable serial
# number. When the serial number is composed of numeric only, the
# binary coding should be used. If it includes figure(s), the manufacture
# can define the coding method. The coding method should be
# mentioned in the manual

uint32 unit_serial_number

