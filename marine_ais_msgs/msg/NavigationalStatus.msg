# This message is a submessage of AISContact and is not intended
# to be used separately.

# messages 1,2,3,27
# Navigational status
# 0 = under way using engine, 1 = at anchor, 2 = not under command,
# 3 = restricted maneuverability, 4 = constrained by her draught, 5 = moored,
# 6 = aground, 7 = engaged in fishing, 8 = under way sailing, 9 = reserved for
# future amendment of navigational status for ships carrying DG, HS, or MP,
# or IMO hazard or pollutant category C, high speed craft (HSC),
# 10 = reserved for future amendment of navigational status for ships carrying
# dangerous goods (DG), harmful substances (HS) or marine pollutants (MP),
# or IMO hazard or pollutant category A, wing in ground (WIG);11 = powerdriven vessel towing astern (regional use),
# 12 = power-driven vessel pushing ahead or towing alongside (regional use);
# 13 = reserved for future use,
# 14 = AIS-SART (active), MOB-AIS, EPIRB-AIS
# 15 = undefined = default (also used by AIS-SART, MOB-AIS and EPIRBAIS under test)

uint8 NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE = 0
uint8 NAVIGATIONAL_STATUS_AT_ANCHOR = 1
uint8 NAVIGATIONAL_STATUS_NOT_UNDER_COMMAND = 2
uint8 NAVIGATIONAL_STATUS_RESTRICTED_MOBILITY = 3
uint8 NAVIGATIONAL_STATUS_DRAUGHT_CONSTRAINED = 4
uint8 NAVIGATIONAL_STATUS_MOORED = 5
uint8 NAVIGATIONAL_STATUS_AGROUND = 6
uint8 NAVIGATIONAL_STATUS_ENGAGED_IN_FISHING = 7
uint8 NAVIGATIONAL_STATUS_UNDER_WAY_SAILING = 8
uint8 NAVIGATIONAL_STATUS_TOWING_ASTERN = 11
uint8 NAVIGATIONAL_STATUS_PUSHING_AHEAD_OR_TOWING_ALONGSIDE = 12
uint8 NAVIGATIONAL_STATUS_SART_MOB_EPIRB = 14
uint8 NAVIGATIONAL_STATUS_UNDEFINED = 15

uint8 status

# messages 1,2,3
# Special manoeuvre indicator (2 bits)
# 0 = not available = default
# 1 = not engaged in special manoeuvre
# 2 = engaged in special manoeuvre
# (i.e. regional passing arrangement on Inland Waterway)
uint8 SPECIAL_MANOEUVRE_NOT_AVAILABLE = 0 # default
uint8 SPECIAL_MANOEUVRE_NOT_ENGANGED  = 1
uint8 SPECIAL_MANOEUVRE_ENGAGED       = 2

uint8 special_manoeuvre


