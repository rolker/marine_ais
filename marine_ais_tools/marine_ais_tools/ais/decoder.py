#!/usr/bin/env python3

# Base on: Recommendation  ITU-R  M.1371-5 (02/2014)

import datetime

def decodePayload(payload, channel, nmea):
    bits = []
    for c in payload:
        nibble = ord(c)-48
        if nibble > 40:
            nibble -= 8
        mask = 1 << 5
        while mask:
            bits.append(bool(nibble&mask))
            mask = mask >> 1
    #print len(bits),'bits'

    message = {}
    message['channel'] = channel
    message['nmea_payload'] = payload
    message['nmea'] = nmea

    message_id = None
    for field in ais_fields_table:
        for entry in ais_fields_table[field]:
            details = ais_fields_table[field][entry]
            process = False
            if entry is None:
                process = True
            elif message_id in entry:
                process = True
            if process:
                if 'size' in details:
                    subbits = bits[details['start_bit']:details['start_bit']+details['size']]
                else:
                    subbits = bits[details['start_bit']:]
                if not 'size' in details or len(subbits) == details['size']:
                    if details['type'] == 'int':
                        message[field] = decodeSignedInt(subbits)
                    elif details['type'] == 'uint':
                        message[field] = decodeUnsignedInt(subbits)
                    elif details['type'] == 'bits':
                        message[field] = subbits
                    elif details['type'] == 'string':
                        message[field] = decodeString(subbits)
                    if 'post' in details:
                        details['post'](message)
        if field == 'message_id':
            message_id = message['message_id']
        if message_id is not None and message_id == 24:
            if 'part_number' in message:
                if message['part_number'] == 0:
                    message_id = '24A'
                elif message['part_number'] == 1:
                    message_id = '24B'
    
    return message

def decodeUnsignedInt(bits):
    ret = 0
    for b in bits:
        ret = ret << 1
        if b:
            ret = ret | 1
    return ret

def decodeSignedInt(bits):
    if not bits[0]: # positive
        return decodeUnsignedInt(bits[1:])
    # negative so two's complement
    ret = 0
    for b in bits:
        ret = ret << 1
        if not b:
            ret = ret | 1
    ret += 1
    return -ret

def decodeString(bits, strip=True):
    ret = ''
    for i in range(0,len(bits),6):
        c = decodeChar(bits[i:])
        if c == '@':
            break
        ret += c
    if strip:
        return ret.rstrip()
    return ret

def decodeChar(bits):
    code = decodeUnsignedInt(bits[:6])
    if code < 32:
        code += 64
    return chr(code)

def decodeData(bits):
    ret = []
    i = 0
    while i < len(bits):
        if i+8 < len(bits):
            byte = bits[i:i+8]
        else:
            byte = bits[i:]
            while len(byte) < 8:
                byte.append(False)
        ret.append(decodeUnsignedInt(byte))
        i+=8
        
    return ret


def processRateOfTurn(message):
    rot = message['rate_of_turn']
    if rot == -128:
        message['rate_of_turn_status'] = rot
        rot = None
    elif rot == -127:
        message['rate_of_turn_status'] = rot
        rot = None
        message['rate_of_turn_note'] = 'turning left at more than 5 deg per 30 sec'
    elif rot == 127:
        message['rate_of_turn_status'] = rot
        rot = None
        message['rate_of_turn_note'] = 'turning right at more than 5 deg per 30 sec'
    else:
        rot = pow(rot/4.733,2.0)
    message['rate_of_turn'] = rot

def processSOG(message):
    sog = message['sog']
    if sog == 1023:
        sog = None
    elif sog == 1022:
        sog = None
        message['sog_note'] = '102.2 knots or higher'
    else:
        sog = sog/10.0
    message['sog'] = sog

def processCOG(message):
    if message['cog'] >= 3600:
        message['cog'] = None
    else:
        message['cog'] /= 10.0


def processLongitude(message):
    if message['longitude'] == 0x6791AC0:
        message['longitude'] = None
    else:
        message['longitude']/=600000.0

def processLatitude(message):
    if message['latitude'] == 0x3412140:
        message['latitude'] = None
    else:
        message['latitude'] /= 600000.0

def processHeading(message):
    if message['true_heading'] == 511:
        message['true_heading'] = None

def processTimeStamp(message):
    return
    ts = message['time_stamp']
    if ts == 60:
        ts = None
    elif ts == 61:
        ts = None
        message['time_stamp_note'] = 'positioning system in manual input mode'
    elif ts == 62:
        ts = None
        message['time_stamp_note'] = 'positioning system in estimed mode (dead reckoning)'
    elif ts == 63:
        ts = None
        message['time_stamp_note'] = 'positioning system is inoperative'
    message['time_stamp'] = ts

def processCommunicationState(message):
    bits = message['communication_state']
    state = {}
    if message['message_id'] in (1,2,4,11) or (message['message_id'] in (9,25) and message['communication_state_selector_flag'] == 0):
        state['sotdma_sync_state'] = decodeUnsignedInt(bits[:2])
        state['sotdma_slot_timeout'] = decodeUnsignedInt(bits[2:5])
        if state['sotdma_slot_timeout'] in (3, 5, 7):
            state['sotdma_received_stations'] = decodeUnsignedInt(bits[5:19])
        if state['sotdma_slot_timeout'] in (2, 4, 6):
            state['sotdma_slot_number'] = decodeUnsignedInt(bits[5:19])
        if state['sotdma_slot_timeout'] == 1:
            state['sotdma_utc_hour'] = decodeUnsignedInt(bits[5:10])
            state['sotdma_utc_minute'] = decodeUnsignedInt(bits[10:17])
        if state['sotdma_slot_timeout'] == 0:
            state['sotdma_slot_offset'] = decodeUnsignedInt(bits[5:19])

    if message['message_id'] in (3,)or (message['message_id'] in (9,25) and message['communication_state_selector_flag'] == 1):
        state['itdma_sync_state'] = decodeUnsignedInt(bits[:2])
        state['itdma_slot_increment'] = decodeUnsignedInt(bits[2:15])
        state['itdma_number_of_slots'] = decodeUnsignedInt(bits[15:18])
        state['itdma_keep_flag'] = decodeUnsignedInt(bits[18:19])
    message['communication_state'] = state

def processUTCTime(message):
    bits = message['utc_time']
    year = decodeUnsignedInt(bits[0:14])
    month = decodeUnsignedInt(bits[14:18])
    day = decodeUnsignedInt(bits[18:23])
    hour = decodeUnsignedInt(bits[23:28])
    minute = decodeUnsignedInt(bits[28:34])
    second = decodeUnsignedInt(bits[34:40])
    if year == 0 or month == 0 or day == 0 or hour == 24 or minute == 60 or second == 60:
        message['utc_time'] = None
        return
    try:
        message['utc_time'] = datetime.datetime(year, month, day, hour, minute, second, tzinfo=datetime.timezone.utc)
    except: 
        message['utc_time'] = None

def processDraught(message):
    d = message['maximum_present_static_draught']
    if d == 0:
        d = None
    else:
        if d == 255:
            message['maximum_present_static_draught_note'] = '25.5 m or greater'
        d /= 10.0
    message['maximum_present_static_draught'] = d

def processBinaryData(message):
    bits = message['binary_data']
    if message['message_id'] == 17:
        message['binary_data'] = {'differential_correction_data': decodeData(bits)}
        return
    if message['message_id'] in (25,26):
        if message['destination_indicator'] == 1:
            bits = bits[32:]
        if message['message_id'] == 26:
            comms_bits = bits[-20:]
            message['communication_state_selector_flag'] = decodeUnsignedInt(comms_bits[:1])
            message['communication_state'] = comms_bits[1:]
            processCommunicationState(message)
            bits = bits[:-20]
    if 'binary_data_flag' in message and message['binary_data_flag'] == 0:
        message['binary_data'] = {'unstructured_data': decodeData(bits)}
        return
    aid = {}
    aid['designated_area_code'] = decodeUnsignedInt(bits[0:10])
    aid['function_identifier'] = decodeUnsignedInt(bits[10:16])
    data = decodeData(bits[16:])
    message['binary_data'] = {'application_identifier': aid, 'application_data':data}

def processAltitude(message):
    if message['altitude'] == 4095:
        message['altitude'] = None
    elif message['altitude'] == 4094:
        message['altitude_note'] = '4096 m or higher'

def processSOGHighSpeed(message):
    sog = message['sog']
    if sog == 1023:
        sog = None
    elif sog == 1022:
        sog = None
        message['sog_note'] = '1022 knots or higher'
    message['sog'] = sog

def processLongitudeLowRes(message):
    label = 'longitude'
    if message['message_id'] in(22, 23):
        if 'longitude_2' in message:
            label = 'longitude_2'
        else:
            label = 'longitude_1'
    if message[label]== 181*600:
        message[label] = None
    else:
        message[label]/=600.0

def processLatitudeLowRes(message):
    label = 'latitude'
    if message['message_id'] in (22,23):
        if 'latitude_2' in message:
            label = 'latitude_2'
        else:
            label = 'latitude_1'
    if message[label] == 91*600:
        message[label] = None
    else:
        message[label] /= 600.0

def processTransitionalZoneSize(message):
    message['transitional_zone_size'] += 1

def processAddressedOrBroadcast(message):
    if message['message_id'] == 22:
        if message['addressed_or_broadcast_indicator'] == 0: #broadcast
            message.pop('destination_id1',None)
            message.pop('destination_id2',None)
        else: # addressed
            message.pop('longitude_1',None)
            message.pop('latitude_1',None)
            message.pop('longitude_2',None)
            message.pop('latitude_2',None)

def processVendorID(message):
    bits = message['vendor_id']
    id = {}
    id['manufacturers_id'] = decodeString(bits[:18])
    id['unit_model_code'] = decodeUnsignedInt(bits[18:22])
    id['unit_serial_number'] = decodeUnsignedInt(bits[22:42])
    message['vendor_id'] = id

def processDestinationID(message):
    if message['message_id'] == 25:
        if message['destination_indicator'] == 0:
            message.pop('destination_id', None)

def processSOGLowRes(message):
    if message['sog'] == 63:
        message['sog'] = None

def processCOGLowRes(message):
    if message['cog'] >= 360:
        message['cog'] = None

ais_fields_table = {
    'message_id':          {None:   {'start_bit':0,   'size':6,  'type':'uint'}},
    'repeat_indicator':    {None:   {'start_bit':6,   'size':2,  'type':'uint'}},
    'id':                  {None:   {'start_bit':8,   'size':30, 'type':'uint'}},
    'navigational_status': {(1,2,3):{'start_bit':38,  'size':4,  'type':'uint'},
                            (27,):  {'start_bit':40,  'size':4,  'type':'uint'}},
    'part_number':         {(24,):  {'start_bit':38,   'size':2,  'type':'uint'}},
    'destination_indicator':
                           {(25,26):{'start_bit':38,   'size':1,  'type':'uint'}},
    'binary_data_flag':
                           {(25,26):{'start_bit':39,   'size':1,  'type':'uint'}},
    'rate_of_turn':        {(1,2,3):{'start_bit':42,  'size':8,  'type':'int',  'post':processRateOfTurn}},
    'sog':                 {(1,2,3):{'start_bit':50,  'size':10, 'type':'uint', 'post':processSOG},
                            (9,):   {'start_bit':50,  'size':10, 'type':'uint', 'post':processSOGHighSpeed},
                            (18,19):{'start_bit':46,  'size':10, 'type':'uint', 'post':processSOG},
                            (27,):  {'start_bit':79,  'size':6, 'type':'uint',  'post':processSOGLowRes},},
    'position_accuracy':   {(1,2,3,9):
                                    {'start_bit':60,  'size':1,  'type':'uint'},
                            (4,11): {'start_bit':78,  'size':1,  'type':'uint'},
                            (18,19):{'start_bit':56,  'size':1,  'type':'uint'},
                            (21,):  {'start_bit':163, 'size':1,  'type':'uint'},
                            (27,):  {'start_bit':38,  'size':1,  'type':'uint'}},
    'longitude':           {(1,2,3,9):
                                    {'start_bit':61,  'size':28, 'type':'int',  'post':processLongitude},
                            (4,11): {'start_bit':79,  'size':28, 'type':'int',  'post':processLongitude},
                            (17,):  {'start_bit':40,  'size':18, 'type':'int',  'post':processLongitudeLowRes},
                            (18,19):{'start_bit':57,  'size':28, 'type':'int',  'post':processLongitude},
                            (21,):  {'start_bit':164, 'size':28, 'type':'int',  'post':processLongitude},
                            (27,):  {'start_bit':44,  'size':18, 'type':'int',  'post':processLongitudeLowRes},},
    'latitude':            {(1,2,3,9):
                                    {'start_bit':89,  'size':27, 'type':'int',  'post':processLatitude},
                            (4,11): {'start_bit':107, 'size':27, 'type':'int',  'post':processLatitude},
                            (17,):  {'start_bit':58,  'size':17, 'type':'int',  'post':processLatitudeLowRes},
                            (18,19):{'start_bit':85,  'size':27, 'type':'int',  'post':processLatitude},
                            (21,):  {'start_bit':192, 'size':27, 'type':'int',  'post':processLatitude},
                            (27,):  {'start_bit':62,  'size':17, 'type':'int',  'post':processLatitudeLowRes}},
    'cog':                 {(1,2,3,9):
                                    {'start_bit':116, 'size':12, 'type':'uint', 'post':processCOG},
                            (18,19):{'start_bit':112, 'size':12, 'type':'uint', 'post':processCOG},
                            (27,)  :{'start_bit':85,  'size':9,  'type':'uint', 'post':processCOGLowRes}},
    'true_heading':        {(1,2,3):{'start_bit':128, 'size':9,  'type':'uint', 'post':processHeading},
                            (18,19):{'start_bit':124, 'size':9,  'type':'uint', 'post':processHeading}},
    'time_stamp':          {(1,2,3):{'start_bit':137, 'size':6,  'type':'uint', 'post':processTimeStamp},
                            (9,):   {'start_bit':128, 'size':6,  'type':'uint', 'post':processTimeStamp},
                            (18,19):{'start_bit':133, 'size':6,  'type':'uint', 'post':processTimeStamp},
                            (21,)  :{'start_bit':257, 'size':6,  'type':'uint', 'post':processTimeStamp}},
    'special_manoeuvre_indicator':
                           {(1,2,3):{'start_bit':143, 'size':2,  'type':'uint'}},
    'raim_flag':           {(1,2,3,4,11):
                                    {'start_bit':148, 'size':1,  'type':'uint'},
                            (9,18): {'start_bit':147, 'size':1,  'type':'uint'},
                            (19,):  {'start_bit':305, 'size':1,  'type':'uint'},
                            (21,):  {'start_bit':274, 'size':1,  'type':'uint'},
                            (27,):  {'start_bit':39,  'size':1,  'type':'uint'}},
    'communication_state_selector_flag':
                           {(9,18):   {'start_bit':148, 'size':1,  'type':'uint'}},
    'communication_state': {(1,2,3,4,9,11,18):
                                    {'start_bit':149, 'size':19, 'type':'bits', 'post':processCommunicationState}},
    'utc_time':            {(4,11): {'start_bit':38,  'size':40, 'type':'bits', 'post':processUTCTime}},
    'position_fixing_device_type':
                           {(4,11): {'start_bit':134, 'size':4,  'type':'uint'},
                            (5,):   {'start_bit':270, 'size':4,  'type':'uint'},
                            (19,):  {'start_bit':301, 'size':4,  'type':'uint'},
                            (21,):  {'start_bit':253, 'size':4,  'type':'uint'},
                            ('24B',):
                                    {'start_bit':142, 'size':4,  'type':'uint'}},
    'long_range_transmission_control':
                           {(4,11): {'start_bit':138, 'size':1,  'type':'uint'}},
    'ais_version':         {(5,):   {'start_bit':38,  'size':2,  'type':'uint'}},
    'imo_number':          {(5,):   {'start_bit':40,  'size':30, 'type':'uint'}},
    'call_sign':           {(5,):   {'start_bit':70,  'size':42, 'type':'string'},
                            ('24B',):
                                    {'start_bit':90,  'size':42, 'type':'string'}},
    'name':                {(5,):   {'start_bit':112, 'size':120,'type':'string'},
                            (19,):  {'start_bit':143, 'size':120,'type':'string'},
                            ('24A',):
                                    {'start_bit':40,  'size':120,'type':'string'}},
    'ship_and_cargo_type': {(5,):   {'start_bit':232, 'size':8,  'type':'uint'},
                            (19,):  {'start_bit':263, 'size':8,  'type':'uint'},
                            (23,):  {'start_bit':114, 'size':8,  'type':'uint'},
                            ('24B',):
                                    {'start_bit':40,  'size':8,  'type':'uint'}},
    'reference_to_bow_distance':
                           {(5,):   {'start_bit':240, 'size':9,  'type':'uint'},
                            (19,):  {'start_bit':271, 'size':9,  'type':'uint'},
                            (21,):  {'start_bit':219, 'size':9,  'type':'uint'},
                            ('24B',):
                                    {'start_bit':132, 'size':9,  'type':'uint'}},
    'reference_to_stern_distance':
                           {(5,):   {'start_bit':249, 'size':9,  'type':'uint'},
                            (19,):  {'start_bit':280, 'size':9,  'type':'uint'},
                            (21,):  {'start_bit':228, 'size':9,  'type':'uint'},
                            ('24B',):
                                    {'start_bit':141, 'size':9,  'type':'uint'}},
    'reference_to_port_distance':
                           {(5,):   {'start_bit':258, 'size':6,  'type':'uint'},
                            (19,):  {'start_bit':289, 'size':6,  'type':'uint'},
                            (21,):  {'start_bit':237, 'size':6,  'type':'uint'},
                            ('24B',):
                                    {'start_bit':150, 'size':6,  'type':'uint'}},
    'reference_to_starboard_distance':
                           {(5,):   {'start_bit':264, 'size':6,  'type':'uint'},
                            (19,):  {'start_bit':295, 'size':6,  'type':'uint'},
                            (21,):  {'start_bit':243, 'size':6,  'type':'uint'},
                            ('24B',):
                                    {'start_bit':156, 'size':6,  'type':'uint'}},
    'eta_month':           {(5,):   {'start_bit':274, 'size':4,  'type':'uint'}},
    'eta_day':             {(5,):   {'start_bit':278, 'size':5,  'type':'uint'}},
    'eta_hour':            {(5,):   {'start_bit':283, 'size':5,  'type':'uint'}},
    'eta_minute':          {(5,):   {'start_bit':288, 'size':6,  'type':'uint'}},
    'maximum_present_static_draught':
                           {(5,):   {'start_bit':294, 'size':8,  'type':'uint', 'post':processDraught}},
    'destination':         {(5,):   {'start_bit':302, 'size':120,'type':'string'}},
    'dte':                 {(5,):   {'start_bit':422, 'size':1,  'type':'uint'},
                            (9,):   {'start_bit':142, 'size':1,  'type':'uint'},
                            (19,):  {'start_bit':306, 'size':1,  'type':'uint'}},
    'sequence_number':     {(6,12): {'start_bit':38,  'size':2,  'type':'uint'}},
    'destination_id':      {(6,10,12,25):
                                    {'start_bit':40,  'size':30, 'type':'uint', 'post':processDestinationID}},
    'retransmit_flag':     {(6,12): {'start_bit':70,  'size':1,  'type':'uint'}},
    'binary_data':         {(6,):   {'start_bit':72,             'type':'bits', 'post':processBinaryData},
                            (8,25,26):
                                    {'start_bit':40,             'type':'bits', 'post':processBinaryData},
                            (17,):  {'start_bit':80,             'type':'bits', 'post':processBinaryData}},
    'destination_id1':     {(7,13,15):
                                    {'start_bit':40,  'size':30, 'type':'uint'},
                            (22,):  {'start_bit':87,  'size':30, 'type':'uint'}},
    'sequence_number_for_id1':
                           {(7,13): {'start_bit':70,  'size':2,  'type':'uint'}},
    'destination_id2':     {(7,13): {'start_bit':72,  'size':30, 'type':'uint'},
                            (15,):  {'start_bit':110, 'size':30, 'type':'uint'},
                            (22,):  {'start_bit':122, 'size':30, 'type':'uint'}},
    'sequence_number_for_id2':
                           {(7,13): {'start_bit':102, 'size':2,  'type':'uint'}},
    'destination_id3':     {(7,13): {'start_bit':104, 'size':30, 'type':'uint'}},
    'sequence_number_for_id3':
                           {(7,13): {'start_bit':134, 'size':2,  'type':'uint'}},
    'destination_id4':     {(7,13): {'start_bit':136, 'size':30, 'type':'uint'}},
    'sequence_number_for_id4':
                           {(7,13): {'start_bit':166, 'size':2,  'type':'uint'}},
    'altitude':            {(9,):   {'start_bit':38,  'size':12, 'type':'uint', 'post':processAltitude}},
    'altitude_sensor':     {(9,):   {'start_bit':134, 'size':1,  'type':'uint'}},
    'assigned_mode_flag':  {(9,):   {'start_bit':146, 'size':1,  'type':'uint'},
                            (19,):  {'start_bit':307, 'size':1,  'type':'uint'}},
    'safety_related_text': {(12,):  {'start_bit':72,             'type':'string'},
                            (14,):  {'start_bit':40,             'type':'string'}},
    'message_id1_1':       {(15,):  {'start_bit':70,  'size':6,  'type':'uint'}},
    'slot_offset_1_1':     {(15,):  {'start_bit':76,  'size':12, 'type':'uint'}},
    'message_id1_2':       {(15,):  {'start_bit':90,  'size':6,  'type':'uint'}},
    'slot_offset_1_2':     {(15,):  {'start_bit':96,  'size':12, 'type':'uint'}},
    'message_id2_1':       {(15,):  {'start_bit':140, 'size':6,  'type':'uint'}},
    'slot_offset_2_1':     {(15,):  {'start_bit':146, 'size':12, 'type':'uint'}},
    'destination_id_a':    {(16,):  {'start_bit':40,  'size':30, 'type':'uint'}},
    'offset_a':            {(16,):  {'start_bit':70,  'size':12, 'type':'uint'}},
    'increment_a':         {(16,):  {'start_bit':82,  'size':10, 'type':'uint'}},
    'destination_id_b':    {(16,):  {'start_bit':92,  'size':30, 'type':'uint'}},
    'offset_b':            {(16,):  {'start_bit':122, 'size':12, 'type':'uint'}},
    'increment_b':         {(16,):  {'start_bit':134, 'size':10, 'type':'uint'}},
    'class_b_unit_flag':   {(18,):  {'start_bit':141, 'size':1,  'type':'uint'}},
    'class_b_display_flag':{(18,):  {'start_bit':142, 'size':1,  'type':'uint'}},
    'class_b_dsc_flag':    {(18,):  {'start_bit':143, 'size':1,  'type':'uint'}},
    'class_b_band_flag':   {(18,):  {'start_bit':144, 'size':1,  'type':'uint'}},
    'class_b_message_22_flag':
                           {(18,):  {'start_bit':145, 'size':1,  'type':'uint'}},
    'mode_flag':           {(18,):  {'start_bit':146, 'size':1,  'type':'uint'}},
    'offset_number_1':     {(20,):  {'start_bit':40,  'size':12, 'type':'uint'}},
    'number_of_slots_1':   {(20,):  {'start_bit':52,  'size':4,  'type':'uint'}},
    'timeout_1':           {(20,):  {'start_bit':56,  'size':3,  'type':'uint'}},
    'increment_1':         {(20,):  {'start_bit':59,  'size':11, 'type':'uint'}},
    'offset_number_2':     {(20,):  {'start_bit':70,  'size':12, 'type':'uint'}},
    'number_of_slots_2':   {(20,):  {'start_bit':82,  'size':4,  'type':'uint'}},
    'timeout_2':           {(20,):  {'start_bit':86,  'size':3,  'type':'uint'}},
    'increment_2':         {(20,):  {'start_bit':89,  'size':11, 'type':'uint'}},
    'offset_number_3':     {(20,):  {'start_bit':100, 'size':12, 'type':'uint'}},
    'number_of_slots_3':   {(20,):  {'start_bit':112, 'size':4,  'type':'uint'}},
    'timeout_3':           {(20,):  {'start_bit':116, 'size':3,  'type':'uint'}},
    'increment_3':         {(20,):  {'start_bit':119, 'size':11, 'type':'uint'}},
    'offset_number_4':     {(20,):  {'start_bit':130, 'size':12, 'type':'uint'}},
    'number_of_slots_4':   {(20,):  {'start_bit':142, 'size':4,  'type':'uint'}},
    'timeout_4':           {(20,):  {'start_bit':146, 'size':3,  'type':'uint'}},
    'increment_4':         {(20,):  {'start_bit':149, 'size':11, 'type':'uint'}},
    'type_of_aids_to_navigation':
                           {(21,):  {'start_bit':38,  'size':5,  'type':'uint'}},
    'name_of_aids_to_navigation':
                           {(21,):  {'start_bit':43,  'size':120,'type':'string'}},
    'off_position_indicator':
                           {(21,):  {'start_bit':263, 'size':1,  'type':'uint'}},
    'aton_status':         {(21,):  {'start_bit':264, 'size':8,  'type':'uint'}},
    'virtual_aton_flag':   {(21,):  {'start_bit':275, 'size':1,  'type':'uint'}},
    'name_of_aids_to_navigation_extension':
                           {(21,):  {'start_bit':276,            'type':'string'}},
    'channel_a':           {(22,):  {'start_bit':40,  'size':12, 'type':'uint'}},
    'channel_b':           {(22,):  {'start_bit':52,  'size':12, 'type':'uint'}},
    'tx_rx_mode':          {(22,):  {'start_bit':64,  'size':4,  'type':'uint'},
                            (23,):  {'start_bit':144, 'size':2,  'type':'uint'}},
    'power':               {(22,):  {'start_bit':68,  'size':1,  'type':'uint'}},
    'longitude_1':         {(22,):  {'start_bit':69,  'size':18, 'type':'int',  'post':processLongitudeLowRes},
                            (23,):  {'start_bit':40,  'size':18, 'type':'int',  'post':processLongitudeLowRes}},
    'latitude_1':          {(22,):  {'start_bit':87,  'size':17, 'type':'int',  'post':processLatitudeLowRes},
                            (23,):  {'start_bit':58,  'size':17, 'type':'int',  'post':processLatitudeLowRes}},
    'longitude_2':         {(22,):  {'start_bit':104, 'size':18, 'type':'int',  'post':processLongitudeLowRes},
                            (23,):  {'start_bit':75,  'size':18, 'type':'int',  'post':processLongitudeLowRes}},
    'latitude_2':          {(22,):  {'start_bit':122, 'size':17, 'type':'int',  'post':processLatitudeLowRes},
                            (23,):  {'start_bit':93, 'size':17, 'type':'int',  'post':processLatitudeLowRes}},
    'addressed_or_broadcast_indicator':
                           {(22,):  {'start_bit':139,  'size':1,  'type':'uint', 'post':processAddressedOrBroadcast}},
    'channel_a_bandwidth': {(22,):  {'start_bit':140,  'size':1,  'type':'uint'}},
    'channel_b_bandwidth': {(22,):  {'start_bit':141,  'size':1,  'type':'uint'}},
    'transitional_zone_size':
                           {(22,):  {'start_bit':142,  'size':3,  'type':'uint', 'post':processTransitionalZoneSize}},
    'station_type':        {(23,):  {'start_bit':110,  'size':4,  'type':'uint'}},
    'reporting_interval':  {(23,):  {'start_bit':146,  'size':4,  'type':'uint'}},
    'quiet_time':          {(23,):  {'start_bit':150,  'size':4,  'type':'uint'}},
    'vendor_id':           {('24B',):
                                    {'start_bit':48,   'size':42, 'type':'bits', 'post':processVendorID}},
    'position_latency':    {(27,):  {'start_bit':94,   'size':1,  'type':'uint'}},

}

class AISDecoder:

    def __init__(self):
        self.pendingMessages = {}
        self.decodedMessages = []
        self.last_nmea_parts = None

    def addNMEA(self, nmea):
        parts = nmea.split(',')
        self.last_nmea_parts = parts
        #print(parts)
        if len(parts) >= 7:
            try:
                fragment_count = int(parts[1])
            except:
                return
            try:
                fragment_number = int(parts[2])
            except:
                return
            channel = parts[4]
            payload = parts[5]
            if fragment_count == 1:
                #complete message in single nmea sentance
                self.addMessage(decodePayload(payload, channel, (nmea,)))
            else:
                if not channel in self.pendingMessages or self.pendingMessages[channel] is None:
                    self.pendingMessages[channel] = {'nmea':[]}
                self.pendingMessages[channel][fragment_number] = payload
                self.pendingMessages[channel]['nmea'].append(nmea)
                if len(self.pendingMessages[channel]) == fragment_count:
                    # we should have all the fragments now
                    combined_payload = ''
                    try:
                        for i in range(1,fragment_count+1):
                            combined_payload += self.pendingMessages[channel][i]
                        self.addMessage(decodePayload(combined_payload, channel,self.pendingMessages[channel]['nmea']))
                    except KeyError:
                        pass
                self.pendingMessages[channel] = None

    def popMessages(self):
        ret = self.decodedMessages
        self.decodedMessages = []
        return ret

                
    def addMessage(self,msg):
        self.decodedMessages.append(msg)


if __name__ == '__main__':
    import sys
    
    d = AISDecoder()

    bin_msg_types = {}
    msg_types = {}
    
    total = 0
    try:
        for fname in sys.argv[1:]:
            print(fname)
            for l in open(fname).readlines():
                if l.startswith('!AIVDM'):
                    parts = (None,l)
                else:
                    parts = l.strip().split(',',1)
                if len(parts) > 1 and len(parts[1]) > 1:
                    if parts[1].startswith('!AIVDM'):
                        try:
                            d.addNMEA(parts[1])
                        except Exception as e:
                            print("Error parsing:", d.last_nmea_parts, ' pending: ', d.pendingMessages)
                            raise
                        for m in d.popMessages():

                            # if m['message_id'] in (27,):
                            #     print (m)

                            if m['message_id'] in (6,8):
                                dac_fid = str(m['binary_data']['application_identifier']['designated_area_code'])+','+str(m['binary_data']['application_identifier']['function_identifier'])
                                if not dac_fid in bin_msg_types:
                                    bin_msg_types[dac_fid] = 0
                                bin_msg_types[dac_fid] = bin_msg_types[dac_fid]+1
                            if not m['message_id'] in msg_types:
                                msg_types[m['message_id']] = 0
                            msg_types[m['message_id']] += 1
                            total += 1
                            if total%10000 == 0:
                                print (total)
    except KeyboardInterrupt:
        pass
    print (total,'ais messages seen.')
    print ('types seen:')
    for k in sorted(msg_types.keys()):
        print ('\ttype:',k,'count:', msg_types[k])
    print ('bin msgs seen:')
    for k in sorted(bin_msg_types):
        print ('\tdac,fi',k,'count:',bin_msg_types[k])

