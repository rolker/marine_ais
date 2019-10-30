#!/usr/bin/env python

# based on https://gpsd.gitlab.io/gpsd/AIVDM.html

class AISDecoder:
    def __init__(self):
        self.pendingMessages = {}
        self.decodedMessages = []
        self.types_seen = {}
        self.types_not_decoded = []
        self.mmsi_db = {}

    def addNMEA(self, nmea):
        parts = nmea.split(',')
        if len(parts) == 7:
            fragment_count = int(parts[1])
            fragment_number = int(parts[2])
            message_id = parts[3]
            channel = parts[4]
            payload = parts[5]
            if fragment_count == 1:
                #complete message in single nmea sentance
                self.decodePayload(payload)
            else:
                if not channel in self.pendingMessages or self.pendingMessages[channel] is None:
                    self.pendingMessages[channel] = {}
                self.pendingMessages[channel][fragment_number] = payload
                if len(self.pendingMessages[channel]) == fragment_count:
                    # we should have all the fragments now
                    combined_payload = ''
                    for i in range(1,fragment_count+1):
                        combined_payload += self.pendingMessages[channel][i]
                    self.decodePayload(combined_payload)
                    self.pendingMessages[channel] = None

    def popMessages(self):
        ret = self.decodedMessages
        self.decodedMessages = []
        return ret

    def decodePayload(self,payload):
        #print 'payload:',payload,len(payload),'nibbles',len(payload*6),'bits?'
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
            
        message_type = self.decodeUnsignedInt(bits[:6])
        if not message_type in self.types_seen:
            self.types_seen[message_type] = 0
        self.types_seen[message_type] += 1
        
        if message_type in (1,2,3):
            self.decodeMessageType123(bits)
        elif message_type == 4:
            self.decodeMessageType4(bits)
        elif message_type == 5:
            self.decodeMessageType5(bits)
        elif message_type == 8:
            self.decodeMessageType8(bits)
        elif message_type == 18:
            self.decodeMessageType18(bits)
        elif message_type == 21:
            self.decodeMessageType21(bits)
        elif message_type == 24:
            self.decodeMessageType24(bits)
        else:
            if not message_type in self.types_not_decoded:
                self.types_not_decoded.append(message_type)
                
    def addMessage(self,msg):
        self.decodedMessages.append(msg)
        if not msg['mmsi'] in self.mmsi_db:
            self.mmsi_db[msg['mmsi']] = {}
        for k,v in msg.iteritems():
            self.mmsi_db[msg['mmsi']][k] = v
        

    def decodeUnsignedInt(self, bits):
        ret = 0
        for b in bits:
            ret = ret << 1
            if b:
                ret = ret | 1
        return ret

    def decodeSignedInt(self, bits):
        if not bits[0]: # positive
            return self.decodeUnsignedInt(bits[1:])
        # negative so two's complement
        ret = 0
        for b in bits:
            ret = ret << 1
            if not b:
                ret = ret | 1
        ret += 1
        return -ret

    def decodeString(self, bits, strip=True):
        ret = ''
        for i in range(0,len(bits),6):
            c = self.decodeChar(bits[i:])
            if c == '@':
                break
            ret += c
        if strip:
            return ret.rstrip()
        return ret

    def decodeChar(self, bits):
        code = self.decodeUnsignedInt(bits[:6])
        if code < 32:
            code += 64
        return chr(code)

    def decodeHeader(self, bits, ret):
        ret['type'] = self.decodeUnsignedInt(bits[:6])
        ret['repeat'] = self.decodeUnsignedInt(bits[6:8])
        ret['mmsi'] = self.decodeUnsignedInt(bits[8:38])

    def decodeLatLon(self, bits, ret):
        ret['accuracy'] = bits[0]
        
        ret['lon'] = self.decodeSignedInt(bits[1:29])
        if ret['lon'] == 0x6791AC0:
            ret['lon'] = None
        else:
            ret['lon']/=600000.0
            
        ret['lat'] = self.decodeSignedInt(bits[29:56])
        if ret['lat'] == 0x3412140:
            ret['lat'] = None
        else:
            ret['lat'] /= 600000.0
        
        
    def decodeCommonNav(self, bits, ret):
        ret['speed'] = self.decodeUnsignedInt(bits[:10])
        if ret['speed'] == 1023:
            ret['speed'] = None
        else:
            ret['speed']*= 0.1

        self.decodeLatLon(bits[10:66],ret)
            
        ret['course'] = self.decodeUnsignedInt(bits[66:78])
        if ret['course'] == 3600:
            ret['course'] = None
        else:
            ret['course'] *= 0.1
            
        ret['heading'] = self.decodeUnsignedInt(bits[78:87])
        if ret['heading'] == 511:
            ret['heading'] = None
            
        ret['second'] = self.decodeUnsignedInt(bits[87:93])
        if ret['second'] == 60:
            ret['second'] = None

    def decodeDimenstions(self, bits, ret):
        ret['to_bow'] = self.decodeUnsignedInt(bits[:9])
        ret['to_stern'] = self.decodeUnsignedInt(bits[9:18])
        ret['to_port'] = self.decodeUnsignedInt(bits[18:24])
        ret['to_starboard'] = self.decodeUnsignedInt(bits[24:30])


    def decodeEPFD(self, bits):
        return (None,'GPS','GLONASS','Combined GPS/GLONASS','Loran-C','Chayka','Integrated navigation system','Surveyed','Galileo',None,None,None,None,None,None,None,None)[self.decodeUnsignedInt(bits)]

    def decodeData(self, bits):
        ret = []
        i = 0
        while i < len(bits):
            if i+8 < len(bits):
                byte = bits[i:i+8]
            else:
                byte = bits[i:]
                while len(byte) < 8:
                    byte.append(False)
            ret.append(self.decodeUnsignedInt(byte))
            i+=8
            
        return ret

        
    def decodeMessageType123(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)

        ret['status'] = self.decodeUnsignedInt(bits[38:42])

        ret['turn'] = self.decodeSignedInt(bits[42:50])
        sign = 1
        if ret['turn'] < 0:
            sign = -1
        if ret['turn'] == -128:
            ret['turn'] = None
        elif ret['turn'] == 127:
            ret['turn'] = 'turning right at more than 5deg/30s (No TI available)'
        elif ret['turn'] == -127:
            ret['turn'] = 'turning left at more than 5deg/30s (No TI available)'
        else:
            ret['turn'] =  sign*((ret['turn']/4.733)**2)

        self.decodeCommonNav(bits[50:143],ret)
        
        ret['maneuver'] = self.decodeUnsignedInt(bits[143:145])
        if ret['maneuver'] == 1:
            ret['maneuver'] = 'No special maneuver'
        elif ret['maneuver'] == 2:
            ret['maneuver'] = 'Special maneuver'
        else:
            ret['maneuver'] = None
            
        ret['raim'] = bits[148]
        ret['radio'] = self.decodeUnsignedInt(bits[149:168])
        
        self.addMessage(ret)
        

    def decodeMessageType4(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)
        
        ret['year'] = self.decodeUnsignedInt(bits[38:52])
        ret['month'] = self.decodeUnsignedInt(bits[52:56])        
        ret['day'] = self.decodeUnsignedInt(bits[56:61])
        ret['hour'] = self.decodeUnsignedInt(bits[61:66])
        ret['minute'] = self.decodeUnsignedInt(bits[66:72])
        ret['second'] = self.decodeUnsignedInt(bits[72:78])
        
        self.decodeLatLon(bits[78:134],ret)
            
        ret['epfd'] = self.decodeEPFD(bits[134:138])
        ret['raim'] = bits[148]
        ret['radio'] = self.decodeUnsignedInt(bits[149:168])

        self.addMessage(ret)
        

    def decodeMessageType5(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)
        ret['ais_version'] = self.decodeUnsignedInt(bits[38:40])
        ret['imo'] = self.decodeUnsignedInt(bits[40:70])
        ret['callsign'] = self.decodeString(bits[70:112])
        ret['shipname'] = self.decodeString(bits[112:232])
        ret['shiptype'] = self.decodeUnsignedInt(bits[232:240])
        self.decodeDimenstions(bits[240:270],ret)
        ret['epfd'] = self.decodeEPFD(bits[270:274])
        ret['month'] = self.decodeUnsignedInt(bits[274:278])
        ret['day'] = self.decodeUnsignedInt(bits[278:283])
        ret['hour'] = self.decodeUnsignedInt(bits[283:288])
        ret['minute'] = self.decodeUnsignedInt(bits[288:293])
        ret['draught'] = self.decodeUnsignedInt(bits[294:302])/10.0
        ret['destination'] = self.decodeString(bits[302:422])
        ret['dte'] = bits[422]

        self.addMessage(ret)

    def decodeMessageType8(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)
        
        ret['dac'] = self.decodeUnsignedInt(bits[40:50])
        ret['fid'] = self.decodeUnsignedInt(bits[50:56])
        ret['data'] = self.decodeData(bits[56:])
        
        self.addMessage(ret)

    def decodeMessageType18(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)
        self.decodeCommonNav(bits[46:139],ret)
        
        ret['regional'] = self.decodeUnsignedInt(bits[139:141])
        ret['cs'] = bits[141]
        ret['display'] = bits[142]
        ret['dsc'] = bits[143]
        ret['band'] = bits[144]
        ret['msg22'] = bits[145]
        ret['assigned'] = bits[146]
        ret['raim'] = bits[147]
        
        self.addMessage(ret)

    def decodeMessageType21(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)
        
        ret['aid_type'] = self.decodeUnsignedInt(bits[38:43])
        ret['aid_type'] = ('Default, Type of Aid to Navigation not specified',
                           'Reference point',
                           'RACON (radar transponder marking a navigation hazard)',
                           'Fixed structure off shore, such as oil platforms, wind farms, rigs. (Note: This code should identify an obstruction that is fitted with an Aid-to-Navigation AIS station.)',
                           'Spare, Reserved for future use.',
                           'Light, without sectors',
                           'Light, with sectors',
                           'Leading Light Front',
                           'Leading Light Rear',
                           'Beacon, Cardinal N',
                           'Beacon, Cardinal E',
                           'Beacon, Cardinal S',
                           'Beacon, Cardinal W',
                           'Beacon, Port hand',
                           'Beacon, Starboard hand',
                           'Beacon, Preferred Channel port hand',
                           'Beacon, Preferred Channel starboard hand',
                           'Beacon, Isolated danger',
                           'Beacon, Safe water',
                           'Beacon, Special mark',
                           'Cardinal Mark N',
                           'Cardinal Mark E',
                           'Cardinal Mark S',
                           'Cardinal Mark W',
                           'Port hand Mark',
                           'Starboard hand Mark',
                           'Preferred Channel Port hand',
                           'Preferred Channel Starboard hand',
                           'Isolated danger',
                           'Safe Water',
                           'Special Mark',
                           'Light Vessel / LANBY / Rigs'
                           )[ret['aid_type']]
        ret['name'] = self.decodeString(bits[43:163],False)
        self.decodeLatLon(bits[163:219],ret)
        self.decodeDimenstions(bits[219:249],ret)
        ret['epfd'] = self.decodeEPFD(bits[249:253])
        ret['second'] = self.decodeUnsignedInt(bits[253:259])
        ret['off_position'] = bits[259]
        ret['regional'] = self.decodeUnsignedInt(bits[260:268])
        ret['raim'] = bits[268]
        ret['virtual_aid'] = bits[269]
        ret['assigned'] = bits[270]
        if len(ret['name']) == 20 and len(bits)>272:
            ret['name'] += self.decodeString(bits[272:])
        ret['name'] = ret['name'].rstrip()
        
        self.addMessage(ret)
        

    def decodeMessageType24(self,bits):
        ret = {}
        self.decodeHeader(bits,ret)
        
        ret['partno'] = self.decodeUnsignedInt(bits[38:40])
        if ret['partno'] == 0: # Part A
            ret['shipname'] = self.decodeString(bits[40:160])
        elif ret['partno'] == 1: # Part B
            ret['shiptype'] = self.decodeUnsignedInt(bits[40:48])
            ret['vendorid'] = self.decodeString(bits[48:66])
            ret['model'] = self.decodeUnsignedInt(bits[66:70])
            ret['serial'] = self.decodeUnsignedInt(bits[70:90])
            ret['callsign'] = self.decodeString(bits[90:132])

            # is this an auxillary craft?
            mmsi_str = str(ret['mmsi'])
            is_aux = len(mmsi_str) == 9 and mmsi_str[:2] == '98'

            if not is_aux:
                self.decodeDimenstions(bits[132:162],ret)
            else:
                ret['mothership_mmsi'] = self.decodeUnsignedInt(bits[132:162])
            
        self.addMessage(ret)


if __name__ == '__main__':
    import sys
    
    d = AISDecoder()
    
    total = 0
    
    for l in file(sys.argv[1]).readlines():
        parts = l.strip().split(',',1)
        if len(parts) > 1 and len(parts[1]) > 1:
            if parts[1].startswith('!AIVDM'):
                d.addNMEA(parts[1])
                for m in d.popMessages():
                    total += 1
                    if total%10000 == 0:
                        print total
    print total,'ais messages seen.'
    print 'types seen:'
    for k,v in d.types_seen.iteritems():
        print '\ttype:',k,'count:',v
    print 'types not decoded:',d.types_not_decoded
    print 'ships seen:'
    for k in d.mmsi_db.iterkeys():
        if 'shipname' in d.mmsi_db[k]:
            print '\t',k, d.mmsi_db[k]['shipname'],
            if 'callsign' in d.mmsi_db[k]:
                print '(',d.mmsi_db[k]['callsign'],')'
            else:
                print
