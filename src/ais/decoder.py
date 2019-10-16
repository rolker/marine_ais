#!/usr/bin/env python

# based on https://gpsd.gitlab.io/gpsd/AIVDM.html

class AISDecoder:
    def __init__(self):
        self.pendingMessages = {}
        self.decodedMessages = []

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
                    

    def decodePayload(self,payload):
        #print 'payload:',payload
        nibbles = []
        for c in payload:
            nibble = ord(c)-48
            if nibble > 40:
                nibble -= 8
            nibbles.append(nibble)
            
        #print '\t',nibbles
        message_type = nibbles[0]
        
        if message_type in (1,2,3):
            self.decodeMessageType123(nibbles)
        else:
            print 'skipping message type',message_type

    def extact(self, nibbles, start, end, return_type='u'):
        start_nibble = start/6
        start_bit = start%6
        print start,start_nibble,start_bit
        end_nibble = end/6
        end_bit = end%6
        print end,end_nibble,end_bit
        

    def decodeMessageType123(self,nibbles):
        repeat = self.extact(nibbles,6,7)
        print 'repeat:',repeat


if __name__ == '__main__':
    import sys
    
    d = AISDecoder()
    
    for l in file(sys.argv[1]).readlines():
        parts = l.strip().split(',',1)
        if len(parts) > 1 and len(parts[1]) > 1:
            if parts[1].startswith('!AIVDM'):
                d.addNMEA(parts[1])
