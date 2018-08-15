#!/usr/bin/env python
import sys
import time
import cancat
import struct
import threading
import json

import cancat.iso_tp as cisotp

class J1939:
    def __init__(self, c, verbose=True):
        self.c = c
        self.verbose = verbose
        self.readJ1939DB()

    def readJ1939DB(self):
        f=open("J1939db.json", "r")
        self.j1939DB = json.load(f)
# printJ1939Msgs --> reprJ1939Msgs --> filterJ1939Msgs --> genJ1939Msgs --> reprJ1939Msg
    def printJ1939Msgs(self, start_msg=0, stop_msg=None, start_bkmk=None, stop_bkmk=None, start_baseline_msg=None, stop_baseline_msg=None, arbids=None, priorities=None, pgns=None, sourceAddresses=None, spns=None, ignore=[]):
        '''
        This function decodes CAN messages into J1939 format. Examples of usage:
        1) Importing and creating a J1939 object
        import j1939
        j=j1939.J1939(c)
        2) Decode messages without filtering:
        j.printJ1939Msgs()
        3) Filter by PGN by passing a list of PGNs:
        j.printJ1939Msgs(pgns={61441,0x123})
        4) Filter by source addresses
        j.printJ1939Msgs(sourceAddresses={1,3,6})
        5) Filter by Priority:
        j.printJ1939Msgs(priorities={0,7})
        6) Filter by spns:
        j.printJ1939Msgs(spns={520,190})
        '''
        print self.reprJ1939Msgs(start_msg, stop_msg, start_bkmk, stop_bkmk, start_baseline_msg, stop_baseline_msg, arbids, priorities, pgns, sourceAddresses, spns, ignore)

    def reprJ1939Msgs(self, start_msg=0, stop_msg=None, start_bkmk=None, stop_bkmk=None, start_baseline_msg=None, stop_baseline_msg=None, arbids=None,priorities=None, pgns=None, sourceAddresses=None, spns=None, ignore=[]):
        '''
        String representation of a set of CAN Messages.
        These can be filtered by start and stop message indexes, as well as
        use a baseline (defined by start/stop message indexes),
        by a list of "desired" arbids as well as a list of
        ignored arbids

        Many functions wrap this one.
        '''
        out = []

        if start_bkmk != None:
            start_msg = self.c.getMsgIndexFromBookmark(start_bkmk)

        if stop_bkmk != None:
            stop_msg = self.c.getMsgIndexFromBookmark(stop_bkmk)



        if start_msg in self.c.bookmarks:
            bkmk = self.c.bookmarks.index(start_msg)
            self.c.out.append("starting from bookmark %d: '%s'" %
                    (bkmk,
                    self.c.bookmark_info[bkmk].get('name'))
                    )

        if stop_msg in self.c.bookmarks:
            bkmk = self.c.bookmarks.index(stop_msg)
            self.c.out.append("stoppng at bookmark %d: '%s'" %
                    (bkmk,
                    self.c.bookmark_info[bkmk].get('name'))
                    )

        last_msg = None
        next_bkmk = 0
        next_bkmk_idx = 0

        msg_count = 0
        last_ts = None
        tot_delta_ts = 0
        counted_msgs = 0    # used for calculating averages, excluding outliers

        data_delta = None


        data_repeat = 0
        data_similar = 0

        for idx, ts, arbid, pgns, msg in self.filterJ1939Msgs(start_msg, stop_msg, start_baseline_msg, stop_baseline_msg, arbids=arbids, priorities=priorities, pgns=pgns,sourceAddresses=sourceAddresses, spns=spns, ignore=ignore):
            diff = []

            # insert bookmark names/comments in appropriate places
            while next_bkmk_idx < len(self.c.bookmarks) and idx >= self.c.bookmarks[next_bkmk_idx]:
                out.append(self.c.reprBookmark(next_bkmk_idx))
                next_bkmk_idx += 1

            msg_count += 1

            # check data
            byte_cnt_diff = 0
            if last_msg != None:
                if len(last_msg) == len(msg):
                    for bidx in range(len(msg)):
                        if last_msg[bidx] != msg[bidx]:
                            byte_cnt_diff += 1

                    if byte_cnt_diff == 0:
                        diff.append("REPEAT")
                        data_repeat += 1
                    elif byte_cnt_diff <=4:
                        diff.append("Similar")
                        data_similar += 1
                    # FIXME: make some better heuristic to identify "out of norm"

            # look for ASCII data (4+ consecutive bytes)
            if hasAscii(msg):
                diff.append("ASCII: %s" % repr(msg))

            # calculate timestamp delta and comment if out of whack
            if last_ts == None:
                last_ts = ts

            delta_ts = ts - last_ts
            if counted_msgs:
                avg_delta_ts = tot_delta_ts / counted_msgs
            else:
                avg_delta_ts = delta_ts


            if abs(delta_ts - avg_delta_ts) <= delta_ts:
                tot_delta_ts += delta_ts
                counted_msgs += 1
            else:
                diff.append("TS_delta: %.3f" % delta_ts)

            out.append(self.reprJ1939Msg(idx, ts, arbid, msg, comment='\t'.join(diff)))
            last_ts = ts
            last_msg = msg

        out.append("Total Messages: %d  (repeat: %d / similar: %d)" % (msg_count, data_repeat, data_similar))

        return "\n".join(out)

    def reprJ1939Msg(self, idx, ts, arbid, data, comment=None):
        #TODO: make decoding spns optional
        if comment == None:
            comment = ''
        priority, pgn, pgnName, sourceAddress = self.splitID(arbid)
        spns= self.getSPNs(pgn)
        d = ""
        #TODO SPNs are appended to data in a different order than the actual CAN frame, not sure if this is of significance
        if spns != None:
            for spn in spns:
                d+= self.getSpnInfo(spn,int(data.encode('hex'), 16))
                #d+='SPN: ',value,', (',name,') '
        return "%.8d %8.3f ID: %.3x, Priority: %d, PGN: %d, SA: %d,  Len: %.2x, Data: %-18s\t%s" % (idx, ts, arbid, priority, pgn, sourceAddress, len(data), d, comment)

    def filterJ1939Msgs(self, start_msg=0, stop_msg=None, start_baseline_msg=None, stop_baseline_msg=None, arbids=None, priorities=None, pgns=None, sourceAddresses=None, spns=None, ignore=[]):
        '''
        returns the received CAN messages between indexes "start_msg" and "stop_msg"
        but only messages to ID's that *do not* appear in the the baseline indicated
        by "start_baseline_msg" and "stop_baseline_msg".

        for message indexes, you *will* want to look into the bookmarking subsystem!
        '''
        self.c.log("starting filtering messages...")
        if stop_baseline_msg != None:
            self.c.log("ignoring arbids from baseline...")
            # get a list of baseline arbids
            filter_ids = { arbid:1 for ts,arbid,data in self.genJ1939Msgs(start_baseline_msg, stop_baseline_msg)
                }.keys()
        else:
            filter_ids = None
        self.c.log("filtering messages...")
        filteredMsgs = [(idx, ts,arbid, pgn, msg) for idx, ts,arbid, pgn, msg in self.genJ1939Msgs(start_msg, stop_msg, arbids=arbids, priorities=priorities, pgns=pgns,sourceAddresses=sourceAddresses, spns=spns)
                if (type(arbids) == list and arbid in arbids) or arbid not in ignore and (filter_ids==None or arbid not in filter_ids)]
        return filteredMsgs

    def getSPNs(self,pgn):
        '''
        Returns a list of spns used in the passed PGN
        '''
        try:
            return self.j1939DB['J1939PGNdb'][str(pgn)]['SPNs']
        except:
            return None

    def genJ1939Msgs(self, start=0, stop=None, arbids=None, priorities=None, pgns=None, sourceAddresses=None, spns=None):
            '''
            CAN message generator.  takes in start/stop indexes as well as a list
            of desired arbids (list)
            '''
            messages = self.c._messages.get(0x30, [])# CMD_CAN_RECV = 0x30
            if stop == None:
                stop = len(messages)
            else:
                stop = stop + 1 # This makes the stop index inclusive if specified

            for idx in xrange(start, stop):
                ts, msg = messages[idx]

                arbid, data = self._splitCanMsg(msg)
                priority, pgn, pgnName, sourceAddress = self.splitID(arbid)
                currentSPNs=self.getSPNs(pgn)

                if arbids != None and arbid not in arbids:
                    # allow filtering of arbids
                    continue
                if pgns != None and pgn not in pgns:
                    # allow filtering of pgns
                    continue
                if priorities != None and priority not in priorities:
                    # allow filtering of priorities
                    continue
                if sourceAddresses != None and sourceAddress not in sourceAddresses:
                    # allow filtering of sourceAddresses
                   continue
                # spns: associated spns with the PGN. spns1: the acutal values in the data field of the J1939 message
                #if spns != None and currentSPNs != None and not spns.issubset(currentSPNs):
                    #continue
                #SPNs filtering needs two steps: 1) filter out frames with empty data fields (i.e., no defined SPNs), and then 2) filter out data fields that do not contain the specified spns
                if spns != None and currentSPNs == None:
                    continue
                if currentSPNs != None and spns != None and not any(x in spns for x in currentSPNs):
                    continue

                yield((idx, ts, arbid, pgn, data))

    def _splitCanMsg(self, msg):
            '''
            takes in captured message
            returns arbid and data

            does not check msg size.  MUST be at least 4 bytes in length as the
            tool should send 4 bytes for the arbid
            '''
            arbid = struct.unpack(">I", msg[:4])[0]
            data = msg[4:]
            return arbid, data

    def getSpnName(self, spn):
        '''
        This is a helper function where you pass a PGN value and get its name
        '''
        name = self.j1939DB['J1939SPNdb'][str(spn)]['Name']
        #bin(0xf07d84b11200f084>>48 & ((1 << 8))-1)
        print "SPN %s: (%s) " % (str(spn),name)

    def splitID(self, arbid):
        '''
        This function extracts and returns priority, PGN, and SA from a 29-bit ID
        '''
        priority = arbid >> 26 & 0b111
        pgn = arbid >> 8 & 0b00001111111111111111
        sourceAddress = arbid & 0b00000000000000000000011111111
        pgnName = self.getPgnName(pgn)
        return priority, pgn, pgnName , sourceAddress

    def constructID(self, priority, pgn, sourceAddress):
        '''
        It is used in J1939xmit and it constructs a 29-bit ID given the 3 building blocks (priority, pgn, and sourceAddress)
        Example:
        hex(j.constructID(3,61440,0))
        '''
        arbid =0
        arbid |= priority << 26
        arbid |= pgn << 8
        arbid |= sourceAddress
        return arbid

    def getPgnName(self, pgn):
        '''
        A utility function that returns a PGN's name by passing it
        Example:
        In [32]: j.getPgnName(61440)
        Out[32]: u'Electronic Retarder Controller 1'
        '''
        try:
            return self.j1939DB['J1939PGNdb'][str(pgn)]['Name']
        except:
            #print "PGN: ", pgn, "not found.\n"
            pass

    def constructSPNs(self, spns):
        '''
        This function receives a dictionary of spn key-value pairs and returns the correcsponding data field that can be transmitted
        Populated spns get they assigned values, whereas the unpopulated get 0xFF.. values
        constructSPNs(spns={190:0x6813, 520:0x12})
        IMPORTANT: This function is incompleteself.
        In [26]: hex(j.constructSPNs(spns={190:0x6813}))
        constructSPNs(spns): spns=  {190: 0x6813}
        Out[26]: '0x6813000000' ==> spn value starting at the correct startBit position (bytes 4 and 5)
        In [27]: hex(j.constructSPNs(spns={1675:13}))
        constructSPNs(spns): spns=  {1675: 13}
        Out[27]: '0xd000000000000' (starting at position 7.1)
        TODO: Instead of returning the spn values, these need to be appended to a data variable until the whole list of spns is constructed
        '''
        data = ""
        print "constructSPNs(spns): spns= ", spns
        print "type: ", type(spns)
        for k, v in spns.iteritems():
            print k, ': ', v
            startBit= self.j1939DB['J1939SPNdb'][str(k)]['StartBit']
            spnLength= self.j1939DB['J1939SPNdb'][str(k)]['SPNLength']
            msg= v <<startBit
            mask = ((1<<spnLength)-1)<<startBit

        return (msg or mask)

        #name = self.j1939DB['J1939SPNdb'][str(spn)]['Name']
        #bin(0xf07d84b11200f084>>48 & ((1 << 8))-1)
        return "%s: %s, " % (str(spn),str(value))
        return data
    def J1939xmit(self, priority, pgn, sourceAddress, data=None,spns=None, timeout=3, count=1):
        '''
        Transmit a J1939 message on the attached CAN bus
        Currently returns the *last* result
        Examples of usage:
        j.J1939xmit(7,61444,4, data='FFFFFF6813FFFFFF'.decode('hex')) ==> Works as expected
        j.J1939xmit(7,61444,4, spns={190:0x123, 571:0x1122}) ===> Incomplete, please check the "half-baked" constructSPNs function
        '''
        extflag = 1 # always 1 because J1939 uses 29-bit IDs
        id= self.constructID(priority,pgn,sourceAddress)
        #TODO spns needs to receive the correct constructed SPNs returned from constructSPNs function
        #spns= self.constructSPNs(spns)# Returns spn-friendly data
        msg = struct.pack('>I',id)+chr(extflag)+data#'FFFFFF6813FFFFFF'.decode('hex')
        print "msg= ", msg
        for i in range(count):
            self.c._send(0x44, msg)
            ts, result = self.c.recv(0x34, timeout)

        if result == None:
            print "J1939xmit:  Return is None!?"
        resval = ord(result)
        if resval != 0:
            print "J1939xmit() failed: %s" % self.c.CAN_RESPS.get(resval)

        return resval

    def getSpnInfo(self, spn, data):
        #TODO Need to catch an error when spn is not in the passed data
        '''
        A utility function critical in extracting spns' info from data
        Example:
        getSpnInfo(190,0x1245)
        In [7]: j.getSpnInfo(190,0xaabbccddeeff)
        Out[7]: '190: 48076, '
        In [8]: hex(48076)
        Out[8]: '0xbbcc'
        '''
        startBit= self.j1939DB['J1939SPNdb'][str(spn)]['StartBit']
        spnLength= self.j1939DB['J1939SPNdb'][str(spn)]['SPNLength']
        value = (data >> startBit & ((1 << spnLength))-1)
        #name = self.j1939DB['J1939SPNdb'][str(spn)]['Name']
        #bin(0xf07d84b11200f084>>48 & ((1 << 8))-1)
        return "%s: %s, " % (str(spn),str(value))
        #return str(value), name

def hasAscii(msg, minbytes=4, strict=True):
    '''
    if minbytes == -1, every character has to be clean ASCII
    otherwise, look for strings of at least minbytes in length
    '''
    ascii_match = 0
    ascii_count = 0
    for byte in msg:
        if 0x30 <= ord(byte) < 0x7f:
            ascii_count +=1
            if ascii_count >= minbytes:
                ascii_match = 1
        else:
            if strict:
                return 0

            ascii_count = 0
    return ascii_match
