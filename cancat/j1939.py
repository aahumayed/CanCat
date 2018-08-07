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

    def printJ1939Msgs(self, start_msg=0, stop_msg=None, start_bkmk=None, stop_bkmk=None, start_baseline_msg=None, stop_baseline_msg=None, arbids=None, priority=None, pgns=None, sourceAddresses=None, spns=None, ignore=[]):
        print self.reprCanMsgs(start_msg, stop_msg, start_bkmk, stop_bkmk, start_baseline_msg, stop_baseline_msg, arbids, priority, pgns, sourceAddresses, spns, ignore)

    def reprCanMsgs(self, start_msg=0, stop_msg=None, start_bkmk=None, stop_bkmk=None, start_baseline_msg=None, stop_baseline_msg=None, arbids=None,priority=None, pgns=None, sourceAddresses=None, spns=None, ignore=[]):
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

        for idx, ts, arbid, pgns, msg in self.filterCanMsgs(start_msg, stop_msg, start_baseline_msg, stop_baseline_msg, arbids=arbids, pgns=pgns,sourceAddresses=sourceAddresses, ignore=ignore):
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

            out.append(self.reprCanMsg(idx, ts, arbid, msg, comment='\t'.join(diff)))
            last_ts = ts
            last_msg = msg

        out.append("Total Messages: %d  (repeat: %d / similar: %d)" % (msg_count, data_repeat, data_similar))

        return "\n".join(out)

    def reprCanMsg(self, idx, ts, arbid, data, comment=None):
        #TODO: make some repr magic that spits out known ARBID's and other subdata
        if comment == None:
            comment = ''
        priority, pgn, pgnName, sourceAddress = splitID(self, arbid)
        return "%.8d %8.3f ID: %.3x, Priority: %d, PGN: %d (%s), SA: %d,  Len: %.2x, Data: %-18s\t%s" % (idx, ts, arbid, priority, pgn, pgnName, sourceAddress, len(data), data.encode('hex'), comment)

    def filterCanMsgs(self, start_msg=0, stop_msg=None, start_baseline_msg=None, stop_baseline_msg=None, arbids=None, priority=None, pgns=None, sourceAddresses=None, spns=None, ignore=[]):
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
            filter_ids = { arbid:1 for ts,arbid,data in self.genCanMsgs(start_baseline_msg, stop_baseline_msg)
                }.keys()
        else:
            filter_ids = None
        self.c.log("filtering messages...")
        filteredMsgs = [(idx, ts,arbid, pgn, msg) for idx, ts,arbid, pgn, msg in self.genCanMsgs(start_msg, stop_msg, arbids=arbids, pgns=pgns,sourceAddresses=sourceAddresses)
                if (type(arbids) == list and arbid in arbids) or arbid not in ignore and (filter_ids==None or arbid not in filter_ids)]
# (idx, ts, arbid, pgn, data)
        return filteredMsgs

    def genCanMsgs(self, start=0, stop=None, arbids=None, priority=None, pgns=None, sourceAddresses=None, spns=None):
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
                priority, pgn, pgnName , sa = splitID(self,arbid)
                #spns=getSPNs(self,pgn)


                if arbids != None and arbid not in arbids:
                    # allow filtering of arbids
                    continue
                if pgns != None and pgn not in pgns:
                    # allow filtering of arbids
                    continue
                #if priority != None and priority not in priority:
                    # allow filtering of arbids
                #    continue
                if sourceAddresses != None and sa not in sourceAddresses:
                    # allow filtering of arbids
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

def splitID(self, arbid):
    priority = arbid >> 26 & 0b111
    pgn = arbid >> 8 & 0b00001111111111111111
    sourceAddress = arbid & 0b00000000000000000000011111111
    pgnName = getPgnName(self, pgn)
    return priority, pgn, pgnName , sourceAddress

def getPgnName(self, pgn):
    try:
        return self.j1939DB['J1939PGNdb'][str(pgn)]['Name']
    except:
        #print "PGN: ", pgn, "not found.\n"
        pass

def getSPNs(self,pgn):
    try:
        return self.j1939DB['J1939PGNdb'][str(pgn)]['SPNs']
    except:
        pass


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
