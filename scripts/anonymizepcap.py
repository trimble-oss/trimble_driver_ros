#!/usr/bin/env python3
# SPDX-License-Identifier: Unlicense

# This file was originally obtained from https://github.com/mwander/anonymizepcap/tree/main
# and was covered under the Unlicense license.

# Author: Matthaeus Wander <mail@wander.science>

# Dependencies:
import dpkt

import argparse
import hashlib
import ipaddress
import hmac
import os
import signal
import socket
import struct
import sys

IGNORE_ETHERTYPES = [
    dpkt.ethernet.ETH_TYPE_ARP,
]

class NetworkList(object):
    def __init__(self, netfile):
        self.networks = []
        self.cache = {}

        with open(netfile, 'r') as fh:
            for line in fh:
                line = line.strip()
                if line:
                    self.networks.append(ipaddress.ip_network(line))

    def __iter__(self):
        return self.networks.__iter__()

    def __contains__(self, ip):
        if ip in self.cache:
            return self.cache[ip]

        # Build cache
        ipa = ipaddress.ip_address(ip)
        for net in self.networks:
            if ipa in net:
                self.cache[ip] = True
                return True

        self.cache[ip] = False
        return False

    def __len__(self):
        return self.networks.__len__()

def replace_address(ipmap, ip, offset, secret):
    if ip not in ipmap:
        assert len(ip) in (4, 16), 'ip address length is neither 4 nor 16'
        is_ip4 = len(ip) == 4

        # keyed-hash
        if secret:
            hmac_obj = hmac.new(secret, ip, hashlib.sha256)
            mapped = hmac_obj.digest()
            
        # first time seen
        else:
            newnum = len(ipmap) + 1
            packvalue = '>I' if is_ip4 else '>Q'
            mapped = struct.pack(packvalue, newnum)

        if is_ip4:
            # 1 offset byte + 3 bytes mapping
            ipmap[ip] = offset + mapped[-3:]

        else:
            # 1 offset byte + 7 null bytes + 8 bytes mapping
            ipmap[ip] = b'\x00' + offset + b'\x00'*6 + mapped[-8:]

    return ipmap[ip]

def has_transport_sum(ip):
    if not isinstance(ip.data, dpkt.udp.UDP) and not isinstance(ip.data, dpkt.tcp.TCP):
        return False

    if isinstance(ip, dpkt.ip.IP):
        return ip.offset == 0

    elif isinstance(ip, dpkt.ip6.IP6):
        if dpkt.ip.IP_PROTO_FRAGMENT not in ip.extension_hdrs:
            return False

        ext = ip.extension_hdrs[dpkt.ip.IP_PROTO_FRAGMENT]
        return ext.frag_off == 0

def main(inhandles, outfile, anonnets, offset, secret):
    pcap_writer = None

    ipmap = {}

    for fh in inhandles:
        pcap_reader = dpkt.pcap.Reader(fh)
        linktype = pcap_reader.datalink()

        if pcap_writer is None:
            fout = open(outfile, 'wb')
            pcap_writer = dpkt.pcap.Writer(fout, linktype=linktype)

        for ts, data in pcap_reader:
            if linktype == dpkt.pcap.DLT_EN10MB: # Ethernet
                frame = dpkt.ethernet.Ethernet(data)
                # clear MAC addresses
                frame.src = b'\x00' * len(frame.src)
                frame.dst = b'\x00' * len(frame.dst)
                packettype = frame.type
            elif linktype == dpkt.pcap.DLT_LINUX_SLL: # Linux cooked capture (SLL)
                frame = dpkt.sll.SLL(data)
                # clear MAC address
                frame.hdr = b'\x00' * len(frame.hdr)
                packettype = frame.ethtype
            elif linktype == dpkt.pcap.DLT_LINUX_SLL2: # Linux cooked capture v2 (SLL2)
                frame = dpkt.sll2.SLL2(data)
                # clear MAC address
                frame.hdr = b'\x00' * len(frame.hdr)
                packettype = frame.ethtype
            elif linktype == 101: # Raw IP dpkt.pcap.DLT_RAW
                frame = dpkt.ip.IP(data)
                packettype = dpkt.ethernet.ETH_TYPE_IP
            else:
                assert False, 'Linktype {} is not implemented yet'.format(linktype)

            if packettype not in (dpkt.ethernet.ETH_TYPE_IP, dpkt.ethernet.ETH_TYPE_IP6, dpkt.ip):
                if packettype not in IGNORE_ETHERTYPES:
                    print('WARNING: skipping unknown EtherType', hex(packettype), file=sys.stderr)
                    print('Are you using this network protocol or did the parser fail to read ethernet frames?', file=sys.stderr)
                    pcap_writer.writepkt(frame, ts)
                continue

            # Anonymize IPv4/IPv6 address
            if linktype == 101:
                ip = frame
            else:
                ip = frame.data
            reset_cksum = False
            if anonnets is None or ip.src in anonnets:
                ip.src = replace_address(ipmap, ip.src, offset, secret)
                reset_cksum = True
                
            if anonnets is None or ip.dst in anonnets:
                ip.dst = replace_address(ipmap, ip.dst, offset, secret)
                reset_cksum = True

            # Reset IP checksums. dpkt serializer will compute new values
            if reset_cksum:
                ip.sum = 0

                # Reset TCP/UDP checksums, if applicable
                if has_transport_sum(ip):
                    ip.data.sum = 0

            pcap_writer.writepkt(frame, ts)

        fh.close()

    if pcap_writer is None:
        print('No input data', file=sys.stderr)
    else:
        pcap_writer.close()
        print('Finished. Used', len(ipmap), 'IP address mappings.', file=sys.stderr)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Anonymize pcap file')
    parser.add_argument('-n', '--networksfile', help='File with line-by-line list of networks to be anonymized (if unset: anonymize all)')
    parser.add_argument('-o', '--offset', help='One byte offset for output IP address range (default: 238)', default=238, type=int)
    parser.add_argument('-s', '--secret', help='Secret for keyed-hash mapping (default: none)')
    parser.add_argument('-i', '--stdin', action='store_true', help='Read pcap input from stdin')
    parser.add_argument('infiles', nargs='*', help='Input pcap filename(s)')
    parser.add_argument('outfile', help='Output pcap filename')
    args = parser.parse_args()

    if args.infiles:
        print('Reading PCAP from', len(args.infiles), 'input files.', file=sys.stderr)
    else:
        print('No input files given.', file=sys.stderr)
    
    inhandles = [ open(infile, 'rb') for infile in args.infiles ]

    if args.stdin:
        if os.isatty(sys.stdin.buffer.fileno()):
            print('ERROR: Cannot read from interactive TTY. Use piped input instead.', file=sys.stderr)
            sys.exit(1)

        print('Reading PCAP from pipe over stdin.', file=sys.stderr)
        inhandles.append(sys.stdin.buffer)
        # If reading from stdin, ignore ctrl+c (will terminate when EOF has been reached)
        signal.signal(signal.SIGINT, signal.SIG_IGN)

    if args.outfile:
        if os.path.exists(args.outfile):
            print('ERROR: Output file exists already:', args.outfile, file=sys.stderr)
            sys.exit(1)

    if args.networksfile:
        anonnets = NetworkList(args.networksfile)
        print('Loaded', len(anonnets), 'networks to be anonymized.', file=sys.stderr)
    else:
        anonnets = None
        print('No networks file given, will anonymize all IP addresses.', file=sys.stderr)

    if not 0 < args.offset < 255:
        print('Offset must be 0-255, not', args.offset, file=sys.stderr)
        sys.exit(1)

    offset = struct.pack('B', args.offset)
    
    if args.secret:
        print('Got secret, using keyed-hash mapping.', file=sys.stderr)
        secret = args.secret.encode('utf-8')
    else:
        print('No secret given, using first time seen mapping.', file=sys.stderr)
        secret = None

    main(inhandles, args.outfile, anonnets, offset, secret)
