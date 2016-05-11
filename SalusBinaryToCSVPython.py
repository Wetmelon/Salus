#! /bin/env python3

import struct
import csv

infilename = '/tmp/Salus_Results_4.bin'
outfilename = '/tmp/Salus_Results.csv'

block_size = 512
recs_per_block = 7

rec_fmt = '<4BH5fB2f3d3f'
rec_len = struct.calcsize(rec_fmt)
assert rec_len == 71

block_end_padding = block_size - (recs_per_block * rec_len)
assert 0 <= block_end_padding < rec_len

fnames = (
    'dataVersion',
    'hour', 'minute', 'seconds', 'milliseconds',
    'latitude', 'longitude',
    'gpsSpeed', 'gpsAngle', 'gpsAltitude', 'satellites',
    'pressure', 'temperature',
    'xAccel', 'yAccel', 'zAccel',
    'xOrient', 'yOrient', 'zOrient',
)

with open(infilename, "rb") as binfile:
    with open(outfilename, 'w') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fnames)
        writer.writeheader()
        recs_read = 0
        while True:
            data = binfile.read(rec_len)
            if not data:
                break
            recs_read += 1
            if recs_read == recs_per_block:
                binfile.read(block_end_padding)
                recs_read = 0
            s = struct.Struct(rec_fmt).unpack_from(data)
            r = dict(zip(fnames, s))
            assert r['dataVersion'] == 0
            writer.writerow(r)
            # print("s=", s)