#!/usr/bin/env python3
# OpenSky aircraftDatabase.csv -> compact binary (assets/aircraft_db.bin)
#   record = reg[8] + type[4] + u16 op_id  (14B), sorted by reg for binary search
import csv, struct, sys
SRC = sys.argv[1] if len(sys.argv)>1 else '/tmp/aircraftDatabase.csv'
OUT = sys.argv[2] if len(sys.argv)>2 else 'assets/aircraft_db.bin'
def norm(r): return ''.join(c for c in (r or '').upper() if c.isalnum())[:8]
ops={'' :0}; op_list=['']            # id 0 = empty operator
rows=[]
with open(SRC, newline='', encoding='utf-8', errors='replace') as f:
    for row in csv.DictReader(f):
        reg=norm(row.get('registration'))
        if not reg: continue
        typ=((row.get('typecode') or row.get('icaoaircrafttype') or '').strip().upper())[:4]
        op=(row.get('operator') or row.get('operatorcallsign') or '').strip()[:31]
        if not typ and not op: continue
        if op not in ops:
            if len(op_list) < 65535:
                ops[op]=len(op_list); op_list.append(op)
            else: op=''
        rows.append((reg, typ, ops.get(op,0)))
rows.sort(key=lambda x:x[0])
uniq=[]; last=None
for r in rows:
    if r[0]!=last: uniq.append(r); last=r[0]
rows=uniq
blob=bytearray(); offs=[]
for name in op_list:
    offs.append(len(blob)); blob+=name.encode('utf-8','replace')+b'\x00'
with open(OUT,'wb') as o:
    o.write(b'ABDB'); o.write(struct.pack('<II', len(rows), len(op_list)))
    for reg,typ,opid in rows:
        o.write(reg.encode('ascii','replace').ljust(8,b'\x00')[:8])
        o.write(typ.encode('ascii','replace').ljust(4,b'\x00')[:4])
        o.write(struct.pack('<H', opid))
    for off in offs: o.write(struct.pack('<I', off))
    o.write(blob)
import os
print(f"records={len(rows)} operators={len(op_list)} size={os.path.getsize(OUT)/1e6:.1f}MB -> {OUT}")
