import struct, math
import sys
sys.path.insert(0, '.')
from decoder import *

class DebugDecoder(Decoder):
    adxl_raw = []
    def convert_adxl371(self, binary_packet):
        self.adxl_raw.append(bytes(binary_packet))
        return super().convert_adxl371(binary_packet)

with open('LOG26.FRM', 'rb') as f:
    decoder = DebugDecoder(f)
    while decoder.read_packet():
        continue

print(f"ADXL371 packets: {len(decoder.adxl371_data)}")
print(f"ICM45686 packets: {len(decoder.icm45686_data)}")

# Show ICM45686 accel data at a few timestamps to confirm orientation
print("\n=== ICM45686 accelerometer (known good) - sampled ===")
print(f"{'Time':>8} {'X(g)':>8} {'Y(g)':>8} {'Z(g)':>8} {'|a|(g)':>8}")
step = max(1, len(decoder.icm45686_data) // 20)
for i in range(0, min(len(decoder.icm45686_data), 20*step), step):
    ts, ax, ay, az = decoder.icm45686_data[i][:4]
    mag = math.sqrt(ax**2 + ay**2 + az**2)
    print(f"{ts:8.2f} {ax:8.3f} {ay:8.3f} {az:8.3f} {mag:8.3f}")

# Now try MANY different interpretations of the ADXL371 raw bytes
print("\n=== Testing different ADXL371 byte interpretations (first 20 packets) ===")
raw_list = decoder.adxl_raw[:20]

def tc(val, bits):
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

interpretations = {}

for name, decode_fn in [
    ("Current: (b0<<4)|(b1>>4)", 
     lambda b, i: tc((b[i*2] << 4) | (b[i*2+1] >> 4), 12)),
    ("Swapped: (b1<<4)|(b0>>4)", 
     lambda b, i: tc((b[i*2+1] << 4) | (b[i*2] >> 4), 12)),
    ("Just byte[0] as signed 8-bit", 
     lambda b, i: tc(b[i*2], 8)),
    ("Just byte[1]>>4 as signed 4-bit", 
     lambda b, i: tc(b[i*2+1] >> 4, 4)),
    ("BE16>>4 (same as current)", 
     lambda b, i: tc(((b[i*2] << 8) | b[i*2+1]) >> 4, 12)),
    ("LE16>>4", 
     lambda b, i: tc(((b[i*2+1] << 8) | b[i*2]) >> 4, 12)),
    ("Interleaved: b[0],b[2],b[4] high; b[1]={x3:0,y3:0}; b[3]={z3:0,0}",
     None),  # special case
    ("Interleaved v2: b[0],b[1],b[2] = X,Y,Z high; b[3]={X,Y low}; b[4]={Z low,0}",
     None),  # special case
]:
    if decode_fn is not None:
        vals = []
        for raw in raw_list:
            x = decode_fn(raw, 0)
            y = decode_fn(raw, 1)
            z = decode_fn(raw, 2)
            vals.append((x, y, z))
        interpretations[name] = vals

# Special interleaved interpretation 1
# byte[0]=X[11:4], byte[2]=Y[11:4], byte[4]=Z[11:4]
# byte[1]={X[3:0], Y[3:0]}, byte[3]={Z[3:0], 0000}, byte[5]=status
vals = []
for raw in raw_list:
    x = tc((raw[0] << 4) | (raw[1] >> 4), 12)
    y = tc((raw[2] << 4) | (raw[1] & 0x0F), 12)
    z = tc((raw[4] << 4) | (raw[3] >> 4), 12)
    vals.append((x, y, z))
interpretations["Interleaved: b[0],b[2],b[4] high; b[1]={x3:0,y3:0}; b[3]={z3:0,0}"] = vals

# Special interleaved interpretation 2
# byte[0]=X[11:4], byte[1]=Y[11:4], byte[2]=Z[11:4]
# byte[3]={X[3:0], Y[3:0]}, byte[4]={Z[3:0], 0000}
vals = []
for raw in raw_list:
    x = tc((raw[0] << 4) | (raw[3] >> 4), 12)
    y = tc((raw[1] << 4) | (raw[3] & 0x0F), 12)
    z = tc((raw[2] << 4) | (raw[4] >> 4), 12)
    vals.append((x, y, z))
interpretations["Interleaved v2: b[0],b[1],b[2] = X,Y,Z high; b[3]={X,Y low}; b[4]={Z low,0}"] = vals

# ICM45686 interleaved style: X_H,X_L,Y_H,Y_L,Z_H,Z_L but with low nibbles packed in extra bytes
# Actually try: high bytes are b[0],b[1],b[2], low nibbles in b[3],b[4],b[5]
vals = []
for raw in raw_list:
    x = tc((raw[0] << 4) | (raw[3] >> 4), 12)
    y = tc((raw[1] << 4) | (raw[4] >> 4), 12)
    z = tc((raw[2] << 4) | (raw[5] >> 4), 12)
    vals.append((x, y, z))
interpretations["ICM-style: b[0:3] high, b[3:6]>>4 low"] = vals

# Also try with low nibble being in low position
vals = []
for raw in raw_list:
    x = tc((raw[0] << 4) | (raw[3] & 0x0F), 12)
    y = tc((raw[1] << 4) | (raw[4] & 0x0F), 12)
    z = tc((raw[2] << 4) | (raw[5] & 0x0F), 12)
    vals.append((x, y, z))
interpretations["ICM-style v2: b[0:3] high, b[3:6]&0xF low"] = vals

# Evaluate each interpretation: compute mean and stddev for first 20 samples
print(f"\n{'Interpretation':<65} {'X mean':>8} {'Y mean':>8} {'Z mean':>8} {'X std':>8} {'Y std':>8} {'Z std':>8} {'|a| mean':>8}")
print("-" * 145)

for name, vals in interpretations.items():
    sf = 10.24
    xs = [v[0]/sf for v in vals]
    ys = [v[1]/sf for v in vals]
    zs = [v[2]/sf for v in vals]
    
    x_mean = sum(xs)/len(xs)
    y_mean = sum(ys)/len(ys)
    z_mean = sum(zs)/len(zs)
    
    x_std = (sum((x - x_mean)**2 for x in xs) / len(xs)) ** 0.5
    y_std = (sum((y - y_mean)**2 for y in ys) / len(ys)) ** 0.5
    z_std = (sum((z - z_mean)**2 for z in zs) / len(zs)) ** 0.5
    
    mags = [math.sqrt(x**2+y**2+z**2) for x,y,z in zip(xs,ys,zs)]
    mag_mean = sum(mags)/len(mags)
    
    print(f"{name:<65} {x_mean:8.3f} {y_mean:8.3f} {z_mean:8.3f} {x_std:8.3f} {y_std:8.3f} {z_std:8.3f} {mag_mean:8.3f}")

# Now use first 500 samples for more stable statistics
print(f"\n=== Statistics over first 500 samples ===")
print(f"{'Interpretation':<65} {'X mean':>8} {'Y mean':>8} {'Z mean':>8} {'X std':>8} {'Y std':>8} {'Z std':>8} {'|a| mean':>8}")
print("-" * 145)

raw500 = decoder.adxl_raw[:500]
for name, decode_axes in [
    ("Current: (b0<<4)|(b1>>4)", 
     lambda b: (tc((b[0]<<4)|(b[1]>>4),12), tc((b[2]<<4)|(b[3]>>4),12), tc((b[4]<<4)|(b[5]>>4),12))),
    ("ICM-style: b[0:3] high, b[3:6]>>4 low",
     lambda b: (tc((b[0]<<4)|(b[3]>>4),12), tc((b[1]<<4)|(b[4]>>4),12), tc((b[2]<<4)|(b[5]>>4),12))),
    ("ICM-style v2: b[0:3] high, b[3:6]&0xF low",
     lambda b: (tc((b[0]<<4)|(b[3]&0xF),12), tc((b[1]<<4)|(b[4]&0xF),12), tc((b[2]<<4)|(b[5]&0xF),12))),
    ("Interleaved v2: b[0],b[1],b[2]=XYZ high; b[3]={X,Y low}; b[4]={Z low}",
     lambda b: (tc((b[0]<<4)|(b[3]>>4),12), tc((b[1]<<4)|(b[3]&0xF),12), tc((b[2]<<4)|(b[4]>>4),12))),
]:
    sf = 10.24
    vals = [decode_axes(r) for r in raw500]
    xs = [v[0]/sf for v in vals]
    ys = [v[1]/sf for v in vals]
    zs = [v[2]/sf for v in vals]
    
    n = len(vals)
    x_mean = sum(xs)/n
    y_mean = sum(ys)/n
    z_mean = sum(zs)/n
    x_std = (sum((x-x_mean)**2 for x in xs)/n)**0.5
    y_std = (sum((y-y_mean)**2 for y in ys)/n)**0.5
    z_std = (sum((z-z_mean)**2 for z in zs)/n)**0.5
    mags = [math.sqrt(x**2+y**2+z**2) for x,y,z in zip(xs,ys,zs)]
    mag_mean = sum(mags)/n
    
    print(f"{name:<65} {x_mean:8.3f} {y_mean:8.3f} {z_mean:8.3f} {x_std:8.3f} {y_std:8.3f} {z_std:8.3f} {mag_mean:8.3f}")
