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

# Bin ADXL data into 1-second windows and compute means
print("=== ADXL371 data - 1-second windowed averages ===")
print(f"{'Window':>8} {'N':>5} {'X mean':>8} {'Y mean':>8} {'Z mean':>8} {'|a|':>8}")

adxl = decoder.adxl371_data
t_start = adxl[0][0]
t_end = adxl[-1][0]

for t in range(int(t_start), int(t_end) + 1):
    window = [d for d in adxl if t <= d[0] < t + 1]
    if not window:
        continue
    n = len(window)
    xm = sum(d[1] for d in window) / n
    ym = sum(d[2] for d in window) / n
    zm = sum(d[3] for d in window) / n
    mag = math.sqrt(xm**2 + ym**2 + zm**2)
    print(f"{t:>7}s {n:5d} {xm:8.3f} {ym:8.3f} {zm:8.3f} {mag:8.3f}")

# Compare ICM45686 at same time windows
print("\n=== ICM45686 data - 1-second windowed averages (reference) ===")
print(f"{'Window':>8} {'N':>5} {'X mean':>8} {'Y mean':>8} {'Z mean':>8} {'|a|':>8}")

icm = decoder.icm45686_data
t_start_icm = icm[0][0]
t_end_icm = icm[-1][0]

for t in range(int(t_start_icm), int(t_end_icm) + 1):
    window = [d for d in icm if t <= d[0] < t + 1]
    if not window:
        continue
    n = len(window)
    xm = sum(d[1] for d in window) / n
    ym = sum(d[2] for d in window) / n
    zm = sum(d[3] for d in window) / n
    mag = math.sqrt(xm**2 + ym**2 + zm**2)
    print(f"{t:>7}s {n:5d} {xm:8.3f} {ym:8.3f} {zm:8.3f} {mag:8.3f}")

# Now check the raw byte patterns at different time periods
# Flat: t=1-5, Side1 (X up): t=8-11, Side2 (Y down): t=14-16
print("\n=== Raw byte analysis per orientation ===")

for label, t_lo, t_hi in [("Flat (t=1-5)", 1, 5), ("Side1-Xup (t=8-11)", 8, 11), ("Side2-Ydn (t=14-16)", 14, 16), ("Flat2 (t=19-22)", 19, 22)]:
    indices = [i for i, d in enumerate(adxl) if t_lo <= d[0] < t_hi]
    if not indices:
        print(f"\n{label}: no data")
        continue
    
    raws = [decoder.adxl_raw[i] for i in indices]
    n = len(raws)
    
    # Compute mean of decoded values
    vals = [adxl[i] for i in indices]
    xm = sum(d[1] for d in vals) / n
    ym = sum(d[2] for d in vals) / n
    zm = sum(d[3] for d in vals) / n
    
    # Also compute mean raw 12-bit values
    def tc(val, bits):
        if (val & (1 << (bits - 1))) != 0:
            val = val - (1 << bits)
        return val
    
    x_raws = [tc((r[0]<<4)|(r[1]>>4), 12) for r in raws]
    y_raws = [tc((r[2]<<4)|(r[3]>>4), 12) for r in raws]
    z_raws = [tc((r[4]<<4)|(r[5]>>4), 12) for r in raws]
    
    xr_mean = sum(x_raws)/n
    yr_mean = sum(y_raws)/n
    zr_mean = sum(z_raws)/n
    
    # Show byte[0] histogram for each axis
    x_b0_counts = {}
    y_b0_counts = {}
    z_b0_counts = {}
    for r in raws:
        x_b0_counts[r[0]] = x_b0_counts.get(r[0], 0) + 1
        y_b0_counts[r[2]] = y_b0_counts.get(r[2], 0) + 1
        z_b0_counts[r[4]] = z_b0_counts.get(r[4], 0) + 1
    
    print(f"\n{label}: {n} packets")
    print(f"  Decoded means: X={xm:.3f}g, Y={ym:.3f}g, Z={zm:.3f}g")
    print(f"  Raw 12-bit means: X={xr_mean:.1f}, Y={yr_mean:.1f}, Z={zr_mean:.1f}")
    print(f"  X byte[0] histogram: {dict(sorted(x_b0_counts.items()))}")
    print(f"  Y byte[2] histogram: {dict(sorted(y_b0_counts.items()))}")
    print(f"  Z byte[4] histogram: {dict(sorted(z_b0_counts.items()))}")
    
    # Show first 5 raw packets
    print(f"  First 5 raw packets:")
    for i in indices[:5]:
        r = decoder.adxl_raw[i]
        hex_str = ' '.join(f'{r[j]:02X}' for j in range(6))
        print(f"    [{hex_str}] -> X={adxl[i][1]:.2f}g Y={adxl[i][2]:.2f}g Z={adxl[i][3]:.2f}g")
