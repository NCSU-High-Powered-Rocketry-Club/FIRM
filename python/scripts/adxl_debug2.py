import struct
import sys
sys.path.insert(0, '.')
from decoder import *

# Patch the convert_adxl371 to also save raw bytes
class DebugDecoder(Decoder):
    adxl_raw = []
    
    def convert_adxl371(self, binary_packet):
        self.adxl_raw.append(binary_packet)
        return super().convert_adxl371(binary_packet)

with open('LOG26.FRM', 'rb') as f:
    decoder = DebugDecoder(f)
    while decoder.read_packet():
        continue

print(f"Total ADXL371 packets: {len(decoder.adxl371_data)}")
print(f"\nFirst 30 ADXL371 packets - raw bytes and decoded values:")
print(f"{'#':>4} {'Raw Hex':>20} {'X raw':>6} {'Y raw':>6} {'Z raw':>6} {'X (g)':>8} {'Y (g)':>8} {'Z (g)':>8}")
print("-" * 80)

for i in range(min(30, len(decoder.adxl371_data))):
    raw = decoder.adxl_raw[i]
    hex_str = ' '.join(f'{b:02X}' for b in raw)
    
    x_raw = (raw[0] << 4) | (raw[1] >> 4)
    y_raw = (raw[2] << 4) | (raw[3] >> 4)
    z_raw = (raw[4] << 4) | (raw[5] >> 4)
    x_tc = twos_complement(x_raw, 12)
    y_tc = twos_complement(y_raw, 12)
    z_tc = twos_complement(z_raw, 12)
    
    ts, xg, yg, zg = decoder.adxl371_data[i]
    print(f"{i:4d} {hex_str:>20} {x_tc:6d} {y_tc:6d} {z_tc:6d} {xg:8.2f} {yg:8.2f} {zg:8.2f}")

# Also show stats for first 100 samples (should be flat on table)
print(f"\n--- Statistics for first 100 samples (should be flat on table) ---")
n = min(100, len(decoder.adxl371_data))
xs = [decoder.adxl371_data[i][1] for i in range(n)]
ys = [decoder.adxl371_data[i][2] for i in range(n)]
zs = [decoder.adxl371_data[i][3] for i in range(n)]
print(f"X: mean={sum(xs)/n:.3f}g, min={min(xs):.3f}g, max={max(xs):.3f}g")
print(f"Y: mean={sum(ys)/n:.3f}g, min={min(ys):.3f}g, max={max(ys):.3f}g")
print(f"Z: mean={sum(zs)/n:.3f}g, min={min(zs):.3f}g, max={max(zs):.3f}g")

# Also check: are the lower nibbles of byte[1], byte[3], byte[5] always 0?
non_zero_lower = 0
for raw in decoder.adxl_raw[:100]:
    if (raw[1] & 0x0F) != 0 or (raw[3] & 0x0F) != 0 or (raw[5] & 0x0F) != 0:
        non_zero_lower += 1
print(f"\nPackets with non-zero lower nibble in byte[1]/[3]/[5]: {non_zero_lower}/100")

# Show a sampling across the full dataset to see the flat->side->otherside transition
total = len(decoder.adxl371_data)
print(f"\n--- Sampled data across full dataset ({total} packets) ---")
print(f"{'#':>6} {'Time (s)':>10} {'X (g)':>8} {'Y (g)':>8} {'Z (g)':>8}")
step = max(1, total // 30)
for i in range(0, total, step):
    ts, xg, yg, zg = decoder.adxl371_data[i]
    print(f"{i:6d} {ts:10.2f} {xg:8.2f} {yg:8.2f} {zg:8.2f}")
