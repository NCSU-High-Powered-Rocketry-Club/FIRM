#!/usr/bin/env python3
"""
Simple test script for STM32 I2C communication
Tests basic connectivity and reads raw data
"""

from smbus2 import SMBus, i2c_msg


def test_device_detection():
    """Test if STM32 is present on I2C bus"""
    print("Testing I2C device detection...")
    
    try:
        with SMBus(1) as bus:
            # Try to read 1 byte to test presence
            msg = i2c_msg.read(0x48, 1)
            bus.i2c_rdwr(msg)
            print("✓ STM32 detected at address 0x48")
            return True
    except Exception as e:
        print(f"✗ Device not found: {e}")
        return False


def test_single_byte_read():
    """Test single byte read"""
    print("\nTesting single byte read...")
    
    try:
        with SMBus(1) as bus:
            msg = i2c_msg.read(0x48, 1)
            bus.i2c_rdwr(msg)
            data = bytes(msg)
            print(f"✓ Read 1 byte: 0x{data[0]:02X}")
            return True
    except Exception as e:
        print(f"✗ Single byte read failed: {e}")
        return False


def test_multi_byte_read():
    """Test reading 2 bytes (header)"""
    print("\nTesting 2-byte read (header)...")
    
    try:
        with SMBus(1) as bus:
            msg = i2c_msg.read(0x48, 2)
            bus.i2c_rdwr(msg)
            data = bytes(msg)
            header = (data[1] << 8) | data[0]  # Little-endian
            print(f"✓ Read 2 bytes: {data.hex()}")
            print(f"  Header value: 0x{header:04X} (expected 0xA55A)")
            return header == 0xA55A
    except Exception as e:
        print(f"✗ 2-byte read failed: {e}")
        return False


def test_full_packet_read():
    """Test reading full 58-byte packet"""
    print("\nTesting full packet read (58 bytes)...")
    
    try:
        with SMBus(1) as bus:
            msg = i2c_msg.read(0x48, 58)
            bus.i2c_rdwr(msg)
            data = bytes(msg)
            print(f"✓ Read {len(data)} bytes")
            print(f"  Data (hex): {data.hex()}")
            
            # Parse header
            header = (data[1] << 8) | data[0]
            length = (data[3] << 8) | data[2]
            print(f"  Header: 0x{header:04X}")
            print(f"  Length: {length} bytes")
            
            # Parse CRC (last 2 bytes)
            crc = (data[57] << 8) | data[56]
            print(f"  CRC: 0x{crc:04X}")
            
            return True
    except Exception as e:
        print(f"✗ Full packet read failed: {e}")
        return False


def test_repeated_reads():
    """Test multiple consecutive reads"""
    print("\nTesting repeated reads (5 times)...")
    
    success_count = 0
    for i in range(5):
        try:
            with SMBus(1) as bus:
                msg = i2c_msg.read(0x48, 2)
                bus.i2c_rdwr(msg)
                data = bytes(msg)
                print(f"  Read {i+1}: {data.hex()}")
                success_count += 1
        except Exception as e:
            print(f"  Read {i+1}: Failed - {e}")
    
    print(f"✓ {success_count}/5 reads successful")
    return success_count == 5


def main():
    """Run all tests"""
    print("="*60)
    print("STM32 I2C Communication Test Suite")
    print("="*60)
    
    tests = [
        ("Device Detection", test_device_detection),
        ("Single Byte Read", test_single_byte_read),
        ("Multi-byte Read", test_multi_byte_read),
        ("Full Packet Read", test_full_packet_read),
        ("Repeated Reads", test_repeated_reads),
    ]
    
    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"\n✗ Test '{name}' crashed: {e}")
            results.append((name, False))
    
    # Summary
    print("\n" + "="*60)
    print("Test Summary:")
    print("="*60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {name}")
    
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n✓ All tests passed! I2C communication is working correctly.")
    else:
        print(f"\n✗ {total - passed} test(s) failed. Check connections and STM32 firmware.")


if __name__ == "__main__":
    main()
