import struct
import sys
import os
import tempfile
import shutil
from typing import Callable, Dict
from decoder import (
    BMP581_ID,
    ICM45686_ID,
    MMC5983MA_ID,
    BMP581_SIZE,
    ICM45686_SIZE,
    MMC5983MA_SIZE,
)


LOG_HEADER_TEXT_SIZE = 14

V10_TEXT = b'FIRM LOG v1.0\n'
V11_TEXT = b'FIRM LOG v1.1\n'
V12_TEXT = b'FIRM LOG v1.2\n'

V10_TIMESTAMP_BYTES = 3  # big endian
V11_TIMESTAMP_BYTES = 3  # little endian
V12_TIMESTAMP_BYTES = 4  # little endian

V11_UID_SIZE = 8
V11_DEVICE_NAME_LEN = 33
V11_PROTOCOL_BOOL_BYTES = 2
V11_PADDING_BYTES = 5
V11_CAL_BYTES = 144
V11_NUM_SCALE_FACTORS = 5

V12_UID_SIZE = 8
V12_DEVICE_NAME_LEN = 32
V12_PROTOCOL_BOOL_BYTES = 4
V12_FIRMWARE_VERSION_LEN = 8
V12_FREQUENCY_LEN = 2
V12_PADDING_BYTES = 2
V12_CAL_BYTES = 144
V12_NUM_SCALE_FACTORS = 5


def default_calibration_bytes() -> bytes:
    # calibration data
    cal_grid = [ 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
                 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
                 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 ]
    num_cal_grid = len( cal_grid )
    format_str = f'<{num_cal_grid}f'
    return struct.pack( format_str, *cal_grid )


def migrate_header_v1_0(src, dst) -> None:
    #adds default values for the header information
    uid_b = b'1' * V11_UID_SIZE
    dst.write( uid_b )
    # Device name is a fixed-length field of 33 bytes
    device_name_b = b'driver0'.ljust(V11_DEVICE_NAME_LEN, b'\x00')
    dst.write( device_name_b )
    comms_b = b'\x01\x00'
    dst.write( comms_b )
    padding = b'\x00' * V11_PADDING_BYTES
    dst.write(padding)

    dst.write(default_calibration_bytes())

    # Scale factors
    scale_factor_bytes = src.read( V11_NUM_SCALE_FACTORS * 4 )
    dst.write(scale_factor_bytes)


def migrate_header_v1_1(src, dst) -> None:
    # reads and saves the header information
    uid_b = src.read( V11_UID_SIZE )
    dst.write( uid_b )
    device_name_b = src.read(V11_DEVICE_NAME_LEN)
    dst.write( device_name_b )
    comms_b = src.read( V11_PROTOCOL_BOOL_BYTES )
    dst.write( comms_b )
    padding_bytes = V11_PADDING_BYTES
    padd = src.read( padding_bytes )
    dst.write( padd )

    # calibration data
    cal_bytes = src.read( V11_CAL_BYTES )
    dst.write( cal_bytes )

    # scale factors
    scale_factor_bytes = src.read( V11_NUM_SCALE_FACTORS * 4 )
    dst.write(scale_factor_bytes)


def migrate_packets_swap_timestamp_endianness(src, dst, *, src_timestamp_bytes: int, dst_timestamp_bytes: int) -> None:
    # reads each packet
    if dst_timestamp_bytes != src_timestamp_bytes:
        return
    num_repeat_whitespace = 0
    while( True ):
        id_byte = src.read( 1 )
        # if end of file, exit
        if len(id_byte) == 0:
            break
        if id_byte[0] == 0:
            num_repeat_whitespace += 1
            # end of data if whitespace repeats enough times
            if num_repeat_whitespace > max([BMP581_SIZE, ICM45686_SIZE, MMC5983MA_SIZE]) + 4:
                break
            continue
        num_repeat_whitespace = 0

        # save id_byte to destination file
        dst.write( id_byte )

        # read timestamp bytes
        clock_count_bytes = src.read(src_timestamp_bytes)
        if len(clock_count_bytes) != src_timestamp_bytes:
            break
        # write timestamp bytes in little-endian order
        dst.write( clock_count_bytes[::-1] )

        # reads specific packet based on id byte
        if id_byte[0] == ord( BMP581_ID ):
            pk_bytes = src.read( BMP581_SIZE )
            dst.write( pk_bytes )
        elif id_byte[0] == ord( ICM45686_ID ):
            pk_bytes = src.read( ICM45686_SIZE )
            dst.write( pk_bytes )
        elif id_byte[0] == ord( MMC5983MA_ID ):
            pk_bytes = src.read( MMC5983MA_SIZE )
            dst.write( pk_bytes )


def copy_packets_no_timestamp_swap(src, dst, *, src_timestamp_bytes: int, dst_timestamp_bytes: int) -> None:
    # reads each packet
    num_repeat_whitespace = 0

    # When converting v1.1 (24-bit timestamps) to v1.2 (32-bit timestamps),
    # reconstruct a 32-bit counter by accumulating positive deltas modulo 2^24.
    clock_count_32 = None

    while( True ):
        id_byte = src.read( 1 )
        # if end of file, exit
        if len(id_byte) == 0:
            break
        if id_byte[0] == 0:
            num_repeat_whitespace += 1
            # end of data if whitespace repeats enough times
            if num_repeat_whitespace > max([BMP581_SIZE, ICM45686_SIZE, MMC5983MA_SIZE]) + 4:
                break
            continue
        num_repeat_whitespace = 0

        # save id_byte to destination file
        dst.write( id_byte )

        # read timestamp bytes
        clock_count_bytes = src.read(src_timestamp_bytes)
        if len(clock_count_bytes) != src_timestamp_bytes:
            break

        if dst_timestamp_bytes == src_timestamp_bytes:
            dst.write( clock_count_bytes )
        elif dst_timestamp_bytes == src_timestamp_bytes + 1:
            if src_timestamp_bytes == 3 and dst_timestamp_bytes == 4:
                cur24 = int.from_bytes(clock_count_bytes, byteorder='little', signed=False)
                if clock_count_32 is None:
                    clock_count_32 = cur24
                else:
                    prev24 = clock_count_32 & 0xFFFFFF
                    delta24 = (cur24 - prev24) & 0xFFFFFF
                    clock_count_32 = (clock_count_32 + delta24) & 0xFFFFFFFF

                dst.write(clock_count_32.to_bytes(4, byteorder='little', signed=False))
            else:
                # Preserve the numeric value by adding a zero MSB.
                dst.write( clock_count_bytes + b'\x00' )
        else:
            return

        # reads specific packet based on id byte
        if id_byte[0] == ord( BMP581_ID ):
            pk_bytes = src.read( BMP581_SIZE )
            dst.write( pk_bytes )
        elif id_byte[0] == ord( ICM45686_ID ):
            pk_bytes = src.read( ICM45686_SIZE )
            dst.write( pk_bytes )
        elif id_byte[0] == ord( MMC5983MA_ID ):
            pk_bytes = src.read( MMC5983MA_SIZE )
            dst.write( pk_bytes )


HEADER_MIGRATORS: Dict[bytes, Callable] = {
    V10_TEXT: migrate_header_v1_0,
    V11_TEXT: migrate_header_v1_1,
}


def convert_to_v1_1(src, dst, header_text: bytes) -> None:
    dst.write(V11_TEXT)
    migrator = HEADER_MIGRATORS.get(header_text)
    if migrator is None:
        return
    migrator(src, dst)
    migrate_packets_swap_timestamp_endianness(
        src,
        dst,
        src_timestamp_bytes=V10_TIMESTAMP_BYTES,
        dst_timestamp_bytes=V11_TIMESTAMP_BYTES,
    )


def convert_v1_1_to_v1_2(src, dst) -> None:
    # converted files will be version 1.2
    src_header_text = src.read( LOG_HEADER_TEXT_SIZE )
    if src_header_text != V11_TEXT:
        return

    dst.write(V12_TEXT)

    # v1.2 header layout differs from v1.1
    uid_b = src.read(V11_UID_SIZE)
    dst.write(uid_b)

    device_name_b = src.read(V11_DEVICE_NAME_LEN)
    dst.write(device_name_b[:V12_DEVICE_NAME_LEN])

    comms_b = src.read(V11_PROTOCOL_BOOL_BYTES)
    dst.write(comms_b + b'\x00\x00')

    firmware_b = b'v1.0.0'.ljust(V12_FIRMWARE_VERSION_LEN, b' ')
    dst.write(firmware_b)

    frequency_b = struct.pack('<H', 100)
    dst.write(frequency_b)

    dst.write(b'\x00' * V12_PADDING_BYTES)

    cal_bytes = src.read(V11_PADDING_BYTES + V11_CAL_BYTES)
    # v1.1 has 5 bytes padding before calibration
    if len(cal_bytes) < (V11_PADDING_BYTES + V11_CAL_BYTES):
        return
    dst.write(cal_bytes[V11_PADDING_BYTES:])

    scale_factor_bytes = src.read(V11_NUM_SCALE_FACTORS * 4)
    dst.write(scale_factor_bytes)

    copy_packets_no_timestamp_swap(
        src,
        dst,
        src_timestamp_bytes=V11_TIMESTAMP_BYTES,
        dst_timestamp_bytes=V12_TIMESTAMP_BYTES,
    )


# migrates a log file to have little-endian timestamp bytes
def new_file( path ):
    # define source(s) and destination(s) files ( for testing )
    dst_file = "migrated_log.FIRM"

    # opens source file and destination file
    try:
        # open source file for reading log and destination file for writing the migrated log
        with open( path, 'rb' ) as src, open( dst_file, "wb" ) as dst:
            # read header version
            header_text = src.read( LOG_HEADER_TEXT_SIZE )

            if( header_text == V12_TEXT ):
                # v1.2 stays as-is
                src.seek(0)
                shutil.copyfileobj(src, dst)
                return

            if( header_text == V11_TEXT ):
                # Incremental: v1.1 -> v1.2
                src.seek(0)
                convert_v1_1_to_v1_2(src, dst)
                return

            if( header_text == V10_TEXT ):
                # Incremental: v1.0 -> v1.1, then v1.1 -> v1.2
                with tempfile.TemporaryFile(mode="w+b") as tmp:
                    convert_to_v1_1(src, tmp, header_text)
                    tmp.seek(0)
                    convert_v1_1_to_v1_2(tmp, dst)
                return

            return

    except Exception as e:
        print( "An error occurred: ", e )


# runs the migration and checks if command prompt arguments are valid
if __name__ == "__main__":
    if( len( sys.argv ) == 2 ):
        path = sys.argv[ 1 ]
        if os.path.exists(path):
            new_file( path )
        else:
            print("Invalid path to file")
    else:
        print( "Specify path of file to decode" )
        sys.exit( 1 )

