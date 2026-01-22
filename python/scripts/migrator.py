import struct
import sys
import os

# Similar to decoder.py, but you know where you get a log file to know where the timestamp bytes are and switch the end bytes.
# identifier for each packet type
BMP581_ID = 'B'
ICM45686_ID = 'I'
MMC5983MA_ID = 'M'

# struct sizes in bytes (not counting timestamp and id bytes)
BMP581_SIZE = 6
ICM45686_SIZE = 15
MMC5983MA_SIZE = 7
HEADER_SIZE_TEXT = 14 # size of the "FIRM LOG vx.x" text
HEADER_UID_SIZE = 8
HEADER_DEVICE_NAME_LEN = 33
HEADER_COMM_SIZE = 2 # 1 byte for usb, 1 for uart
HEADER_CAL_SIZE = (3 + 9) * 3 * 4 # 3 offsets, 3x3 scale factor matrix, 3 sensors, 4 bytes per float
HEADER_NUM_SCALE_FACTORS = 5 # number of scale factor floats in the header

TIMESTAMP_SIZE = 3 # size of timestamp in bytes

num_repeat_whitespace = 0

# migrates a log file to have little-endian timestamp bytes
def new_file( path ):
    # define source(s) and destination(s) files ( for testing )
    dst_file = "LOGX_copy.txt"

    # opens source file and destination file
    try:
        # open source file for reading log and destination file for writing the migrated log
        with open( path, 'rb' ) as src, open( dst_file, "wb" ) as dst:
            # Checks if there is header info - skip/filler if not
            if( src.read( HEADER_SIZE_TEXT ) != b'FIRM LOG v1.0\n' ):
                return
            # reads and saves the header information
            header_text = src.read( HEADER_SIZE_TEXT )
            dst.write( header_text )
            uid_b = src.read( HEADER_UID_SIZE )
            dst.write( uid_b )
            device_name_b = src.read( HEADER_DEVICE_NAME_LEN)
            dst.write( device_name_b )
            comms_b = src.read( HEADER_COMM_SIZE )
            dst.write( comms_b )
            padding_bytes = 8 - ( HEADER_UID_SIZE + HEADER_DEVICE_NAME_LEN + HEADER_COMM_SIZE ) % 8
            padd = src.read( padding_bytes )
            dst.write( padd )

            # calibration data
            cal_bytes = src.read( HEADER_CAL_SIZE )
            dst.write( cal_bytes )

            # scale factors
            scale_factor_bytes = src.read( HEADER_NUM_SCALE_FACTORS * 4 )
            dst.write( scale_factor_bytes )

            # reads each packet
            while( True ):
                id_byte = src.read( 1 )
                # if end of file, exit
                if id_byte == 0:
                    # whitespace, skip
                    num_repeat_whitespace += 1
                    # end of data if whitespace repeats enough times
                    if num_repeat_whitespace > max([BMP581_SIZE, ICM45686_SIZE, MMC5983MA_SIZE]) + 4:
                        return False
                    return True
                num_repeat_whitespace = 0

                # save id_byte to destination file
                dst.write( id_byte )

                # reads specific packet based on id byte
                if id_byte == ord( BMP581_ID ):
                    pk_bytes = src.read( BMP581_SIZE )
                    dst.write( pk_bytes )
                elif id_byte == ord( ICM45686_ID ):
                    pk_bytes = src.read( ICM45686_SIZE )
                    dst.write( pk_bytes )
                elif id_byte == ord( MMC5983MA_ID ):
                    pk_bytes = src.read( MMC5983MA_SIZE )
                    dst.write( pk_bytes )

                # read timestamp bytes
                clock_count_bytes = src.read( TIMESTAMP_SIZE )

                if len(clock_count_bytes) != TIMESTAMP_SIZE:
                    break

                # write timestamp bytes in little-endian order
                dst.write( clock_count_bytes[::-1] )
    except Exception as e:
        print( "An error occurred: ", e )

# runs the migration and checks if command prompt arguments are valid
if __name__ == "__main__":
    if( len( sys.argv ) != 3 ):
        path = sys.argv[ 1 ]
        if os.path.exists(path):
            new_file( path )
        else:
            print("Invalid path to file")
    else:
        print( "Specify path of file to decode" )
        sys.exit( 1 )
