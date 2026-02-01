#ifndef FF_DEFINED
#define FF_DEFINED

#include <stdint.h>

typedef char TCHAR;
typedef uint8_t BYTE;
typedef uint32_t UINT;
typedef uint32_t DWORD;
typedef uint16_t WORD;
typedef uint64_t FSIZE_t;

/* Result Codes */
typedef enum {
    FR_OK = 0,
    FR_DISK_ERR,
    FR_INT_ERR,
    FR_NOT_READY,
    FR_NO_FILE,
    FR_NO_PATH,
    FR_INVALID_NAME,
    FR_DENIED,
    FR_EXIST,
    FR_INVALID_OBJECT,
    FR_WRITE_PROTECTED,
    FR_INVALID_DRIVE,
    FR_NOT_ENABLED,
    FR_NO_FILESYSTEM,
    FR_MKFS_ABORTED,
    FR_TIMEOUT,
    FR_LOCKED,
    FR_NOT_ENOUGH_CORE,
    FR_TOO_MANY_OPEN_FILES,
    FR_INVALID_PARAMETER
} FRESULT;

/* File Access Flags */
#define FA_READ 0x01
#define FA_WRITE 0x02
#define FA_OPEN_EXISTING 0x00
#define FA_CREATE_NEW 0x04
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_ALWAYS 0x10
#define FA_OPEN_APPEND 0x30

/* Dummy Structures (Content doesn't matter, just the name) */
typedef struct { int dummy; } FATFS;
typedef struct { int dummy; } FIL;
typedef struct { int dummy; } DIR;
typedef struct { 
    DWORD fsize; 
    char fname[13]; 
} FILINFO;

/* Prototypes to Mock */
FRESULT f_open (FIL* fp, const TCHAR* path, BYTE mode);
FRESULT f_close (FIL* fp);
FRESULT f_mount (FATFS* fs, const TCHAR* path, BYTE opt);
FRESULT f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern);
FRESULT f_truncate (FIL* fp);
FRESULT f_sync (FIL* fp);
FRESULT f_expand (FIL* fp, FSIZE_t fsz, BYTE opt);
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);

#endif