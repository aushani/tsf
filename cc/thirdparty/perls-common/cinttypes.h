#ifndef __PERLS_COMMON_CINTTYPES_H__
#define __PERLS_COMMON_CINTTYPES_H__

/*
 *	ISO C99: 7.8 Format conversion of integer types	similar to <inttypes.h>
 */

/**
 * @defgroup PerlsCommonCinttypes Inttypes Convenience Utility for C++ Source
 * @brief C++ inttypes library.
 * @ingroup PerlsCommon
 * @include: perls-common/cinttypes.h
 *
 * @{
 */


# if __WORDSIZE == 64
#   define __cPRI64_PREFIX       "l"
#   define __cPRIPTR_PREFIX      "l"
# else
#   define __cPRI64_PREFIX       "ll"
#   define __cPRIPTR_PREFIX
# endif

/* Macros for printing format specifiers.  */

/* Decimal notation.  */
#define cPRId8         "d"
#define cPRId16        "d"
#define cPRId32        "d"
#define cPRId64        __cPRI64_PREFIX "d"

#define cPRIdLEAST8    "d"
#define cPRIdLEAST16   "d"
#define cPRIdLEAST32   "d"
#define cPRIdLEAST64   __cPRI64_PREFIX "d"

#define cPRIdFAST8     "d"
#define cPRIdFAST16    __cPRIPTR_PREFIX "d"
#define cPRIdFAST32    __cPRIPTR_PREFIX "d"
#define cPRIdFAST64    __cPRI64_PREFIX "d"


#define cPRIi8         "i"
#define cPRIi16        "i"
#define cPRIi32        "i"
#define cPRIi64        __cPRI64_PREFIX "i"

#define cPRIiLEAST8    "i"
#define cPRIiLEAST16   "i"
#define cPRIiLEAST32   "i"
#define cPRIiLEAST64   __cPRI64_PREFIX "i"

#define cPRIiFAST8     "i"
#define cPRIiFAST16    __cPRIPTR_PREFIX "i"
#define cPRIiFAST32    __cPRIPTR_PREFIX "i"
#define cPRIiFAST64    __cPRI64_PREFIX "i"

/* Octal notation.  */
#define cPRIo8         "o"
#define cPRIo16        "o"
#define cPRIo32        "o"
#define cPRIo64        __cPRI64_PREFIX "o"

#define cPRIoLEAST8    "o"
#define cPRIoLEAST16   "o"
#define cPRIoLEAST32   "o"
#define cPRIoLEAST64   __cPRI64_PREFIX "o"

#define cPRIoFAST8     "o"
#define cPRIoFAST16    __cPRIPTR_PREFIX "o"
#define cPRIoFAST32    __cPRIPTR_PREFIX "o"
#define cPRIoFAST64    __cPRI64_PREFIX "o"

/* Unsigned integers.  */
#define cPRIu8         "u"
#define cPRIu16        "u"
#define cPRIu32        "u"
#define cPRIu64        __cPRI64_PREFIX "u"

#define cPRIuLEAST8    "u"
#define cPRIuLEAST16   "u"
#define cPRIuLEAST32   "u"
#define cPRIuLEAST64   __cPRI64_PREFIX "u"

#define cPRIuFAST8     "u"
#define cPRIuFAST16    __cPRIPTR_PREFIX "u"
#define cPRIuFAST32    __cPRIPTR_PREFIX "u"
#define cPRIuFAST64    __cPRI64_PREFIX "u"

/* lowercase hexadecimal notation.  */
#define cPRIx8         "x"
#define cPRIx16        "x"
#define cPRIx32        "x"
#define cPRIx64        __cPRI64_PREFIX "x"

#define cPRIxLEAST8    "x"
#define cPRIxLEAST16   "x"
#define cPRIxLEAST32   "x"
#define cPRIxLEAST64   __cPRI64_PREFIX "x"

#define cPRIxFAST8     "x"
#define cPRIxFAST16    __cPRIPTR_PREFIX "x"
#define cPRIxFAST32    __cPRIPTR_PREFIX "x"
#define cPRIxFAST64    __cPRI64_PREFIX "x"

/* UPPERCASE hexadecimal notation.  */
#define cPRIX8         "X"
#define cPRIX16        "X"
#define cPRIX32        "X"
#define cPRIX64        __cPRI64_PREFIX "X"

#define cPRIXLEAST8    "X"
#define cPRIXLEAST16   "X"
#define cPRIXLEAST32   "X"
#define cPRIXLEAST64   __cPRI64_PREFIX "X"

#define cPRIXFAST8     "X"
#define cPRIXFAST16    __cPRIPTR_PREFIX "X"
#define cPRIXFAST32    __cPRIPTR_PREFIX "X"
#define cPRIXFAST64    __cPRI64_PREFIX "X"


/* Macros for printing `intmax_t' and `uintmax_t'.  */
#define cPRIdMAX       __cPRI64_PREFIX "d"
#define cPRIiMAX       __cPRI64_PREFIX "i"
#define cPRIoMAX       __cPRI64_PREFIX "o"
#define cPRIuMAX       __cPRI64_PREFIX "u"
#define cPRIxMAX       __cPRI64_PREFIX "x"
#define cPRIXMAX       __cPRI64_PREFIX "X"


/* Macros for printing `intptr_t' and `uintptr_t'.  */
#define cPRIdPTR       __cPRIPTR_PREFIX "d"
#define cPRIiPTR       __cPRIPTR_PREFIX "i"
#define cPRIoPTR       __cPRIPTR_PREFIX "o"
#define cPRIuPTR       __cPRIPTR_PREFIX "u"
#define cPRIxPTR       __cPRIPTR_PREFIX "x"
#define cPRIXPTR       __cPRIPTR_PREFIX "X"


/* Macros for scanning format specifiers.  */

/* Signed decimal notation.  */
#define cSCNd8         "hhd"
#define cSCNd16        "hd"
#define cSCNd32        "d"
#define cSCNd64        __cPRI64_PREFIX "d"

#define cSCNdLEAST8    "hhd"
#define cSCNdLEAST16   "hd"
#define cSCNdLEAST32   "d"
#define cSCNdLEAST64   __cPRI64_PREFIX "d"

#define cSCNdFAST8     "hhd"
#define cSCNdFAST16    __cPRIPTR_PREFIX "d"
#define cSCNdFAST32    __cPRIPTR_PREFIX "d"
#define cSCNdFAST64    __cPRI64_PREFIX "d"

/* Signed decimal notation.  */
#define cSCNi8         "hhi"
#define cSCNi16        "hi"
#define cSCNi32        "i"
#define cSCNi64        __cPRI64_PREFIX "i"

#define cSCNiLEAST8    "hhi"
#define cSCNiLEAST16   "hi"
#define cSCNiLEAST32   "i"
#define cSCNiLEAST64   __cPRI64_PREFIX "i"

#define cSCNiFAST8     "hhi"
#define cSCNiFAST16    __cPRIPTR_PREFIX "i"
#define cSCNiFAST32    __cPRIPTR_PREFIX "i"
#define cSCNiFAST64    __cPRI64_PREFIX "i"

/* Unsigned decimal notation.  */
#define cSCNu8         "hhu"
#define cSCNu16        "hu"
#define cSCNu32        "u"
#define cSCNu64        __cPRI64_PREFIX "u"

#define cSCNuLEAST8    "hhu"
#define cSCNuLEAST16   "hu"
#define cSCNuLEAST32   "u"
#define cSCNuLEAST64   __cPRI64_PREFIX "u"

#define cSCNuFAST8     "hhu"
#define cSCNuFAST16    __cPRIPTR_PREFIX "u"
#define cSCNuFAST32    __cPRIPTR_PREFIX "u"
#define cSCNuFAST64    __cPRI64_PREFIX "u"

/* Octal notation.  */
#define cSCNo8         "hho"
#define cSCNo16        "ho"
#define cSCNo32        "o"
#define cSCNo64        __cPRI64_PREFIX "o"

#define cSCNoLEAST8    "hho"
#define cSCNoLEAST16   "ho"
#define cSCNoLEAST32   "o"
#define cSCNoLEAST64   __cPRI64_PREFIX "o"

#define cSCNoFAST8     "hho"
#define cSCNoFAST16    __cPRIPTR_PREFIX "o"
#define cSCNoFAST32    __cPRIPTR_PREFIX "o"
#define cSCNoFAST64    __cPRI64_PREFIX "o"

/* Hexadecimal notation.  */
#define cSCNx8         "hhx"
#define cSCNx16        "hx"
#define cSCNx32        "x"
#define cSCNx64        __cPRI64_PREFIX "x"

#define cSCNxLEAST8    "hhx"
#define cSCNxLEAST16   "hx"
#define cSCNxLEAST32   "x"
#define cSCNxLEAST64   __cPRI64_PREFIX "x"

#define cSCNxFAST8     "hhx"
#define cSCNxFAST16    __cPRIPTR_PREFIX "x"
#define cSCNxFAST32    __cPRIPTR_PREFIX "x"
#define cSCNxFAST64    __cPRI64_PREFIX "x"


/* Macros for scanning `intmax_t' and `uintmax_t'.  */
#define cSCNdMAX       __cPRI64_PREFIX "d"
#define cSCNiMAX       __cPRI64_PREFIX "i"
#define cSCNoMAX       __cPRI64_PREFIX "o"
#define cSCNuMAX       __cPRI64_PREFIX "u"
#define cSCNxMAX       __cPRI64_PREFIX "x"

/* Macros for scaning `intptr_t' and `uintptr_t'.  */
#define cSCNdPTR       __cPRIPTR_PREFIX "d"
#define cSCNiPTR       __cPRIPTR_PREFIX "i"
#define cSCNoPTR       __cPRIPTR_PREFIX "o"
#define cSCNuPTR       __cPRIPTR_PREFIX "u"
#define cSCNxPTR       __cPRIPTR_PREFIX "x"

/**
 * @}
 */
#endif /* __PERLS_COMMON_CINTTYPES_H__ */
