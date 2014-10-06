/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  ADLER-32
+------------------------------------------------------------------------------
|             Copyright (c) 1996 L. Peter Deutsch and Jean-Loup Gailly
|
|             Permission is granted to copy and distribute this document for any
|             purpose and without charge, including translations into other
|             languages and incorporation into compilations, provided that the
|             copyright notice and this notice are preserved, and that any
|             substantive changes or deletions from the original are clearly
|             marked.
+----------------------------------------------------------------------------*/

#ifndef ADLER32_H
#define ADLER32_H


#ifdef __cplusplus
extern "C" {
#endif

    unsigned long update_adler32(unsigned long adler, unsigned char *buf, int len);
    unsigned long adler32(unsigned char *buf, int len);
    unsigned long checksum(unsigned char *buf, int len);

#ifdef __cplusplus
}
#endif

#endif // ADLER32_H

