/*-----------------------------------------------------------------------------
|  Project :  CSST
|  Module  :  Host
+------------------------------------------------------------------------------
|             Copyright 2003 Texas Instruments.
|             All rights reserved.
|
|             This file is confidential and a trade secret of Texas
|             Instruments .
|             The receipt of or possession of this file does not convey
|             any rights to reproduce or disclose its contents or to
|             manufacture, use, or sell anything it may describe, in
|             whole, or in part, without the specific written consent of
|             Texas Instruments.
+------------------------------------------------------------------------------
| Filename:   version.h
| Author:     Thomas Lund Søhus
|
| Purpose:    This header file defines the CSST version information
+----------------------------------------------------------------------------*/

/*==== DECLARATION CONTROL ==================================================*/
#ifndef __version_h__
#define __version_h__

/*==== INCLUDES =============================================================*/

/*==== MACROS ===============================================================*/
#define VER_XTOSTR(x)           VER_TOSTR(x)
#define VER_TOSTR(x)            #x

/* Product specific version information */
#define VER_COMPANY_NAME        "Texas Instruments Inc.\0"
#define VER_PRODUCT_NAME        "Cellular Systems Software Tools - CSST 3430\0"
#define VER_COPYRIGHT           "Copyright © TI 2009\0"
#define VER_PRODUCT_VERSION     2.6.2.5

/*==== CONSTS ===============================================================*/

/*==== TYPES ================================================================*/

/*==== EXPORTS ==============================================================*/

#endif /* __version_h__ */

/*==== END OF FILE ==========================================================*/

