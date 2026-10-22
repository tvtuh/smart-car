/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Vendor contact details omitted; original attribution retained. */
/* --- Vendor contact details omitted; original attribution retained. */
/* --- Vendor contact details omitted; original attribution retained. */
/* --- Web: www.STCAI.com ---------------------------------------------*/
/* --- BBS: www.STCAIMCU.com  -----------------------------------------*/
/* --- Vendor contact details omitted; original attribution retained. */
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序        */
/*---------------------------------------------------------------------*/

#include "stc.h"
#include "util.h"

DWORD reverse4(DWORD d)
{   
    DWORD ret;
    
    ((BYTE *)&ret)[0] = ((BYTE *)&d)[3];
    ((BYTE *)&ret)[1] = ((BYTE *)&d)[2];
    ((BYTE *)&ret)[2] = ((BYTE *)&d)[1];
    ((BYTE *)&ret)[3] = ((BYTE *)&d)[0];

    return ret;
}
   
WORD reverse2(WORD w)
{
    WORD ret;
    
    ((BYTE *)&ret)[0] = ((BYTE *)&w)[1];
    ((BYTE *)&ret)[1] = ((BYTE *)&w)[0];

    return ret;
}
