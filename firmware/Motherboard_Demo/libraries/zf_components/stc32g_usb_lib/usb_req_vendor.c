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
#include "usb.h"
#include "usb_req_vendor.h"

void usb_req_vendor()
{
    usb_setup_stall();
}
