/* Actual bounded SDK helper with fake GPIO/SPI. No network-ready or RX-drain
 * loop may hold the control loop indefinitely. */
#include "zf_common_typedef.h"
#include "zf_device_wifi_spi.h"
#undef WIFI_SPI_INT_PIN
#define WIFI_SPI_INT_PIN 0
static wifi_spi_state_enum wifi_spi_mutex=WIFI_SPI_IDLE;
static int wifi_spi_fifo;
static int ready=1,transfers,fifo_bytes;
static uint32 delay_us;
static uint8 reply_command=WIFI_SPI_REPLY_OK;
static uint16 reply_length;
static uint8 gpio_get_level(int pin){(void)pin;return (uint8)ready;}
static void system_delay_us(uint32 us){delay_us+=us;}
static void fifo_write_buffer(int *fifo,const uint8 *buffer,uint16 len)
{(void)fifo;(void)buffer;fifo_bytes+=len;}
static void wifi_spi_dma_transfer_data(const uint8 *buffer,wifi_spi_packets_struct *packet,uint16 len)
{(void)buffer;(void)len;transfers++;packet->head.command=reply_command;packet->head.length=reply_length;}
#include "wifi_sync_under_test.inc"
#define CHECK(c,m) do{if(!(c)){fputs(m "\n",stderr);return 1;}}while(0)
int main(int argc,char **argv)
{
    int mode;
    uint8 input[WIFI_SPI_TRANSFER_SIZE+1]={0};
    uint32 remaining;
    if(argc!=2){return 2;}mode=atoi(argv[1]);
    if(mode==1){ready=0;}
    if(mode==2){reply_command=WIFI_SPI_REPLY_ERROR;}
    if(mode==3){reply_command=WIFI_SPI_REPLY_DATA_END;reply_length=33;}
    if(mode==4){reply_command=WIFI_SPI_REPLY_DATA_START;reply_length=32;}
    if(mode==5)
    {
        CHECK(wifi_spi_send_buffer_timeout(NULL,8,5)==8,"FAIL null buffer accepted");
        CHECK(wifi_spi_send_buffer_timeout(input,0,5)==0,"FAIL zero-length behavior");
        CHECK(wifi_spi_send_buffer_timeout(input,WIFI_SPI_TRANSFER_SIZE+1,5)==WIFI_SPI_TRANSFER_SIZE+1,"FAIL oversized packet accepted");
        CHECK(!transfers && !delay_us,"FAIL invalid input touched SPI");return 0;
    }
    if(mode==6){wifi_spi_mutex=WIFI_SPI_BUSY;}
    remaining=wifi_spi_send_buffer_timeout(input,8,5);
    if(mode==1)
    {CHECK(remaining==8 && delay_us==5000 && !transfers,"FAIL not-ready wait exceeded the requested 5ms");}
    else if(mode==2 || mode==3)
    {CHECK(remaining==8 && transfers==1 && !fifo_bytes,"FAIL malformed reply accepted or copied");}
    else if(mode==4)
    {CHECK(!remaining && transfers==1 && fifo_bytes==32,"FAIL DATA_START caused extra drain/wait instead of returning");}
    else if(mode==6)
    {CHECK(remaining==8 && !transfers && wifi_spi_mutex==WIFI_SPI_BUSY,"FAIL mutex owner was disturbed");}
    else
    {CHECK(!remaining && transfers==1 && !delay_us,"FAIL ready packet did not complete synchronously");}
    if(mode!=6){CHECK(wifi_spi_mutex==WIFI_SPI_IDLE,"FAIL SPI mutex leaked");}
    printf("PASS bounded synchronous WiFi mode=%d delay_us=%lu\n",mode,(unsigned long)delay_us);return 0;
}
