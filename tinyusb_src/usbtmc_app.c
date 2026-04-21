/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Nathan Conrad
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#define GPIO0_pin PA02
#define GPIO1_pin PA03
#define IDN        "SAMD21"
#define IDN_QUERY  "*idn?"
#define RST_CMD    "*rst"
#define TRIG_CMD   "*trig"
#define GPIO0_LEV_CMD    "gpio0:lev " // GPIO0:LEVel
#define GPIO0_LEV_QUERY  "gpio0:lev?" // GPIO0:LEVel
#define GPIO1_LEV_CMD    "gpio1:lev " // GPIO1:LEVel
#define GPIO1_LEV_QUERY  "gpio1:lev?" // GPIO1:LEVel
#define GPIO2_LEV_CMD    "gpio2:lev " // GPIO2:LEVel
#define GPIO2_LEV_QUERY  "gpio2:lev?" // GPIO2:LEVel
#define GPIO3_LEV_CMD    "gpio3:lev " // GPIO3:LEVel
#define GPIO3_LEV_QUERY  "gpio3:lev?" // GPIO3:LEVel
#define GPIO4_LEV_CMD    "gpio4:lev " // GPIO4:LEVel
#define GPIO4_LEV_QUERY  "gpio4:lev?" // GPIO4:LEVel
#define GPIO5_LEV_CMD    "gpio5:lev " // GPIO5:LEVel
#define GPIO5_LEV_QUERY  "gpio5:lev?" // GPIO5:LEVel
#define GPIO6_LEV_CMD    "gpio6:lev " // GPIO6:LEVel
#define GPIO6_LEV_QUERY  "gpio6:lev?" // GPIO6:LEVel
#define GPIO7_LEV_CMD    "gpio7:lev " // GPIO7:LEVel
#define GPIO7_LEV_QUERY  "gpio7:lev?" // GPIO7:LEVel
#define GPIO8_LEV_CMD    "gpio8:lev " // GPIO8:LEVel
#define GPIO8_LEV_QUERY  "gpio8:lev?" // GPIO8:LEVel
#define DLY1_CMD         "dly1 "      // 
#define DLY1_QUERY       "dly1?"      // 
#define DLY2_CMD         "dly2 "      //
#define DLY2_QUERY       "dly2?"      //
#define DLY3_CMD         "dly3 "      //
#define DLY3_QUERY       "dly3?"      //
#define END_RESPONSE     "\n"         // USB488

#include <string.h>
#include <stdlib.h>     /* atoi */
#include <stdio.h>      /* fprintf */
#include "tusb.h"
#include "bsp/board_api.h"
#include "main.h"
#include "usbtmc_app.h"
#include "sam.h"

char * get_value(char *in_string);
void ftoa(float num, char *str);
static inline void delay_us(volatile uint32_t count);

#if (CFG_TUD_USBTMC_ENABLE_488)
static usbtmc_response_capabilities_488_t const
#else
static usbtmc_response_capabilities_t const
#endif
tud_usbtmc_app_capabilities  =
{
    .USBTMC_status = USBTMC_STATUS_SUCCESS,
    .bcdUSBTMC = USBTMC_VERSION,
    .bmIntfcCapabilities =
    {
        .listenOnly = 0,
        .talkOnly = 0,
        .supportsIndicatorPulse = 1
    },
    .bmDevCapabilities = {
        .canEndBulkInOnTermChar = 0
    },

#if (CFG_TUD_USBTMC_ENABLE_488)
    .bcdUSB488 = USBTMC_488_VERSION,
    .bmIntfcCapabilities488 =
    {
        .supportsTrigger = 1,
        .supportsREN_GTL_LLO = 0,
        .is488_2 = 1
    },
    .bmDevCapabilities488 =
    {
      .SCPI = 1,
      .SR1 = 0,
      .RL1 = 0,
      .DT1 =0,
    }
#endif
};

#define IEEE4882_STB_QUESTIONABLE (0x08u)
#define IEEE4882_STB_MAV          (0x10u)
#define IEEE4882_STB_SER          (0x20u)
#define IEEE4882_STB_SRQ          (0x40u)

static const char idn[] = "GPIO0:LEV 1\nDLY1 100\n*TRIG\nhttps://github.com/charkster/mosfet_switch_board_v1\r\n";
static volatile uint8_t status;

// 0=not query, 1=queried, 2=delay,set(MAV), 3=delay 4=ready?
// (to simulate delay)
static volatile uint16_t queryState = 0;
static volatile uint32_t queryDelayStart;
static volatile uint32_t bulkInStarted;
static volatile uint32_t idnQuery;
static volatile bool rst_cmd;
static volatile bool gpio_lev_cmd;
static volatile bool gpio_lev_query;
static volatile bool dly_cmd;
static volatile bool dly_query;
static volatile bool trig_cmd;

static uint32_t resp_delay = 125u; // Adjustable delay, to allow for better testing
static size_t buffer_len;
static size_t buffer_tx_ix; // for transmitting using multiple transfers
static uint8_t buffer[225]; // A few packets long should be enough.

char gpio_lev_str[2];
char dly_str[10];
uint32_t dly1_value = 0;
uint32_t dly2_value = 0;
uint32_t dly3_value = 0;

void tud_usbtmc_open_cb(uint8_t interface_id)
{
  (void)interface_id;
  tud_usbtmc_start_bus_read();
}

#if (CFG_TUD_USBTMC_ENABLE_488)
usbtmc_response_capabilities_488_t const *
#else
usbtmc_response_capabilities_t const *
#endif
tud_usbtmc_get_capabilities_cb(void)
{
  return &tud_usbtmc_app_capabilities;
}


bool tud_usbtmc_msg_trigger_cb(usbtmc_msg_generic_t* msg) {
  (void)msg;
  // Let trigger set the SRQ
  status |= IEEE4882_STB_SRQ;
  return true;
}

bool tud_usbtmc_msgBulkOut_start_cb(usbtmc_msg_request_dev_dep_out const * msgHeader)
{
  (void)msgHeader;
  buffer_len = 0;
  if(msgHeader->TransferSize > sizeof(buffer))
  {

    return false;
  }
  return true;
}

bool tud_usbtmc_msg_data_cb(void *data, size_t len, bool transfer_complete)
{
  // If transfer isn't finished, we just ignore it (for now)

  if(len + buffer_len < sizeof(buffer))
  {
    memcpy(&(buffer[buffer_len]), data, len);
    buffer_len += len;
  }
  else
  {
    return false; // buffer overflow!
  }
  queryState = transfer_complete;
  idnQuery = 0;
  rst_cmd        = false;
  trig_cmd       = false;
  gpio_lev_cmd   = false;
  gpio_lev_query = false;
  dly_cmd        = false;
  dly_query      = false;
  uint32_t dly1_mod = 0;
  uint32_t dly2_mod = 0;
  uint32_t dly3_mod = 0;

  if ( transfer_complete && (len >= 4) &&
       (!strncmp("*idn?", data, 4) || !strncmp("*IDN?", data, 4)) )
  {
    idnQuery = 1;
  }
  else if ( transfer_complete && (len >=4) && !strncasecmp(RST_CMD,data,4))
  {
    rst_cmd = true;
    dly1_value = 0;
    dly2_value = 0;
    dly3_value = 0;
    PORT->Group[0].OUTCLR.reg = PORT_PA02; // GPIO0
    PORT->Group[0].OUTCLR.reg = PORT_PA03; // GPIO1
    PORT->Group[0].OUTCLR.reg = PORT_PA04; // GPIO2
    PORT->Group[0].OUTCLR.reg = PORT_PA05; // GPIO3
    PORT->Group[0].OUTCLR.reg = PORT_PA16; // GPIO4
    PORT->Group[0].OUTCLR.reg = PORT_PA17; // GPIO5
    PORT->Group[0].OUTCLR.reg = PORT_PA06; // GPIO6
    PORT->Group[0].OUTCLR.reg = PORT_PA07; // GPIO7
    PORT->Group[0].OUTCLR.reg = PORT_PA11; // GPIO8
  } 
  else if ( transfer_complete && (len >=5) && !strncasecmp(TRIG_CMD,data,5))
  {
    trig_cmd = true;
    if (dly1_value != 0) {
      dly1_mod = dly1_value + (dly1_value * 0.14); // adjustment for large delays
    }
    if (dly2_value != 0) {
      dly2_mod = dly2_value + (dly2_value * 0.14); // adjustment for large delays
    }
    if (dly3_value != 0) {
      dly3_mod = dly3_value + (dly3_value * 0.14); // adjustment for large delays
    }
    if (dly1_value != 0) {
      if (PORT->Group[0].OUT.reg & PORT_PA02) {
        PORT->Group[0].OUTCLR.reg = PORT_PA02; // drive low value
      }
      else {
        PORT->Group[0].OUTSET.reg = PORT_PA02; // drive high value
      }
      delay_us(dly1_mod);
      if (PORT->Group[0].OUT.reg & PORT_PA03) {
        PORT->Group[0].OUTCLR.reg = PORT_PA03; // drive low value
      }
      else {
        PORT->Group[0].OUTSET.reg = PORT_PA03; // drive high value
      }
    }
    if (dly2_value != 0) {
      delay_us(dly2_mod);
      if (PORT->Group[0].OUT.reg & PORT_PA04) {
        PORT->Group[0].OUTCLR.reg = PORT_PA04; // drive low value
      }
      else {
        PORT->Group[0].OUTSET.reg = PORT_PA04; // drive high value
      }
    }
    if (dly3_value != 0) {
      delay_us(dly3_mod);
      if (PORT->Group[0].OUT.reg & PORT_PA05) {
        PORT->Group[0].OUTCLR.reg = PORT_PA05; // drive low value
      }
      else {
        PORT->Group[0].OUTSET.reg = PORT_PA05; // drive high value
      }
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO0_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA02; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA02; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO0_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
    
    if (PORT->Group[0].OUT.reg & PORT_PA02)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO1_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA03; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA03; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO1_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA03)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO2_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA04; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA04; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO2_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;
   
    if (PORT->Group[0].OUT.reg & PORT_PA04)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO3_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA05; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA05; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO3_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA05)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO4_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA16; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA16; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO4_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA16)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO5_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA17; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA17; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO5_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA17)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO6_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA06; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA06; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO6_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA06)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO7_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA07; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA07; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO7_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA07)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if ( transfer_complete && (len >= 10) && !strncasecmp(GPIO8_LEV_CMD,data,10) )
  {
    gpio_lev_cmd = true;
    char *ptr_value = get_value(data);
    int gpio_level = atoi(ptr_value);
    if (gpio_level == 1)
    {
      PORT->Group[0].OUTSET.reg = PORT_PA11; // drive high value
    }
    else if (gpio_level == 0)
    {
      PORT->Group[0].OUTCLR.reg = PORT_PA11; // drive low value
    }
  }
  else if (transfer_complete && (len >= 10) && !strncasecmp(GPIO8_LEV_QUERY,data,10))
  {
    gpio_lev_query = true;

    if (PORT->Group[0].OUT.reg & PORT_PA11)
    {
      strcpy(gpio_lev_str,"1");
    }
    else
    {
      strcpy(gpio_lev_str,"0");
    }
  }
  else if (transfer_complete && (len >=5) && !strncasecmp(DLY1_CMD,data,5))
  {
    dly_cmd         = true;
    char *ptr_value = get_value(data);
    dly1_value      = (uint32_t)atoi(ptr_value);
  }
  else if (transfer_complete && (len >= 5) && !strncasecmp(DLY1_QUERY,data,5))
  {
    dly_query = true;
    itoa(dly1_value,dly_str,10);
  }
  else if (transfer_complete && (len >=5) && !strncasecmp(DLY2_CMD,data,5))
  {
    dly_cmd         = true;
    char *ptr_value = get_value(data);
    dly2_value      = (uint32_t)atoi(ptr_value);
  }
  else if (transfer_complete && (len >= 5) && !strncasecmp(DLY2_QUERY,data,5))
  {
    dly_query = true;
    itoa(dly2_value,dly_str,10);
  }
  else if (transfer_complete && (len >=5) && !strncasecmp(DLY3_CMD,data,5))
  {
    dly_cmd         = true;
    char *ptr_value = get_value(data);
    dly3_value      = (uint32_t)atoi(ptr_value);
  }
  else if (transfer_complete && (len >= 5) && !strncasecmp(DLY3_QUERY,data,5))
  {
    dly_query = true;
    itoa(dly3_value,dly_str,10);
  }

  if ( transfer_complete &&
       (!strncmp("delay ", data, 5) || !strncmp("DELAY ", data, 5)) )
  {
    queryState = 0;
    int d = atoi((char*)data + 5);
    if(d > 10000)
      d = 10000;
    if(d<0)
      d=0;
    resp_delay = (uint32_t)d;
  }
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_msgBulkIn_complete_cb(void)
{
  if((buffer_tx_ix == buffer_len) || idnQuery) // done
  {
    status &= (uint8_t)~(IEEE4882_STB_MAV); // clear MAV
    queryState = 0;
    bulkInStarted = 0;
    buffer_tx_ix = 0;
  }
  tud_usbtmc_start_bus_read();

  return true;
}

static unsigned int msgReqLen;

bool tud_usbtmc_msgBulkIn_request_cb(usbtmc_msg_request_dev_dep_in const * request)
{
  msgReqLen = request->TransferSize;

#ifdef xDEBUG
  uart_tx_str_sync("MSG_IN_DATA: Requested!\r\n");
#endif
  if(queryState == 0 || (buffer_tx_ix == 0))
  {
    TU_ASSERT(bulkInStarted == 0);
    bulkInStarted = 1;

    // > If a USBTMC interface receives a Bulk-IN request prior to receiving a USBTMC command message
    //   that expects a response, the device must NAK the request (*not stall*)
  }
  else
  {
    size_t txlen = tu_min32(buffer_len-buffer_tx_ix,msgReqLen);
    tud_usbtmc_transmit_dev_msg_data(&buffer[buffer_tx_ix], txlen,
        (buffer_tx_ix+txlen) == buffer_len, false);
    buffer_tx_ix += txlen;
  }
  // Always return true indicating not to stall the EP.
  return true;
}

void usbtmc_app_task_iter(void) {
  switch(queryState) {
  case 0:
    break;
  case 1:
    queryDelayStart = tusb_time_millis_api();
    queryState = 2;
    break;
  case 2:
    if( (tusb_time_millis_api() - queryDelayStart) > resp_delay) {
      queryDelayStart = tusb_time_millis_api();
      queryState=3;
      status |= 0x10u; // MAV
      status |= 0x40u; // SRQ
    }
    break;
  case 3:
    if( (tusb_time_millis_api() - queryDelayStart) > resp_delay) {
      queryState = 4;
    }
    break;
  case 4: // time to transmit;
    if(bulkInStarted && (buffer_tx_ix == 0)) {
      if(idnQuery)
      {
        tud_usbtmc_transmit_dev_msg_data(idn,  tu_min32(sizeof(idn)-1,msgReqLen),true,false);
        queryState = 0;
        bulkInStarted = 0;
      }
      else if (gpio_lev_query)
      {
        tud_usbtmc_transmit_dev_msg_data(gpio_lev_str, tu_min32(sizeof(gpio_lev_str)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (dly_query)
      {
        tud_usbtmc_transmit_dev_msg_data(dly_str, tu_min32(strlen(dly_str),msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else if (rst_cmd || trig_cmd || gpio_lev_cmd || dly_cmd)
      {
        tud_usbtmc_transmit_dev_msg_data(END_RESPONSE, tu_min32(sizeof(END_RESPONSE)-1,msgReqLen),true,false);
        queryState    = 0;
        bulkInStarted = 0;
      }
      else
      {
        buffer_tx_ix = tu_min32(buffer_len,msgReqLen);
        tud_usbtmc_transmit_dev_msg_data(buffer, buffer_tx_ix, buffer_tx_ix == buffer_len, false);
      }
      // MAV is cleared in the transfer complete callback.
    }
    break;
  default:
    TU_ASSERT(false,);
  }
}

bool tud_usbtmc_initiate_clear_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  return true;
}

bool tud_usbtmc_check_clear_cb(usbtmc_get_clear_status_rsp_t *rsp)
{
  queryState = 0;
  bulkInStarted = false;
  status = 0;
  buffer_tx_ix = 0u;
  buffer_len = 0u;
  rsp->USBTMC_status = USBTMC_STATUS_SUCCESS;
  rsp->bmClear.BulkInFifoBytes = 0u;
  return true;
}
bool tud_usbtmc_initiate_abort_bulk_in_cb(uint8_t *tmcResult)
{
  bulkInStarted = 0;
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}
bool tud_usbtmc_check_abort_bulk_in_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

bool tud_usbtmc_initiate_abort_bulk_out_cb(uint8_t *tmcResult)
{
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;

}
bool tud_usbtmc_check_abort_bulk_out_cb(usbtmc_check_abort_bulk_rsp_t *rsp)
{
  (void)rsp;
  tud_usbtmc_start_bus_read();
  return true;
}

void tud_usbtmc_bulkIn_clearFeature_cb(void)
{
}
void tud_usbtmc_bulkOut_clearFeature_cb(void)
{
  tud_usbtmc_start_bus_read();
}

// Return status byte, but put the transfer result status code in the rspResult argument.
uint8_t tud_usbtmc_get_stb_cb(uint8_t *tmcResult)
{
  uint8_t old_status = status;
  status = (uint8_t)(status & ~(IEEE4882_STB_SRQ)); // clear SRQ

  *tmcResult = USBTMC_STATUS_SUCCESS;
  // Increment status so that we see different results on each read...

  return old_status;
}

bool tud_usbtmc_indicator_pulse_cb(tusb_control_request_t const * msg, uint8_t *tmcResult)
{
  (void)msg;
  led_indicator_pulse();
  *tmcResult = USBTMC_STATUS_SUCCESS;
  return true;
}

//---------------------------- New Code -------------------------//

void gpio_setup(void) {
  PORT->Group[0].DIRSET.reg = PORT_PA02; // PA02 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA02; // PA02 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA03; // PA03 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA03; // PA03 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA04; // PA02 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA04; // PA02 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA05; // PA03 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA05; // PA03 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA16; // PA02 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA16; // PA02 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA17; // PA03 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA17; // PA03 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA06; // PA02 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA06; // PA02 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA07; // PA03 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA07; // PA03 initialized low
  PORT->Group[0].DIRSET.reg = PORT_PA11; // PA03 as output
  PORT->Group[0].OUTCLR.reg = PORT_PA11; // PA03 initialized low
}

char * get_value(char *in_string) {
  char *ptr = strrchr(in_string,' ') + 1;
  return ptr;
}

// 48MHz cpu clock, need to delay by 48 cycles, plus overhead
static inline void delay_us(volatile uint32_t count) {
  while (count--) {
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
     __NOP();
  }
}
