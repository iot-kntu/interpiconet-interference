/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Application main file for the BLE multirole LE Secure Connections (LESC) example.
 *
 * @detail This application demonstrates bonding with LE Secure Connections both as a peripheral and as a central.
 *
 * LED layout:
 * LED 1: Central side is scanning.       LED 2: Central side is connected to a peripheral.
 * LED 3: Peripheral side is advertising. LED 4: Peripheral side is connected to a central.
 *
 * @note: This application requires the use of an external ECC library for public key and shared secret calculation.
 *        Refer to the application's documentation for more details.
 *
 */

#include "sdk_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "app_util.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_hrs.h"
#include "ble_hrs_c.h"
#include "ble_conn_state.h"
#include "fds.h"
#include "nrf_crypto.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"
#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_gap.h"
#include "nrf_delay.h"

#include "packet_error_rate.c"


#define LESC_DEBUG_MODE                 0                                               /**< Set to 1 to use the LESC debug keys. The debug mode allows you to use a sniffer to inspect traffic. */
#define LESC_MITM_NC                    1                                               /**< Use MITM (Numeric Comparison). */

/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT              (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)

#define APP_BLE_CONN_CFG_TAG            1                                               /**< Tag that identifies the SoftDevice BLE configuration. */

#define CENTRAL_SCANNING_LED            BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED           BSP_BOARD_LED_1
#define PERIPHERAL_ADVERTISING_LED      BSP_BOARD_LED_2
#define PERIPHERAL_CONNECTED_LED        BSP_BOARD_LED_3

#define SCAN_DURATION                   0x0000                                          /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning continues until it is explicitly disabled. */
#define APP_ADV_DURATION                18000                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */


#define SEC_PARAMS_BOND                 1                                               /**< Perform bonding. */
#if LESC_MITM_NC
#define SEC_PARAMS_MITM                 1                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_YESNO                   /**< Display Yes/No to force Numeric Comparison. */
#else
#define SEC_PARAMS_MITM                 0                                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                            /**< No I/O caps. */
#endif
#define SEC_PARAMS_LESC                 0                                               /**< LE Secure Connections pairing required. */
#define SEC_PARAMS_KEYPRESS             0                                               /**< Keypress notifications not required. */
#define SEC_PARAMS_OOB                  0                                               /**< Out Of Band data not available. */
#define SEC_PARAMS_MIN_KEY_SIZE         7                                               /**< Minimum encryption key size in octets. */
#define SEC_PARAMS_MAX_KEY_SIZE         16                                              /**< Maximum encryption key size in octets. */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                           /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                          /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                               /**< Number of attempts before giving up the connection parameter negotiation. */


#define SCHED_MAX_EVENT_DATA_SIZE           APP_TIMER_SCHED_EVENT_DATA_SIZE            /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                    20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                    10                                         /**< Maximum number of events in the scheduler queue. */
#endif

#define MINIMUM_CHANNEL_SURVEY_SELECTION              15                                /* Use the first number of the channel maps after channel survey */


/**@brief   Priority of the application BLE event handler.
 * @note    There is no need to modify this value.
 */
#define APP_BLE_OBSERVER_PRIO           3

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

static uint8_t rssi_count = 0;

static ble_opt_t new_opt_config;
int is_survey = 1;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

typedef struct
{
    bool           is_connected;
    ble_gap_addr_t address;
} conn_peer_t;

NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                            /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
BLE_HRS_DEF(m_hrs);                                                         /**< Heart Rate Service instance. */
BLE_HRS_C_DEF(m_hrs_c);                                                     /**< Structure used to identify the Heart Rate client module. */
NRF_BLE_GATT_DEF(m_gatt);                                                   /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT);                      /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                         /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                            /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                                   /**< Scanning Module instance. */

static uint16_t           m_conn_handle_hrs_c                = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the HRS central application. */
static volatile uint16_t  m_conn_handle_num_comp_central     = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the central that needs a numeric comparison button press. */
static volatile uint16_t  m_conn_handle_num_comp_peripheral  = BLE_CONN_HANDLE_INVALID;  /**< Connection handle for the peripheral that needs a numeric comparison button press. */

static conn_peer_t        m_connected_peers[NRF_BLE_LINK_COUNT];                         /**< Array of connected peers. */

static char * roles_str[] =
{
    "INVALID_ROLE",
    "PERIPHERAL",
    "CENTRAL",
};

/**@brief Names that the central application scans for, and that are advertised by the peripherals.
 *  If these are set to empty strings, the UUIDs defined below are used.
 */
static const char m_target_periph_name[] = "Heartrate";


/**@brief UUIDs that the central application scans for if the name above is set to an empty string,
 * and that are to be advertised by the peripherals.
 */
static ble_uuid_t m_adv_uuids[] = {};



static void on_adv_report(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    
    const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;

    const ble_gap_addr_t peer_addr = p_adv_report->peer_addr;

    if(peer_addr.addr[0] == 0x85 && p_adv_report->type.scan_response && p_adv_report->data.len){

          printf("Data 0 Manuf %x\n",p_adv_report->data.p_data[0]
      );
              printf("Data 1 Manuf %x\n",p_adv_report->data.p_data[1]
      );
                  printf("Data 2 Manuf %x\n",p_adv_report->data.p_data[2]
      );
                  printf("Data 3 Manuf %x\n",p_adv_report->data.p_data[3]
      );
                  printf("Data 4 Manuf %x\n",p_adv_report->data.p_data[4]
      );
                      printf("Data 5 Manuf %x\n",p_adv_report->data.p_data[5]
      );
                      printf("Data 6 Manuf %x\n",p_adv_report->data.p_data[6]
      );
                      printf("Data 7 Manuf %x\n",p_adv_report->data.p_data[7]
      );
                      printf("Data 8 Manuf %x\n",p_adv_report->data.p_data[8]
      );
                      printf("Data 9 Manuf %x\n",p_adv_report->data.p_data[9]
      );
                      printf("Data 10 Manuf %x\n",p_adv_report->data.p_data[10]
      );
                      printf("Data 11 Manuf %x\n",p_adv_report->data.p_data[11]
      );
                      printf("Data 12 Manuf %x\n",p_adv_report->data.p_data[12]
      );
                          printf("Data 13 Manuf %x\n",p_adv_report->data.p_data[13]
      );
                          printf("Data 14 Manuf %x\n",p_adv_report->data.p_data[14]
      );
                          printf("Data 15 Manuf %x\n",p_adv_report->data.p_data[15]
      );
                          printf("Data 16 Manuf %x\n",p_adv_report->data.p_data[16]
      );
                          printf("Data 17 Manuf %x\n",p_adv_report->data.p_data[17]
      );
                          printf("Data 18 Manuf %x\n",p_adv_report->data.p_data[18]
      );
                          printf("Data 19 Manuf %x\n",p_adv_report->data.p_data[19]
      );
                          printf("Data 20 Manuf %x\n",p_adv_report->data.p_data[20]
      );
                          printf("Data 21 Manuf %x\n",p_adv_report->data.p_data[21]
      );
                          printf("Data 22 Manuf %x\n",p_adv_report->data.p_data[22]
      );
                          printf("Data 23 Manuf %x\n",p_adv_report->data.p_data[23]
      );
                          printf("Data 24 Manuf %x\n",p_adv_report->data.p_data[24]
      );
                          printf("Data 25 Manuf %x\n",p_adv_report->data.p_data[25]
      );
                          printf("Data 26 Manuf %x\n",p_adv_report->data.p_data[26]
      );
                          printf("Data 27 Manuf %x\n",p_adv_report->data.p_data[27]
      );
                          printf("Data 28 Manuf %x\n",p_adv_report->data.p_data[28]
      );
                          printf("Data 29 Manuf %x\n",p_adv_report->data.p_data[29]
      );
                          printf("Data 30 Manuf %x\n",p_adv_report->data.p_data[30]
      );

    printf("Address %02x%02x%02x%02x%02x%02x\r\n",
                             peer_addr.addr[5],
                             peer_addr.addr[4],
                             peer_addr.addr[3],
                             peer_addr.addr[2],
                             peer_addr.addr[1],
                             peer_addr.addr[0]
                             );

            }

ble_gap_addr_t ble_addr;

sd_ble_gap_addr_get(&ble_addr);
    printf("My Address %x\r\n",
                             *ble_addr.addr
                             );

static ble_gap_addr_t const ble_addr0 =
{
    .addr      = 0xa9
};

static ble_gap_addr_t const ble_addr1 =
{
    .addr      = 0x9f
};

static ble_gap_addr_t const ble_addr2 =
{
    .addr      = 0xcd
};

    printf("My Address 2 %x\r\n",
                             *ble_addr1.addr
                             );

if(peer_addr.addr[0] == 0x85 && *ble_addr.addr == *ble_addr0.addr){

  NRF_LOG_INFO("set channel map from 15 ta 19");
  printf("Data Length %u\n",p_adv_report->data.len);
                            printf("Data 15 Manuf %u\n",p_adv_report->data.p_data[15]
      );
                          printf("Data 16 Manuf %u\n",p_adv_report->data.p_data[16]
      );
                          printf("Data 17 Manuf %u\n",p_adv_report->data.p_data[17]
      );
                          printf("Data 18 Manuf %u\n",p_adv_report->data.p_data[18]
      );
                          printf("Data 19 Manuf %u\n",p_adv_report->data.p_data[19]
      );

// set new channel map for test
                ret_code_t    error;
		memset(&new_opt_config, 0, sizeof(new_opt_config));
                new_opt_config.gap_opt.ch_map.ch_map[0] = p_adv_report->data.p_data[15];
		new_opt_config.gap_opt.ch_map.ch_map[1] = p_adv_report->data.p_data[16];
		new_opt_config.gap_opt.ch_map.ch_map[2] = p_adv_report->data.p_data[17];
		new_opt_config.gap_opt.ch_map.ch_map[3] = p_adv_report->data.p_data[18];
		new_opt_config.gap_opt.ch_map.ch_map[4] = p_adv_report->data.p_data[19];

                is_survey = 0;
}

if(peer_addr.addr[0] == 0x85 && *ble_addr.addr == *ble_addr1.addr){

  NRF_LOG_INFO("set channel map from 20 ta 24");
  printf("Data Length %u\n",p_adv_report->data.len);
                            printf("Data 20 Manuf %u\n",p_adv_report->data.p_data[20]
      );
                          printf("Data 21 Manuf %u\n",p_adv_report->data.p_data[21]
      );
                          printf("Data 22 Manuf %u\n",p_adv_report->data.p_data[22]
      );
                          printf("Data 23 Manuf %u\n",p_adv_report->data.p_data[23]
      );
                          printf("Data 24 Manuf %u\n",p_adv_report->data.p_data[24]
      );

// set new channel map for test
                ret_code_t    error;
		memset(&new_opt_config, 0, sizeof(new_opt_config));
                new_opt_config.gap_opt.ch_map.ch_map[0] = p_adv_report->data.p_data[20];
		new_opt_config.gap_opt.ch_map.ch_map[1] = p_adv_report->data.p_data[21];
		new_opt_config.gap_opt.ch_map.ch_map[2] = p_adv_report->data.p_data[22];
		new_opt_config.gap_opt.ch_map.ch_map[3] = p_adv_report->data.p_data[23];
		new_opt_config.gap_opt.ch_map.ch_map[4] = p_adv_report->data.p_data[24];

                is_survey = 0;
}
if(peer_addr.addr[0] == 0x85 && *ble_addr.addr == *ble_addr2.addr){
  NRF_LOG_INFO("set channel map from 25 ta 29");
  printf("Data Length %u\n",p_adv_report->data.len);
                            printf("Data 25 Manuf %u\n",p_adv_report->data.p_data[25]
      );
                          printf("Data 26 Manuf %u\n",p_adv_report->data.p_data[26]
      );
                          printf("Data 27 Manuf %u\n",p_adv_report->data.p_data[27]
      );
                          printf("Data 28 Manuf %u\n",p_adv_report->data.p_data[28]
      );
                          printf("Data 29 Manuf %u\n",p_adv_report->data.p_data[29]
      );

// set new channel map for test
                ret_code_t    error;
		memset(&new_opt_config, 0, sizeof(new_opt_config));
                new_opt_config.gap_opt.ch_map.ch_map[0] = p_adv_report->data.p_data[25];
		new_opt_config.gap_opt.ch_map.ch_map[1] = p_adv_report->data.p_data[26];
		new_opt_config.gap_opt.ch_map.ch_map[2] = p_adv_report->data.p_data[27];
		new_opt_config.gap_opt.ch_map.ch_map[3] = p_adv_report->data.p_data[28];
		new_opt_config.gap_opt.ch_map.ch_map[4] = p_adv_report->data.p_data[29];

                is_survey = 0;
}

// Establish connection
/*
if(!memcmp( nus_service_uuid,type_data.p_data,16))
{
    // Stop scanning.
    err_code = sd_ble_gap_scan_stop();
    if (err_code != NRF_SUCCESS)
    {
        printf("[APPL]: Scan stop failed, reason %d\r\n", (int)err_code);
    }
    nrf_gpio_pin_clear(SCAN_LED_PIN_NO);
    
    m_scan_param.selective = 0; 

    // Initiate connection.
    err_code = sd_ble_gap_connect(&p_gap_evt->params.adv_report.\
                                   peer_addr,
                                   &m_scan_param,
                                   &m_connection_param);

    if (err_code != NRF_SUCCESS)
    {
        printf("[APPL]: Connection Request Failed, reason %d\r\n", (int)err_code);
    }
    break;
}
*/



}




                                   
/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling the Heart Rate Service Client.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void hrs_c_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code that contains information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    ble_uuid_t          target_uuid = 
    {
        .uuid = BLE_UUID_HEART_RATE_SERVICE,
        .type = BLE_UUID_TYPE_BLE
    };
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_NAME_FILTER, 
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_ble_scan_filter_set(&m_scan, 
                                       SCAN_UUID_FILTER, 
                                       &target_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, 
                                           NRF_BLE_SCAN_NAME_FILTER | NRF_BLE_SCAN_UUID_FILTER, 
                                           false);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Scanning");
}


/**@brief Function for initializing the advertising and the scanning.
 */
static void adv_scan_start(void)
{
    ret_code_t err_code;

    scan_start();

    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);

    // Start advertising.
    //err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    //APP_ERROR_CHECK(err_code);

    
    // Custom Start advertising.
    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Advertising");
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            adv_scan_start();
            break;

        default:
            break;
    }
}


/**@brief Function for changing filter settings after establishing the connection.
 */
static void filter_settings_change(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_scan_all_filter_remove(&m_scan);
    APP_ERROR_CHECK(err_code);

    if (strlen(m_target_periph_name) != 0)
    {
        err_code = nrf_ble_scan_filter_set(&m_scan, 
                                           SCAN_NAME_FILTER, 
                                           m_target_periph_name);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Handles events coming from the Heart Rate central module.
 */
static void hrs_c_evt_handler(ble_hrs_c_t * p_hrs_c, ble_hrs_c_evt_t * p_hrs_c_evt)
{
    switch (p_hrs_c_evt->evt_type)
    {
        case BLE_HRS_C_EVT_DISCOVERY_COMPLETE:
        {
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                ret_code_t err_code;

                m_conn_handle_hrs_c = p_hrs_c_evt->conn_handle;

                // We do not want to connect to two peripherals offering the same service, so when
                // a UUID is matched, we check whether we are not already connected to a peer which
                // offers the same service
                filter_settings_change();

                NRF_LOG_INFO("CENTRAL: HRS discovered on conn_handle 0x%x",
                             m_conn_handle_hrs_c);

                err_code = ble_hrs_c_handles_assign(p_hrs_c,
                                                    m_conn_handle_hrs_c,
                                                    &p_hrs_c_evt->params.peer_db);
                APP_ERROR_CHECK(err_code);

                // Heart rate service discovered. Enable notification of Heart Rate Measurement.
                err_code = ble_hrs_c_hrm_notif_enable(p_hrs_c);
                APP_ERROR_CHECK(err_code);
            }
        } break; // BLE_HRS_C_EVT_DISCOVERY_COMPLETE

        case BLE_HRS_C_EVT_HRM_NOTIFICATION:
        {
            NRF_LOG_INFO("CENTRAL: Heart Rate = %d", p_hrs_c_evt->params.hrm.hr_value);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for checking whether a link already exists with a newly connected peer.
 *
 * @details This function checks whether the newly connected device is already connected.
 *
 * @param[in]   p_connected_evt Bluetooth connected event.
 * @return                      True if the peer's address is found in the list of connected peers,
 *                              false otherwise.
 */
static bool is_already_connected(ble_gap_addr_t const * p_connected_adr)
{
    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        if (m_connected_peers[i].is_connected)
        {
            if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type)
            {
                if (memcmp(m_connected_peers[i].address.addr,
                           p_connected_adr->addr,
                           sizeof(m_connected_peers[i].address.addr)) == 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}


/** @brief Function for handling a numeric comparison match request. */
static void on_match_request(uint16_t conn_handle, uint8_t role)
{
    // Mark the appropriate conn_handle as pending. The rest is handled on button press.
    NRF_LOG_INFO("Press Button 1 to confirm, Button 2 to reject");
    if (role == BLE_GAP_ROLE_CENTRAL)
    {
        m_conn_handle_num_comp_central = conn_handle;
    }
    else if (role == BLE_GAP_ROLE_PERIPH)
    {
        m_conn_handle_num_comp_peripheral = conn_handle;
    }
}





/**@brief Function for assigning new connection handle to the available instance of QWR module.
 *
 * @param[in] conn_handle New connection handle.
 */
static void multi_qwr_conn_handle_assign(uint16_t conn_handle)
{
    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
            APP_ERROR_CHECK(err_code);
            break;
        }
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            bsp_board_led_on(PERIPHERAL_ADVERTISING_LED);
            bsp_board_led_off(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);

        } break;

        default:
            // No implementation needed.
            break;
    }
}

#define BLE_GAP_QOS_CHANNEL_SURVERY_INTERVAL      0
#define NUM_REPORTS_BEFORE_AVERAGE_PRINTOUT       100

#define ADVERTISING_CHANNEL_37      37
#define ADVERTISING_CHANNEL_38      38
#define ADVERTISING_CHANNEL_39      39



typedef struct
{
        uint8_t index;
        bool channel_enable;
        float ch_energy;
} channel_energy_t;

static uint32_t m_num_channel_survey_reports_received;
static channel_energy_t ch_config_energy[BLE_GAP_CHANNEL_COUNT];
static channel_energy_t m_average_ch_energy[BLE_GAP_CHANNEL_COUNT];
static bool update_channel_survey_status = false;

uint32_t connection_channel_survey_start(void)
{
        ret_code_t err_code = sd_ble_gap_qos_channel_survey_start(BLE_GAP_QOS_CHANNEL_SURVERY_INTERVAL);
        APP_ERROR_CHECK(err_code);
        return err_code;
}

uint32_t connection_channel_survey_stop(void)
{
        ret_code_t err_code = sd_ble_gap_qos_channel_survey_stop();
        APP_ERROR_CHECK(err_code);
        return err_code;
}

static void sort_channel_survey(void)
{
        int n, c, d;
        float t;
        uint8_t index = 0;
        n = BLE_GAP_CHANNEL_COUNT;
        for (c = 1; c <= n - 1; c++)
        {
                d = c;
                while ( d > 0 && m_average_ch_energy[d-1].ch_energy > m_average_ch_energy[d].ch_energy)
                {
                        t          = m_average_ch_energy[d].ch_energy;
                        index      = m_average_ch_energy[d].index;
                        m_average_ch_energy[d].ch_energy   = m_average_ch_energy[d-1].ch_energy;
                        m_average_ch_energy[d].index       = m_average_ch_energy[d-1].index;
                        m_average_ch_energy[d-1].ch_energy = t;
                        m_average_ch_energy[d-1].index     = index;
                        d--;
                }
        }
        return;
}

void update_and_sort_channel_survey_handler(void * p_event_data, uint16_t size)
{
        NRF_LOG_INFO("Channel energy report:\n--------------\n");
        for (uint8_t i = 0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                int8_t channel_energy = m_average_ch_energy[i].ch_energy;
                m_average_ch_energy[i].index = i;
        }
        NRF_LOG_DEBUG("00-04: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[0].ch_energy, \
                      (int8_t)m_average_ch_energy[1].ch_energy, \
                      (int8_t)m_average_ch_energy[2].ch_energy, \
                      (int8_t)m_average_ch_energy[3].ch_energy, \
                      (int8_t)m_average_ch_energy[4].ch_energy  \
                      );
        NRF_LOG_DEBUG("05-09: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[5].ch_energy, \
                      (int8_t)m_average_ch_energy[6].ch_energy, \
                      (int8_t)m_average_ch_energy[7].ch_energy, \
                      (int8_t)m_average_ch_energy[8].ch_energy, \
                      (int8_t)m_average_ch_energy[9].ch_energy  \
                      );
        NRF_LOG_DEBUG("10-14: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[10].ch_energy, \
                      (int8_t)m_average_ch_energy[11].ch_energy, \
                      (int8_t)m_average_ch_energy[12].ch_energy, \
                      (int8_t)m_average_ch_energy[13].ch_energy, \
                      (int8_t)m_average_ch_energy[14].ch_energy  \
                      );
        NRF_LOG_DEBUG("15-19: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[15].ch_energy, \
                      (int8_t)m_average_ch_energy[16].ch_energy, \
                      (int8_t)m_average_ch_energy[17].ch_energy, \
                      (int8_t)m_average_ch_energy[18].ch_energy, \
                      (int8_t)m_average_ch_energy[19].ch_energy  \
                      );
        NRF_LOG_DEBUG("20-24: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[20].ch_energy, \
                      (int8_t)m_average_ch_energy[21].ch_energy, \
                      (int8_t)m_average_ch_energy[22].ch_energy, \
                      (int8_t)m_average_ch_energy[23].ch_energy, \
                      (int8_t)m_average_ch_energy[24].ch_energy  \
                      );
        NRF_LOG_DEBUG("25-29: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[25].ch_energy, \
                      (int8_t)m_average_ch_energy[26].ch_energy, \
                      (int8_t)m_average_ch_energy[27].ch_energy, \
                      (int8_t)m_average_ch_energy[28].ch_energy, \
                      (int8_t)m_average_ch_energy[29].ch_energy  \
                      );

        NRF_LOG_DEBUG("30-34: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[30].ch_energy, \
                      (int8_t)m_average_ch_energy[31].ch_energy, \
                      (int8_t)m_average_ch_energy[32].ch_energy, \
                      (int8_t)m_average_ch_energy[33].ch_energy, \
                      (int8_t)m_average_ch_energy[34].ch_energy  \
                      );
        NRF_LOG_DEBUG("35-39: %4d %4d %4d %4d %4d", \
                      (int8_t)m_average_ch_energy[35].ch_energy, \
                      (int8_t)m_average_ch_energy[36].ch_energy, \
                      (int8_t)m_average_ch_energy[37].ch_energy, \
                      (int8_t)m_average_ch_energy[38].ch_energy, \
                      (int8_t)m_average_ch_energy[39].ch_energy  \
                      );

        sort_channel_survey();

        NRF_LOG_DEBUG("Top Best Clean Channel Index:");

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[0].index,  \
                      m_average_ch_energy[1].index,  \
                      m_average_ch_energy[2].index,  \
                      m_average_ch_energy[3].index,  \
                      m_average_ch_energy[4].index   \
                      );

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[5].index,  \
                      m_average_ch_energy[6].index,  \
                      m_average_ch_energy[7].index,  \
                      m_average_ch_energy[8].index,  \
                      m_average_ch_energy[9].index   \
                      );

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[10].index,  \
                      m_average_ch_energy[11].index,  \
                      m_average_ch_energy[12].index,  \
                      m_average_ch_energy[13].index,  \
                      m_average_ch_energy[14].index   \
                      );

        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[15].index,  \
                      m_average_ch_energy[16].index,  \
                      m_average_ch_energy[17].index,  \
                      m_average_ch_energy[18].index,  \
                      m_average_ch_energy[19].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[20].index,  \
                      m_average_ch_energy[21].index,  \
                      m_average_ch_energy[22].index,  \
                      m_average_ch_energy[23].index,  \
                      m_average_ch_energy[24].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[25].index,  \
                      m_average_ch_energy[26].index,  \
                      m_average_ch_energy[27].index,  \
                      m_average_ch_energy[28].index,  \
                      m_average_ch_energy[29].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d %02d %02d %02d",  \
                      m_average_ch_energy[30].index,  \
                      m_average_ch_energy[31].index,  \
                      m_average_ch_energy[32].index,  \
                      m_average_ch_energy[33].index,  \
                      m_average_ch_energy[34].index   \
                      );
        NRF_LOG_DEBUG("%02d %02d",  \
                      m_average_ch_energy[35].index,  \
                      m_average_ch_energy[36].index
                      );


        advertise_request_update(MINIMUM_CHANNEL_SURVEY_SELECTION);

        update_channel_survey_status = true;

}

                      
static void process_channel_survey_report(ble_gap_evt_qos_channel_survey_report_t * p_report)
{
        for (uint8_t i = 0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                float energy_sample = p_report->channel_energy[i];

                if (energy_sample == BLE_GAP_POWER_LEVEL_INVALID)
                {
                        continue;
                }

                if (m_num_channel_survey_reports_received == 0)
                {
                        m_average_ch_energy[i].ch_energy = energy_sample;
                }
                else
                {
                        m_average_ch_energy[i].ch_energy =  (m_num_channel_survey_reports_received * m_average_ch_energy[i].ch_energy + energy_sample)
                                                           / (m_num_channel_survey_reports_received + 1);
                }
        }
}

uint32_t channel_map_request_update(uint16_t conn_handle, uint8_t first_best_channel_number)
{
        ret_code_t err_code;
        uint8_t number_channel_request = 0;
        ble_gap_opt_ch_map_t channel_map = {0};

       // if (conn_handle == BLE_CONN_HANDLE_INVALID)
       // {
         //       NRF_LOG_ERROR("Failure: because of disconnection!");
          //      return -1;
     //   }

        NRF_LOG_DEBUG("channel_map_request_update!!");

        channel_map.conn_handle = conn_handle;  //we get conn_handle on a CONNECT event

        for (uint8_t i=0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                uint8_t freq_index = m_average_ch_energy[i].index;

                if (freq_index != ADVERTISING_CHANNEL_37 && freq_index != ADVERTISING_CHANNEL_38 && freq_index != ADVERTISING_CHANNEL_39)
                {
                        if (freq_index < 8)
                        {
                                channel_map.ch_map[0] |= 1 << (freq_index);
                                //NRF_LOG_INFO("< 8, %d %x", freq_index, 1 << (freq_index));
                        }
                        else if (freq_index < 16)
                        {
                                channel_map.ch_map[1] |= 1 << (freq_index-8);
                                // NRF_LOG_INFO("< 16, %d %x", freq_index, 1 << (freq_index - 8));
                        }
                        else if (freq_index < 24)
                        {
                                channel_map.ch_map[2] |= 1 << (freq_index-16);
                                // NRF_LOG_INFO("< 24, %d %x", freq_index, 1 << (freq_index - 16));
                        }
                        else if (freq_index < 32)
                        {
                                channel_map.ch_map[3] |= 1 << (freq_index-24);
                                // NRF_LOG_INFO("< 32, %d %x", freq_index, 1 << (freq_index - 24));
                        }
                        else
                        {
                                channel_map.ch_map[4] |= 1 << (freq_index-32);
                                // NRF_LOG_INFO("%d %x", freq_index, 1 << (freq_index - 32));
                        }
                        number_channel_request++;
                }
                if (number_channel_request > first_best_channel_number)
                {
                        break;
                }
        }
        NRF_LOG_HEXDUMP_DEBUG(channel_map.ch_map, 5);

        err_code = sd_ble_opt_set(BLE_GAP_OPT_CH_MAP, (ble_opt_t *)&channel_map);
        APP_ERROR_CHECK(err_code);

        update_channel_survey_status = false;
}

void advertise_request_update(uint8_t first_best_channel_number)
{
        ret_code_t err_code;
        uint8_t number_channel_request = 0;
        ble_gap_opt_ch_map_t channel_map = {0};
        uint8_t data[] = "    ";

        NRF_LOG_DEBUG("advertise request update!!");

        for (uint8_t i=0; i < BLE_GAP_CHANNEL_COUNT; i++)
        {
                uint8_t freq_index = m_average_ch_energy[i].index;

                if (freq_index != ADVERTISING_CHANNEL_37 && freq_index != ADVERTISING_CHANNEL_38 && freq_index != ADVERTISING_CHANNEL_39)
                {
                        if (freq_index < 8)
                        {
                                channel_map.ch_map[0] |= 1 << (freq_index);
                                data[0] |= 1 << (freq_index);
                                //NRF_LOG_INFO("< 8, %d %x", freq_index, 1 << (freq_index));
                        }
                        else if (freq_index < 16)
                        {
                                channel_map.ch_map[1] |= 1 << (freq_index-8);
                                data[1] |= 1 << (freq_index-8);
                                // NRF_LOG_INFO("< 16, %d %x", freq_index, 1 << (freq_index - 8));
                        }
                        else if (freq_index < 24)
                        {
                                channel_map.ch_map[2] |= 1 << (freq_index-16);
                                data[2] |= 1 << (freq_index-16);
                                // NRF_LOG_INFO("< 24, %d %x", freq_index, 1 << (freq_index - 16));
                        }
                        else if (freq_index < 32)
                        {
                                channel_map.ch_map[3] |= 1 << (freq_index-24);
                                data[3] |= 1 << (freq_index-24);
                                // NRF_LOG_INFO("< 32, %d %x", freq_index, 1 << (freq_index - 24));
                        }
                        else
                        {
                                channel_map.ch_map[4] |= 1 << (freq_index-32);
                                data[4] |= 1 << (freq_index-32);
                                // NRF_LOG_INFO("%d %x", freq_index, 1 << (freq_index - 32));
                        }
                        number_channel_request++;
                }
                if (number_channel_request > first_best_channel_number)
                {
                        break;
                }
        }
        NRF_LOG_HEXDUMP_DEBUG(channel_map.ch_map, 5);


        
    // update advertising
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data
    


    /* get channel map please */

    manuf_data.company_identifier             =  0x0059; //Nordics company ID
    manuf_data.data.p_data                    = data;
    manuf_data.data.size                      = sizeof(data);
    

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    advdata.p_manuf_specific_data = &manuf_data;
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = m_adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;


    sd_ble_gap_adv_stop(m_adv_handle);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Advertising");
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Advertising updated");
    
   // err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

   //update advertising

   update_channel_survey_status = false;
}


uint32_t channel_survey_get_report_event(ble_gap_evt_qos_channel_survey_report_t *channel_survey_report)
{
        uint32_t err_code;

        process_channel_survey_report(channel_survey_report);
        m_num_channel_survey_reports_received++;


        if (m_num_channel_survey_reports_received > NUM_REPORTS_BEFORE_AVERAGE_PRINTOUT)
        {
        
                err_code = sd_ble_gap_qos_channel_survey_stop();
                APP_ERROR_CHECK(err_code);

                //NRF_LOG_INFO("Stop Channel Survey received count %d", m_num_channel_survey_reports_received);

                err_code = app_sched_event_put(NULL, 0, update_and_sort_channel_survey_handler);
                

                APP_ERROR_CHECK(err_code);
                m_num_channel_survey_reports_received = 0;

        }

}


/**@brief Function for handling BLE Stack events that are common to both the central and peripheral roles.
 * @param[in] conn_handle Connection Handle.
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const * p_ble_evt)
{
ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;


    char        passkey[BLE_GAP_PASSKEY_LEN + 1];
    uint16_t    role = ble_conn_state_role(conn_handle);

    pm_handler_secure_on_connection(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {


      case BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT:
          
          channel_survey_get_report_event(&p_ble_evt->evt.gap_evt.params.qos_channel_survey_report);
           
           break;



      case BLE_GAP_EVT_ADV_REPORT:
           on_adv_report(p_ble_evt);
           
            break;

        case BLE_GAP_EVT_CONNECTED:
            m_connected_peers[conn_handle].is_connected = true;
            m_connected_peers[conn_handle].address = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
            multi_qwr_conn_handle_assign(conn_handle);

            

//update channel map after survey if it is first time connected

if(is_survey == 1){
NRF_LOG_INFO("From Survey");
channel_map_request_update(m_conn_handle_hrs_c, MINIMUM_CHANNEL_SURVEY_SELECTION);

NRF_LOG_INFO("Waiting 10 Sec");
nrf_delay_ms(10000);
NRF_LOG_INFO("Wait Done");
}                

else if (is_survey == 0){
NRF_LOG_INFO("From Controller");
sd_ble_opt_set(BLE_GAP_OPT_CH_MAP, &new_opt_config) ;
NRF_LOG_INFO("Set New Channel Map");
// set new channel map for test

NRF_LOG_INFO("Waiting 10 Sec");
nrf_delay_ms(10000);
NRF_LOG_INFO("Wait Done");
}

    // update advertising
    
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data
    


    /* get channel map please */

    ble_opt_t opt;
    memset(&opt, 0, sizeof(opt));
    sd_ble_opt_get(BLE_GAP_OPT_CH_MAP, &opt);

    uint8_t data[]                            = "    ";

    data[0] = opt.gap_opt.ch_map.ch_map[0];
    data[1] = opt.gap_opt.ch_map.ch_map[1];
    data[2] = opt.gap_opt.ch_map.ch_map[2];
    data[3] = opt.gap_opt.ch_map.ch_map[3];
    data[4] = opt.gap_opt.ch_map.ch_map[4];

    manuf_data.company_identifier             =  0x0059; //Nordics company ID
    manuf_data.data.p_data                    = data;
    manuf_data.data.size                      = sizeof(data);
    

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    advdata.p_manuf_specific_data = &manuf_data;
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = m_adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;


    sd_ble_gap_adv_stop(m_adv_handle);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Advertising");
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Advertising updated");
    
   // err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

            //update advertising

            //stop advertising
            sd_ble_gap_adv_stop(m_adv_handle);

            //packet error rate
            packet_error_rate_reset_counter();
            packet_error_rate_detect_enable();
            
            err_code = sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle, 1, 2);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
                if (rssi_count % 10 == 0)
                {
                        packet_error_rate_timeout_handler();
                        NRF_LOG_INFO("RSSI = %d (dBm), PSR = %03d%%", p_ble_evt->evt.gap_evt.params.rssi_changed.rssi, get_packet_success_rate());
                }
                rssi_count++;           

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            memset(&m_connected_peers[conn_handle], 0x00, sizeof(m_connected_peers[0]));
            NRF_LOG_INFO("Disconnected");
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_SEC_PARAMS_REQUEST", nrf_log_push(roles_str[role]));
            break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, BLE_GAP_PASSKEY_LEN);
            passkey[BLE_GAP_PASSKEY_LEN] = 0x00;
            NRF_LOG_INFO("%s: BLE_GAP_EVT_PASSKEY_DISPLAY: passkey=%s match_req=%d",
                         nrf_log_push(roles_str[role]),
                         nrf_log_push(passkey),
                         p_ble_evt->evt.gap_evt.params.passkey_display.match_request);

            if (p_ble_evt->evt.gap_evt.params.passkey_display.match_request)
            {
                on_match_request(conn_handle, role);
            }
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_KEY_REQUEST", nrf_log_push(roles_str[role]));
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST", nrf_log_push(roles_str[role]));
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          nrf_log_push(roles_str[role]),
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            ret_code_t err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events that are related to central application.
 *
 * @details This function keeps the connection handles of central application up-to-date. It
 * parses scanning reports, initiates a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report the central application's activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              must be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t const * p_ble_evt)
{
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    ret_code_t            err_code;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected (HR or RSC), initiate DB
        //  discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("CENTRAL: Connected, handle: %d.", p_gap_evt->conn_handle);
            
            // If no Heart Rate Sensor is currently connected, try to find them on this peripheral.
            if (m_conn_handle_hrs_c == BLE_CONN_HANDLE_INVALID)
            {
                NRF_LOG_INFO("CENTRAL: Searching for HRS on conn_handle 0x%x", p_gap_evt->conn_handle);

                err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
                APP_ERROR_CHECK(err_code);
            }
            // Update status of LEDs.
            bsp_board_led_off(CENTRAL_SCANNING_LED);
            bsp_board_led_on(CENTRAL_CONNECTED_LED);

            //disconnect after advertising updated
            //sd_ble_gap_disconnect(p_gap_evt->conn_handle, BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION);
            NRF_LOG_INFO("disconnected");

        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the status of LEDs, and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("CENTRAL: Disconnected, handle: %d, reason: 0x%x",
                         p_gap_evt->conn_handle,
                       p_gap_evt->params.disconnected.reason);

            // Update the status of LEDs.
            bsp_board_led_off(CENTRAL_CONNECTED_LED);
            bsp_board_led_on(CENTRAL_SCANNING_LED);

            if (p_gap_evt->conn_handle == m_conn_handle_hrs_c)
            {
                ble_uuid_t target_uuid = {.uuid = BLE_UUID_HEART_RATE_SERVICE, .type = BLE_UUID_TYPE_BLE};
                m_conn_handle_hrs_c    = BLE_CONN_HANDLE_INVALID;

                err_code = nrf_ble_scan_filter_set(&m_scan, 
                                                   SCAN_UUID_FILTER, 
                                                   &target_uuid);
                APP_ERROR_CHECK(err_code);
            }
            
            scan_start();
        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only connection attemps can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("CENTRAL: Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("CENTRAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("CENTRAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling BLE Stack events that involves peripheral applications. Manages the
 * LEDs used to report the status of the peripheral applications.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_peripheral_evt(ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("PERIPHERAL: Connected, handle %d.", p_ble_evt->evt.gap_evt.conn_handle);
            bsp_board_led_off(PERIPHERAL_ADVERTISING_LED);
            bsp_board_led_on(PERIPHERAL_CONNECTED_LED);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("PERIPHERAL: Disconnected, handle %d, reason 0x%x.",
                         p_ble_evt->evt.gap_evt.conn_handle,
                         p_ble_evt->evt.gap_evt.params.disconnected.reason);
            // LED indication will be changed when advertising starts.
        break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("PERIPHERAL: GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("PERIPHERAL: GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    uint16_t role        = ble_conn_state_role(conn_handle);

    if (    (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
        &&  (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr)))
    {
        NRF_LOG_INFO("%s: Already connected to this device as %s (handle: %d), disconnecting.",
                     (role == BLE_GAP_ROLE_PERIPH) ? "PERIPHERAL" : "CENTRAL",
                     (role == BLE_GAP_ROLE_PERIPH) ? "CENTRAL"    : "PERIPHERAL",
                     conn_handle);

        (void)sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

        // Do not process the event further.
        return;
    }

    on_ble_evt(conn_handle, p_ble_evt);

    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        on_ble_peripheral_evt(p_ble_evt);
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {

        on_ble_central_evt(p_ble_evt);
    }

}


/**@brief Function for initializing the Heart Rate Service client. */
static void hrs_c_init(void)
{
    ret_code_t       err_code;
    ble_hrs_c_init_t hrs_c_init_obj;

    hrs_c_init_obj.evt_handler   = hrs_c_evt_handler;
    hrs_c_init_obj.error_handler = hrs_c_error_handler;
    hrs_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_hrs_c_init(&m_hrs_c, &hrs_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

     // Configure the BLE stack by using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

        //channel survey configuration
    /* Make Channel Survey feature available to the application */
        ble_cfg_t ble_cfg;
        // Configure the GATTS attribute table.
        memset(&ble_cfg, 0x00, sizeof(ble_cfg));
        ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = NRF_SDH_BLE_PERIPHERAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.central_role_count = NRF_SDH_BLE_CENTRAL_LINK_COUNT;
        ble_cfg.gap_cfg.role_count_cfg.qos_channel_survey_role_available = true; /* Enable channel survey role */

        err_code = sd_ble_cfg_set(BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, &ram_start);
        if (err_code != NRF_SUCCESS)
        {
                NRF_LOG_ERROR("sd_ble_cfg_set() returned %s when attempting to set BLE_GAP_CFG_ROLE_COUNT.",
                              nrf_strerror_get(err_code));
        }


    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);


            err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        APP_ERROR_CHECK(err_code);

        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);


    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Peer Manager. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_params;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_params, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_params.bond           = SEC_PARAMS_BOND;
    sec_params.mitm           = SEC_PARAMS_MITM;
    sec_params.lesc           = SEC_PARAMS_LESC;
    sec_params.keypress       = SEC_PARAMS_KEYPRESS;
    sec_params.io_caps        = SEC_PARAMS_IO_CAPABILITIES;
    sec_params.oob            = SEC_PARAMS_OOB;
    sec_params.min_key_size   = SEC_PARAMS_MIN_KEY_SIZE;
    sec_params.max_key_size   = SEC_PARAMS_MAX_KEY_SIZE;
    sec_params.kdist_own.enc  = 1;
    sec_params.kdist_own.id   = 1;
    sec_params.kdist_peer.enc = 1;
    sec_params.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_params);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Delete all data stored for all peers. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for accepting or rejecting a numeric comparison. */
static void num_comp_reply(uint16_t conn_handle, bool accept)
{
    uint8_t    key_type;
    ret_code_t err_code;

    if (accept)
    {
        NRF_LOG_INFO("Numeric Match. Conn handle: %d", conn_handle);
        key_type = BLE_GAP_AUTH_KEY_TYPE_PASSKEY;
    }
    else
    {
        NRF_LOG_INFO("Numeric REJECT. Conn handle: %d", conn_handle);
        key_type = BLE_GAP_AUTH_KEY_TYPE_NONE;
    }

    err_code = sd_ble_gap_auth_key_reply(conn_handle,
                                         key_type,
                                         NULL);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for handling button presses for numeric comparison match requests. */
static void on_num_comp_button_press(bool accept)
{
    // Check whether any links have pending match requests, and if so, send a reply.
    if (m_conn_handle_num_comp_central != BLE_CONN_HANDLE_INVALID)
    {
        num_comp_reply(m_conn_handle_num_comp_central, accept);
        m_conn_handle_num_comp_central = BLE_CONN_HANDLE_INVALID;
    }
    else if (m_conn_handle_num_comp_peripheral != BLE_CONN_HANDLE_INVALID)
    {
        num_comp_reply(m_conn_handle_num_comp_peripheral, accept);
        m_conn_handle_num_comp_peripheral = BLE_CONN_HANDLE_INVALID;
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, 87);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != NRF_ERROR_RESOURCES) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                )
            {
                APP_ERROR_HANDLER(err_code);
            }

            on_num_comp_button_press(true);
            break;

      case BSP_EVENT_KEY_1:
            on_num_comp_button_press(false);
            break;

        default:
            break;
    }
}


/**@brief Function for initializing buttons and LEDs.
 *
 * @param[out] p_erase_bonds  True if the clear bonding button is pressed to
 *                            wake the application up. False otherwise.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the GAP.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device, including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params = m_scan.conn_params;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function is passed to each service that may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write instances.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++)
    {
        err_code = nrf_ble_qwr_init(&m_qwr[i], &qwr_init_obj);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID; // Start upon connection.
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;  // Ignore events.
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_hrs_on_db_disc_evt(&m_hrs_c, p_evt);
}


/**@brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(db_init));

    db_init.evt_handler =  db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Heart Rate service. */
static void hrs_init(void)
{
    ret_code_t     err_code;
    ble_hrs_init_t hrs_init_params;
    uint8_t        body_sensor_location;

    // Initialize the Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init_params, 0, sizeof(hrs_init_params));

    hrs_init_params.evt_handler                 = NULL;
    hrs_init_params.is_sensor_contact_supported = true;
    hrs_init_params.p_body_sensor_location      = &body_sensor_location;

    // Require LESC with MITM (Numeric Comparison).
    hrs_init_params.hrm_cccd_wr_sec = SEC_MITM;
    hrs_init_params.bsl_rd_sec      = SEC_MITM;

    err_code = ble_hrs_init(&m_hrs, &hrs_init_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the advertising functionality. */
static void advertising_init(void)
{   

    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    ble_advdata_manuf_data_t                  manuf_data; //Variable to hold manufacturer specific data


    /* get channel map please */

  ble_opt_t opt;
  memset(&opt, 0, sizeof(opt));
  sd_ble_opt_get(BLE_GAP_OPT_CH_MAP, &opt);

    uint8_t data[]                            = "    ";
    data[0] = opt.gap_opt.ch_map.ch_map[0];
    data[1] = opt.gap_opt.ch_map.ch_map[1];
    data[2] = opt.gap_opt.ch_map.ch_map[2];
    data[3] = opt.gap_opt.ch_map.ch_map[3];
    data[4] = opt.gap_opt.ch_map.ch_map[4];

    manuf_data.company_identifier             =  0x0059; //Nordics company ID
    manuf_data.data.p_data                    = data;
    manuf_data.data.size                      = sizeof(data);
    



    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.p_manuf_specific_data = &manuf_data;

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = m_adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log or key operations, or both, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;
    
    err_code = nrf_ble_lesc_request_handler();

    app_sched_execute();
    APP_ERROR_CHECK(err_code);
    
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
        APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}



int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timer_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    scheduler_init();
    scan_init();
    gap_params_init();
    gatt_init();
    conn_params_init();
    db_discovery_init();
    qwr_init();
    hrs_init();
    hrs_c_init();
    peer_manager_init();
    advertising_init();

    uint32_t              err_code;

    // start channel survey
    
    NRF_LOG_INFO("Start channel survey");
    err_code = connection_channel_survey_start();
    APP_ERROR_CHECK(err_code);


    // Start execution.
    NRF_LOG_INFO("LE Secure Connections example started.");

    if (erase_bonds == true)
    {
        delete_bonds();
        // Scanning and advertising is started by PM_EVT_PEERS_DELETE_SUCEEDED.
    }
    else
    {
        adv_scan_start();
 
    }

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
