/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "bsp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_twi.h"
#include "sht4x.h"
#include "lis3dh.h"
#include "nrf_gpio.h"
#include "ble_bas.h"
#include "battery_voltage.h"

//const uint8_t device_name_arr[2]        = {'N', 2};
//#define DEVICE_NAME                     device_name_arr//"Nordic_Template"      /**< Name of device. Will be included in the advertising data. */
//#define MANUFACTURER_NAME               "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                32 //32 //80 = 50ms //300 karna hae                  /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

//#define APP_ADV_DURATION                2 //200 = 2s//1800 50, 300 ,20 karna hae                 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define ADV_INTERVAL                    APP_TIMER_TICKS(1000*30)                /**< Interval after which device will send notification (in milli seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

 
#define APP_BEACON_INFO_LENGTH						 0x1B   //packet size= 27 Bytes
#define SENSOR_TYPE2                       0x02
#define WORKING_COND                       0x01
#define SENSOR_TYPE1                       0x01


#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define ACC_VDD                         20                                      
#define TEMP_VDD                        28
#define ACC_SDO                         19
static uint32_t button_tick = 0;
BLE_BAS_DEF(m_bas); 
NRF_BLE_QWR_DEF(m_qwr);      


static ble_gap_adv_params_t m_adv_params;
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;


static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


//APP_TIMER_DEF(m_adv_timer_id);                                         /**< timer to send notification. */


uint8_t  battery_level; 
int v_g_battery;                                                                                 
#define TOP_INDEX                       152
uint8_t num_packets= TOP_INDEX;
#define BT0_LONG_PUSH  BSP_EVENT_KEY_0


APP_TIMER_DEF(m_repeated_timer_id); 
APP_TIMER_DEF(motion_sensor_timer_id); 
APP_TIMER_DEF(sensor_timer_id); 
APP_TIMER_DEF(m_battery_timer_id);

uint8_t battery=100;
uint8_t data_to_advertise[27]; 
//SHT40 variable
float temp;
float hum;
int16_t temp1,hum1;
int flag=0;                    // variable to count two readings
int g_sleep_flag=1;
uint8_t flag1=0;



extern int acc_working_condition;
uint8_t temp_working_condition=0;
int16_t tmp_int16[3] = {0};
static int Motion_index=0;

//static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
static uint8_t              m_enc_advdata[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];

static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata[0],
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};


#define ADV_DATA_UPDATE_INTERVAL        APP_TIMER_TICKS(15)     //(25)              //5000 karna hae  - 120000       500
#define ADV_DATA_UPDATE                 APP_TIMER_TICKS(10000)                      //supposed to be 60000*4           
#define Motion_Data_Update              APP_TIMER_TICKS(100)
#define BATTERY_DATA_UPDATE             APP_TIMER_TICKS(10) 


static const nrf_drv_twi_t i2c1_instance = NRF_DRV_TWI_INSTANCE(0);
static const nrf_drv_twi_t  m_twi = NRF_DRV_TWI_INSTANCE(1);

//#define PAYLOAD_SIZE                    1

#define APP_COMPANY_IDENTIFIER          0x0059                                   /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

typedef struct
{
uint8_t x_msb;
uint8_t x_lsb;	
uint8_t y_msb;
uint8_t y_lsb;	
uint8_t z_msb;
uint8_t z_lsb;	
}motion_data;
motion_data Motion_data[600];                // structure to hold Lis3dh data 8
uint8_t*ptr;


//static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/* YOUR_JOB: Declare all services structure your application is using
 *  BLE_XYZ_DEF(m_xyz);
 */

// YOUR_JOB: Use UUIDs for service(s) used in your application.
//static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
//{
//    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
//};

static void advertising_init(void);
static void advertising_start(void);
static void advertising_update(void);

void send_notification(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */



static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);  //all LEDs are off
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_app_evt_wait(); //sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void battery_services_init(void)
{
        ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_bas_init_t bas_init;
    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
   
	  memset(&bas_init, 0, sizeof(bas_init));
		
		bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;
		
		bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
  
}




static void battery_level_update(void)
{
    ret_code_t err_code;
    // uint8_t  battery_level;
    uint16_t vbatt; 
    battery_voltage_get(&vbatt);                                 // Get new battery voltage
		NRF_LOG_INFO("ADC result: %d\r\n", vbatt);

    battery_level = battery_level_in_percent(vbatt);            //Transform the millivolts value into battery level percent.
    NRF_LOG_INFO("ADC result in percent: %d\r\n", battery_level);
  
    err_code = ble_bas_battery_level_update(&m_bas, battery_level, m_conn_handle);
	 	NRF_LOG_INFO("ADC result: %d\r\n", battery_level);
    if(v_g_battery==1)
		{			
    if ( (err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
  	}
}
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    NRF_LOG_INFO("Battery Level timeout event");
    
    // Only send the battery level update if we are connected
  //   if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
           
		     battery_level_update();
		 
		num_packets=num_packets-1;
		advertising_init();	 
		advertising_start();
	  ret_code_t error_code;				
    error_code = app_timer_start(m_repeated_timer_id, 
                                    ADV_DATA_UPDATE_INTERVAL, 
                                    NULL);
  	APP_ERROR_CHECK(error_code);	
	
    
}


static void repeated_timer_handler(void * p_context)
{
		//ret_code_t err_code; 

		
		advertising_update();
	  //advertising_init();	 
		
		
    //NRF_LOG_INFO("Advertising start");
	  //NRF_LOG_INFO("Number of packets remaining %d",num_packets);
		
	
		//advertising_start();    //start advertising and blink LED1
	
				/*if(num_packets==0)
			{
	  	Motion_index=0;
  		num_packets=TOP_INDEX;
  	  //remaining_packet=152;
      app_timer_stop(m_repeated_timer_id);	
			
		  ret_code_t error_code;
			
			sleep_mode_enter();
      error_code = app_timer_start(sensor_timer_id,                                         
                                    ADV_DATA_UPDATE, 
                                    NULL);
      APP_ERROR_CHECK(error_code);
      acc_working_condition=0; 
	  	temp_working_condition=0;	
			
			  //after all the data has been sent, enter sleep mode, stop blinking LEDs
		 }*/

 	
}




static void motion_sensor_timer_handler(void *p_context)
{
    float LIS3DH_Acc[3];
	  nrf_gpio_pin_set(ACC_VDD);
	  nrf_gpio_pin_set(ACC_SDO);
	  twi_init(); 
	  LIS3DH_Init();                                  //LIS3DH Sensor Init
	  LIS3DH_GetAccData(LIS3DH_Acc);
    int16_t tmp_int16[3] = {0};
 
     tmp_int16[0] =(int16_t)( LIS3DH_Acc[0]);
		 tmp_int16[1] =(int16_t)( LIS3DH_Acc[1]);
		 tmp_int16[2] =(int16_t)( LIS3DH_Acc[2]);
		

		 NRF_LOG_INFO(" value of temp x %d ",tmp_int16[0]);
		 NRF_LOG_INFO(" value of temp y %d ",tmp_int16[1]);
		 NRF_LOG_INFO(" value of temp z %d ",tmp_int16[2]);
		
	  	Motion_data[flag].x_msb= (uint8_t)((tmp_int16[0] & 0xFF00)>>8);
		  Motion_data[flag].x_lsb= (int8_t)(tmp_int16[0] & 0xFF);
	  	Motion_data[flag].y_msb= (int8_t)((tmp_int16[1] & 0xFF00)>>8);
		  Motion_data[flag].y_lsb= (int8_t)(tmp_int16[1] & 0xFF);
	  	Motion_data[flag].z_msb= (int8_t)((tmp_int16[2] & 0xFF00)>>8);
	  	Motion_data[flag].z_lsb= (int8_t)(tmp_int16[2] & 0xFF);
	//	 NRF_LOG_INFO(" value of x msb %d ",Motion_data[flag].x_msb);
	//	 NRF_LOG_INFO(" value of x lsb %d ",Motion_data[flag].x_lsb);
	//	 NRF_LOG_INFO(" value of y msb %d ",Motion_data[flag].y_msb);
	//	 NRF_LOG_INFO(" value of y lsb %d ",Motion_data[flag].y_lsb);
  //   NRF_LOG_INFO(" value of z msb %d ",Motion_data[flag].z_msb);
	//	 NRF_LOG_INFO(" value of z lsb %d ",Motion_data[flag].z_lsb);
	   NRF_LOG_INFO(" value of flag %d ",flag);
	 	 ++flag;
				
	   nrf_drv_twi_disable(&m_twi);          // disable twi
	   nrf_drv_twi_uninit(&m_twi);  
	 
	   if(flag==600)
	   {
	     flag=0;		         
		   app_timer_stop(motion_sensor_timer_id);				
		   ret_code_t error_code;
			 
       error_code = app_timer_start(m_battery_timer_id, 
                                    BATTERY_DATA_UPDATE, 
                                    NULL);
       APP_ERROR_CHECK(error_code);	
       nrf_gpio_pin_clear(ACC_VDD);	
       nrf_gpio_pin_clear(ACC_SDO);			 
		
    	}
			  	
	
}




static void sensor_timer_handler(void *p_context)
{
 
	    UNUSED_PARAMETER(p_context);
	     nrf_gpio_pin_set(TEMP_VDD); 
     	 sensirion_i2c_init();
			 while (sht4x_probe() != STATUS_OK) 
				{
						NRF_LOG_INFO("SHT sensor probing failed\n");
						nrf_delay_ms(200);
				}
				
        NRF_LOG_INFO("SHT sensor probing successful\n");
        int32_t temperature, humidity;
        /* Measure temperature and relative humidity and store into variables
         * temperature, humidity (each output multiplied by 1000).
         */
        int8_t ret = sht4x_measure_blocking_read(&temperature, &humidity);
        if (ret == STATUS_OK)
				{
						 NRF_LOG_INFO("measured temperature: %3.2f degreeCelsius,"
													"measured humidity: %3.2f percentRH\n",temperature / 1000.0f, humidity / 1000.0f);
													 temp =temperature / 1000.0f;          
													 hum =humidity / 1000.0f;                     
                           temp_working_condition=1;
						NRF_LOG_INFO("temp=%f,hum=%f",temp,hum);
				} 
			  else 
			   {
						NRF_LOG_INFO("error reading measurement\n");
			   }

			 		// disable and uninitialize twi
		    nrf_drv_twi_disable(&i2c1_instance);
		    nrf_drv_twi_uninit(&i2c1_instance); 		
			  temp1=temp*100;
			  hum1=hum*100;	
	      nrf_gpio_pin_clear(TEMP_VDD);
			  ret_code_t error_code;
        error_code = app_timer_start(motion_sensor_timer_id, Motion_Data_Update, NULL);																		
        APP_ERROR_CHECK(error_code);		 			 			 
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.

 
//		 // Timer to start/stop sending notification
//		err_code = app_timer_create(&m_adv_timer_id, APP_TIMER_MODE_REPEATED, adv_timeout_handler);
//    APP_ERROR_CHECK(err_code);
	
	   // advertising
	  err_code = app_timer_create(&m_repeated_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeated_timer_handler);
    APP_ERROR_CHECK(err_code);
    
	  // temp sensor
	  err_code = app_timer_create(&sensor_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT, 
                                sensor_timer_handler);
    APP_ERROR_CHECK(err_code);
	
	  // motion sensor
	  err_code = app_timer_create(&motion_sensor_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                motion_sensor_timer_handler);
    APP_ERROR_CHECK(err_code);
	
	
		err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
		ble_gap_addr_t          m_adv_addr;
		
	//SETTING CUSTOM RANDOM STATIC DEVICE ADDRESS (6 bytes)
		m_adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC; //address type
		m_adv_addr.addr[0] = 0xfd;
		m_adv_addr.addr[1] = 0x71;
		m_adv_addr.addr[2] = 0x5f;
		m_adv_addr.addr[3] = 0x45;
		m_adv_addr.addr[4] = 0x2b;
		m_adv_addr.addr[5] = 0xC6; // 2MSB must be set 11
	
		sd_ble_gap_addr_set(&m_adv_addr);
    
}




/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */

/**@brief Function for initializing the Connection Parameters module.
 */

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{	 
			 
	   ret_code_t error_code;
     error_code = app_timer_start(sensor_timer_id, 
                                    ADV_DATA_UPDATE, 
                                    NULL);
     APP_ERROR_CHECK(error_code);

}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
				     v_g_battery=0;
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
				     v_g_battery=1;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
 
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    
		NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
		uint32_t      err_code;
    ble_advdata_t advdata;
  
		//static uint8_t payload_index = 0;
	
	
		//num_packets=num_packets-1;    //when the first packet is sent, number of packets remaining reduces to TOP_INDEX-1
	
	
	uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
		{
			num_packets,
			SENSOR_TYPE1,
			temp_working_condition,
		 (uint8_t)((temp1 & 0xFF00) >> 8),
			(uint8_t)(temp1 & 0xFF),
			battery_level, 
		};
		

		
	// Manufacturing data
		
		
		ble_advdata_manuf_data_t 						    manuf_data; //Variable to hold manufacturer specific data
    manuf_data.company_identifier           = 0x0059; //Nordics company ID
		manuf_data.data.p_data                  = (uint8_t *) m_beacon_info;
		manuf_data.data.size                    = APP_BEACON_INFO_LENGTH;
		
		memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.p_manuf_specific_data = &manuf_data;

    memset(&m_adv_params, 0, sizeof(m_adv_params));		
		
		
	  m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED; //Beacon needs to be scannable to receive scan response
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
		m_adv_params.interval        = APP_ADV_INTERVAL;
		//m_adv_params.duration        = APP_ADV_DURATION;
		//m_adv_params.max_adv_evts    = 1;
		//m_adv_params.channel_mask[4] = 0xC0;   //use only channel-37 for advertising

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

		//Configure the advertising set- the data that you wish to send, and the parameters that you wish to use.
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

}


static void advertising_update(void)
{
		
		uint32_t      err_code;
    ble_advdata_t advdata;
  
		//static uint8_t payload_index = 0;
	
	
		//num_packets=num_packets-1;    //when the first packet is sent, number of packets remaining reduces to TOP_INDEX-1
	
	num_packets=num_packets-1;
	uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
		{
			num_packets,
			SENSOR_TYPE1,
			temp_working_condition,
		 (uint8_t)((temp1 & 0xFF00) >> 8),
			(uint8_t)(temp1 & 0xFF),
			battery_level, 
		};
		

		
	// Manufacturing data
		
		
		ble_advdata_manuf_data_t 						    manuf_data; //Variable to hold manufacturer specific data
    manuf_data.company_identifier           = 0x0059; //Nordics company ID

		if(num_packets<TOP_INDEX && num_packets>=TOP_INDEX-2)    //when the first two packets have been sent, we start advertising accelerometer data
		{
			manuf_data.data.p_data                  = (uint8_t *) m_beacon_info;
		}
		
		else if(num_packets>=0)
		{
			ptr=&Motion_data[Motion_index].x_msb;	
			uint8_t m_beacon_info1[APP_BEACON_INFO_LENGTH] =                    /**< accelerometer data advertised by the Beacon. */
				{
					num_packets,
					SENSOR_TYPE2,
					acc_working_condition,
					*ptr,
					*(ptr+1),
					*(ptr+2),
					*(ptr+3),
					*(ptr+4),
					*(ptr+5),
					*(ptr+6),
					*(ptr+7),
					*(ptr+8),
					*(ptr+9),
					*(ptr+10),
					*(ptr+11),
					*(ptr+12),
					*(ptr+13),
					*(ptr+14),
					*(ptr+15),
					*(ptr+16),
					*(ptr+17),
					*(ptr+18),
					*(ptr+19),
					*(ptr+20),
					*(ptr+21),
					*(ptr+22),
					*(ptr+23),
				};
				Motion_index=Motion_index+4;
		manuf_data.data.p_data                  = (uint8_t *) m_beacon_info1;
		}
		if(num_packets==255)
			{
			sd_ble_gap_adv_stop(m_adv_handle);   //stop advertising
	  	Motion_index=0;
  		num_packets=TOP_INDEX;
  	  //remaining_packet=152;
      app_timer_stop(m_repeated_timer_id);	
			
		  ret_code_t error_code;
			
			sleep_mode_enter();
      error_code = app_timer_start(sensor_timer_id,                                         
                                    ADV_DATA_UPDATE, 
                                    NULL);
      APP_ERROR_CHECK(error_code);
      acc_working_condition=0; 
	  	temp_working_condition=0;	
			
			  //after all the data has been sent, enter sleep mode, stop blinking LEDs
		 }
	
		manuf_data.data.size                    = APP_BEACON_INFO_LENGTH;
		
		memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.p_manuf_specific_data = &manuf_data;

		m_adv_data.adv_data.p_data =
        (m_adv_data.adv_data.p_data == m_enc_advdata[0]) ? m_enc_advdata[1] : m_enc_advdata[0];
		
    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

		//Configure the advertising set- the data that you wish to send, and the parameters that you wish to use.
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, NULL);
    APP_ERROR_CHECK(err_code); 
		
		
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing LEDs.
 */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP
             
        case BT0_LONG_PUSH:                    // BSP_EVENT_KEY_0
				      NRF_LOG_INFO("Button is pressed.");
              g_sleep_flag=0;				
				     break;
        default:
            break;
    }
}

static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
	
		err_code = bsp_event_to_button_action_assign(BSP_BOARD_BUTTON_0,  BSP_BUTTON_ACTION_RELEASE , BT0_LONG_PUSH); //add release action trigger on the button. call BSP_EVENT_KEY_2 event  BSP_BUTTON_ACTION_RELEASE
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
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code; 
	
		err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);  //start advertising
    APP_ERROR_CHECK(err_code);
	
		err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}

static void setting_pin()
{
   NRF_LOG_INFO("Setting pin 28,19 and 20 as output");
	 nrf_gpio_cfg_output(TEMP_VDD);
	 nrf_gpio_cfg_output(ACC_VDD);
	 nrf_gpio_cfg_output(ACC_SDO);
	 nrf_gpio_pin_clear(TEMP_VDD);
	 nrf_gpio_pin_clear(ACC_VDD);
	 nrf_gpio_pin_clear(ACC_SDO);

}

void sleepmode()
{			
						bsp_wakeup_button_disable(BSP_BOARD_BUTTON_1);
						bsp_wakeup_button_disable(BSP_BOARD_BUTTON_2);
						bsp_wakeup_button_disable(BSP_BOARD_BUTTON_3);
						bsp_wakeup_button_enable(BSP_BOARD_BUTTON_0);
						bsp_indication_set(BSP_INDICATE_USER_STATE_0);
						nrf_delay_ms(50);
						bsp_indication_set(BSP_INDICATE_USER_STATE_OFF);
						NRF_LOG_INFO("Going to sleep.\r\n");
						nrf_delay_ms(100);			
						// Enter system OFF. After wakeup the chip will be reset, and code execution will run from the top 
						NRF_POWER->SYSTEMOFF = 1;																
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize.
    log_init();
    timers_init();
		//leds_init();
    buttons_leds_init();
		battery_voltage_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
   // gatt_init();
	
		setting_pin();
    battery_services_init();
    //peer_manager_init();

    // Start execution.
    NRF_LOG_INFO("Template example started.");

		NRF_LOG_INFO("Template example started.");
	 
      	while(bsp_button_is_pressed(BSP_BOARD_BUTTON_0) == true)
    {
				if (button_tick >= 65534*100) 
				{
					button_tick = 0;
					bsp_indication_set(BSP_INDICATE_USER_STATE_0);
					NRF_LOG_INFO("BeforeFivesec"); 
					g_sleep_flag = 0;
					break;
				}
			  button_tick++;
	  }		
		
		while(g_sleep_flag)
		{
		 // sd_power_system_off();
			  sleepmode();
		}
		
		application_timers_start();
		
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
