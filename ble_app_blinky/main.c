/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "app_scheduler.h"
#include "nordic_common.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "sdk_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_drv_clock.h"
#include "nrf_soc.h"

#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh_ble.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"



APP_TIMER_DEF(m_led_a_timer_id);


#define ST7586_SPI_INSTANCE  0/**< SPI instance index. */
static const nrf_drv_spi_t st7586_spi = NRF_DRV_SPI_INSTANCE(ST7586_SPI_INSTANCE);  /**< SPI instance. */
static volatile bool st7586_spi_xfer_done = false;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define ST_COMMAND	0
#define ST_DATA		1

#define RATIO_SPI0_LCD_SCK          4
#define RATIO_SPI0_LCD_A0		    28
#define RATIO_SPI0_LCD_MOSI		    29
#define RATIO_SPI0_LCD_BSTB		    30
#define RATIO_SPI0_LCD_CS			31

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(nrf_drv_gpiote_pin_t))
#define SCHED_QUEUE_SIZE                10

#define LED_1_PIN                       BSP_LED_0     // LED 1 on the nRF51-DK or nRF52-DK
#define LED_2_PIN                       BSP_LED_1     // LED 3 on the nRF51-DK or nRF52-DK
#define LED_3_PIN                       BSP_LED_2     // LED 3 on the nRF51-DK or nRF52-DK
#define BUTTON_1_PIN                    BSP_BUTTON_0  // Button 1 on the nRF51-DK or nRF52-DK
#define BUTTON_2_PIN                    BSP_BUTTON_1  // Button 2 on the nRF51-DK or nRF52-DK
#define BUTTON_3_PIN                    BSP_BUTTON_2  // Button 3 on the nRF51-DK or nRF52-DK
#define BUTTON_4_PIN                    BSP_BUTTON_3  // Button 3 on the nRF51-DK or nRF52-DK


#define H	0
#define HOR	true
#define VER false
#define PLUS true
#define MINUS false

#define LCD_INIT_DELAY(t) nrf_delay_ms(t)

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */

#define DEVICE_NAME                     "SNAKE"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */


static unsigned char rx_data;

int body = 10;
int doritosX =0, doritosY=0;
bool direction= HOR;
bool sign = PLUS;
bool inc = false;

/**
 * @brief SPI user event handler.
 * @param event
 */

struct snake
{
	int up;
	int down;
	int left;
	int right;
};


struct snake s[100];

uint8_t GenerateRandomNumber()
{
	uint8_t num_rand_bytes_available;
	uint8_t random_number;
	sd_rand_application_bytes_available_get(&num_rand_bytes_available);

	if(num_rand_bytes_available > 0)
	{
		sd_rand_application_vector_get(&random_number, 1);
	}

	return random_number;
}

void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
	st7586_spi_xfer_done = true;
}




void st7586_write(const uint8_t category, const uint8_t data)
{

	int err_code;


    nrf_gpio_pin_write(RATIO_SPI0_LCD_A0, category);

    st7586_spi_xfer_done = false;

    err_code = nrf_drv_spi_transfer(&st7586_spi, &data, 1, &rx_data, 0);


    APP_ERROR_CHECK(err_code);

    while (!st7586_spi_xfer_done) {
    	__WFE();
    }


    nrf_delay_us(10);

}


static inline void st7586_pinout_setup()
{
    // spi setup
	int err_code;
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin   = RATIO_SPI0_LCD_CS;
    spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.mosi_pin = RATIO_SPI0_LCD_MOSI;
    spi_config.sck_pin  = RATIO_SPI0_LCD_SCK;
    spi_config.frequency = NRF_SPI_FREQ_1M;
    spi_config.mode = NRF_DRV_SPI_MODE_3;

    err_code = nrf_drv_spi_init(&st7586_spi, &spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_set(RATIO_SPI0_LCD_A0);
    nrf_gpio_cfg_output(RATIO_SPI0_LCD_A0);

    nrf_gpio_pin_clear(RATIO_SPI0_LCD_BSTB);
    nrf_gpio_cfg_output(RATIO_SPI0_LCD_BSTB);
}


static inline void doritos()
{
	do
	{
		doritosX = GenerateRandomNumber();
		doritosY = GenerateRandomNumber();
	}while(!(doritosX<=156 && doritosY<=42 && doritosX%3==0));
}



static inline void st7586_initialization()
{
    nrf_delay_ms(10);

    nrf_gpio_pin_set(RATIO_SPI0_LCD_BSTB);

    nrf_delay_ms(120);
    st7586_write(ST_COMMAND, 0xD7);
    st7586_write(ST_DATA, 0x9F);
    st7586_write(ST_COMMAND, 0xE0);
    st7586_write(ST_DATA, 0x00);
    nrf_delay_ms(10);
    st7586_write(ST_COMMAND, 0xE3);
    nrf_delay_ms(20);
    st7586_write(ST_COMMAND, 0xE1);
    st7586_write(ST_COMMAND, 0x11);
    st7586_write(ST_COMMAND, 0x28);
    nrf_delay_ms(50);
    st7586_write(ST_COMMAND, 0xC0);
    st7586_write(ST_DATA, 0x53);
    st7586_write(ST_DATA, 0x01);
    st7586_write(ST_COMMAND, 0xC3);
    st7586_write(ST_DATA, 0x02);
    st7586_write(ST_COMMAND, 0xC4);
    st7586_write(ST_DATA, 0x06);
    st7586_write(ST_COMMAND, 0xD0);
    st7586_write(ST_DATA, 0x1D);
    st7586_write(ST_COMMAND, 0xB5);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_COMMAND, 0x39);
    st7586_write(ST_COMMAND, 0x3A);
    st7586_write(ST_DATA, 0x02);
    st7586_write(ST_COMMAND, 0x36);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_COMMAND, 0xB0);
    st7586_write(ST_DATA, 0x9F);
    st7586_write(ST_COMMAND, 0xB4);
    st7586_write(ST_DATA, 0xA0);
    st7586_write(ST_COMMAND, 0x30);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x77);
    st7586_write(ST_COMMAND, 0x20);

    st7586_write(ST_COMMAND, 0x2A);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x7F);
    st7586_write(ST_COMMAND, 0x2B);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x00);
    st7586_write(ST_DATA, 0x9F);

    st7586_write(ST_COMMAND, 0x2C);
    for(int i=0;i<20500;i++)
    {
    	st7586_write(ST_DATA, 0x00);
    }

	body = 10;
	direction= HOR;
	sign = PLUS;
	inc = false;

    int left = 0x00; int right = 0x02;
    int down = 0x09; int up = 0x09;
    for(int i=body-1;i>=0;i--)
    {
    	s[i].down = down; s[i].up = up;
    	s[i].left = left; s[i].right = right;

		st7586_write(ST_COMMAND, 0x2A);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].down);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].up);
		st7586_write(ST_COMMAND, 0x2B);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].left);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].right);
		st7586_write(ST_COMMAND, 0x2C);
		for(int i=0;i<3;i++)
			st7586_write(ST_DATA, 0xFF);
		left+=3; right+=3;
    }

    doritosX=99;doritosY=9;

	st7586_write(ST_COMMAND, 0x2A);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, doritosY);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, doritosY);
	st7586_write(ST_COMMAND, 0x2B);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, doritosX-3);
	st7586_write(ST_DATA, 0x00);
	st7586_write(ST_DATA, doritosX);
	st7586_write(ST_COMMAND, 0x2C);
	for(int i=0;i<3;i++)
		st7586_write(ST_DATA, 0xFF);

    st7586_write(ST_COMMAND, 0x29);

}

static inline void gameover()
{
	if(s[H].left < 0x00 || s[H].right > 0x9F || s[H].up > 0x2A || s[H].down < 0X00)
	{
		st7586_initialization();
		app_timer_stop(m_led_a_timer_id);
	}
	for(int i=1;i<body;i++)
	{
		if(s[H].left == s[i].left && s[H].up == s[i].up)
		{
			st7586_initialization();
			app_timer_stop(m_led_a_timer_id);
		}
	}
}

static inline void gamestart()
{
	for(int i=body-1;i>=0;i--)
	{
		if(i==body-1)
		{
			st7586_write(ST_COMMAND, 0x2A);
			st7586_write(ST_DATA, 0x00);
			st7586_write(ST_DATA, s[i].down);
			st7586_write(ST_DATA, 0x00);
			st7586_write(ST_DATA, s[i].up);
			st7586_write(ST_COMMAND, 0x2B);
			st7586_write(ST_DATA, 0x00);
			st7586_write(ST_DATA, s[i].left);
			st7586_write(ST_DATA, 0x00);
			st7586_write(ST_DATA, s[i].right);
			st7586_write(ST_COMMAND, 0x2C);
			for(int i=0;i<3;i++)
				st7586_write(ST_DATA, 0x00);
		}
		if(i==0)
		{
			if(direction==HOR)
			{
				if(sign)
				{
					s[i].right+=3;
					s[i].left+=3;
				}
				else
				{
					s[i].right-=3;
					s[i].left-=3;
				}
			}
			else
			{
				if(sign)
				{
					s[i].up+=1;
					s[i].down+=1;
				}
				else
				{
					s[i].up-=1;
					s[i].down-=1;
				}
			}
		}
		if(i!=0)
		{
			s[i].right = s[i-1].right;
			s[i].left = s[i-1].left;
			s[i].up = s[i-1].up;
			s[i].down = s[i-1].down;
		}

		st7586_write(ST_COMMAND, 0x2A);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].down);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].up);
		st7586_write(ST_COMMAND, 0x2B);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].left);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, s[i].right);
		st7586_write(ST_COMMAND, 0x2C);
		for(int i=0;i<3;i++)
			st7586_write(ST_DATA, 0xFF);
	}

	gameover();


	if((s[H].left==doritosX-3) && (s[H].up==doritosY))
	{
		doritos();

		st7586_write(ST_COMMAND, 0x2A);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, doritosY);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, doritosY);
		st7586_write(ST_COMMAND, 0x2B);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, doritosX-3);
		st7586_write(ST_DATA, 0x00);
		st7586_write(ST_DATA, doritosX);
		st7586_write(ST_COMMAND, 0x2C);
		for(int i=0;i<3;i++)
			st7586_write(ST_DATA, 0xFF);

		s[body].up = s[body-1].up;
		s[body].down = s[body-1].down;
		s[body].left = s[body-1].left;
		s[body].right = s[body-1].right;
		body++;
	}

}


bool main_context ( void )
{
    static const uint8_t ISR_NUMBER_THREAD_MODE = 0;
    uint8_t isr_number =__get_IPSR();
    if ((isr_number ) == ISR_NUMBER_THREAD_MODE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// Timeout handler for the repeated timer
static void timer_handler(void * p_context)
{
    if (main_context())
    {
        NRF_LOG_INFO("in main context...\r\n");
    }
    else
    {
    }
}

// Create timers
static void create_timers()
{
    uint32_t err_code;

    // Create timers
    err_code = app_timer_create(&m_led_a_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_handler);
    APP_ERROR_CHECK(err_code);
}


// Function for controlling LED's based on button presses.

void button_handler(nrf_drv_gpiote_pin_t pin)
{

    uint32_t err_code;
    int32_t TICKS = APP_TIMER_TICKS(100); // speed of recurrrence

    // Handle button presses.
    switch (pin)
    {
    case BUTTON_1_PIN:
        err_code = app_timer_start(m_led_a_timer_id, TICKS, NULL);
        if (err_code != NRF_SUCCESS) {
            NRF_LOG_INFO("*Error starting app timer*\r\n");
        }
        else {
            NRF_LOG_INFO("Started timer\r\n");
        }
        NRF_LOG_FLUSH();
        break;
    case BUTTON_2_PIN:
        err_code = app_timer_stop(m_led_a_timer_id);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("Stopped timer\r\n");
        NRF_LOG_FLUSH();
        break;
    case BUTTON_3_PIN:
    	if(!direction)
    	{
    		if(sign)
    			sign=!sign;
    		else
    			sign=!sign;
    	}
    	direction = !direction;
    	break;
    case BUTTON_4_PIN:
    	if(direction)
    	{
    		if(sign)
    			sign=!sign;
    		else
    			sign=!sign;
    	}
    	direction = !direction;
    	break;
    default:

        break;
    }

    // Light LED 2 if running in main context and turn it off if running in an interrupt context.
    // This has no practical use in a real application, but it is useful here in the tutorial.
    if (main_context())
    {
    }
    else
    {
    }
}

// Button handler function to be called by the scheduler.
void button_scheduler_event_handler(void *p_event_data, uint16_t event_size)
{
    // In this case, p_event_data is a pointer to a nrf_drv_gpiote_pin_t that represents
    // the pin number of the button pressed. The size is constant, so it is ignored.
    button_handler(*((nrf_drv_gpiote_pin_t*)p_event_data));
}

//Button event handler.
void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // The button_handler function could be implemented here directly, but is
    // extracted to a separate function as it makes it easier to demonstrate
    // the scheduler with less modifications to the code later in the tutorial.

    //button_handler(pin);

    app_sched_event_put(&pin, sizeof(pin), button_scheduler_event_handler);
}


// Function for configuring GPIO.
static void gpio_config()
{
    ret_code_t err_code;

    // Initialze driver.
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // Make a configuration for input pints. This is suitable for both pins in this example.
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    // Configure input pins for buttons, with separate event handlers for each button.
    err_code = nrf_drv_gpiote_in_init(BUTTON_1_PIN, &in_config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(BUTTON_2_PIN, &in_config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(BUTTON_3_PIN, &in_config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_gpiote_in_init(BUTTON_4_PIN, &in_config, gpiote_event_handler);
    APP_ERROR_CHECK(err_code);

    // Enable input pins for buttons.
    nrf_drv_gpiote_in_event_enable(BUTTON_1_PIN, true);
    nrf_drv_gpiote_in_event_enable(BUTTON_2_PIN, true);
    nrf_drv_gpiote_in_event_enable(BUTTON_3_PIN, true);
    nrf_drv_gpiote_in_event_enable(BUTTON_4_PIN, true);
}




/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */

static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
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
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &srdata);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED);
    	if(!direction)
    	{
    		if(sign)
    			sign=!sign;
    		else
    			sign=!sign;
    	}
    	direction = !direction;
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
    	bsp_board_led_on(LEDBUTTON_LED);
    	if(!direction)
    	{
    		if(sign)
    			sign=!sign;
    		else
    			sign=!sign;
    	}
    	direction = !direction;
        NRF_LOG_INFO("Received LED OFF!");
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t     err_code;
    ble_lbs_init_t init;

    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

#ifndef S140
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
#endif

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

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

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
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

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */


/**@brief Function for initializing the button handler module.
 */



static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for the Power Manager.
 */

static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    st7586_pinout_setup();
    st7586_initialization();

    gpio_config();
    // Initialize.
    leds_init();
    timers_init();
    log_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();


    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    // Start execution.
    NRF_LOG_INFO("Blinky example started.");
    advertising_start();

    create_timers();

    while (true)
    {
        sd_app_evt_wait();
        app_sched_execute();
    	if (NRF_LOG_PROCESS() == false)
    	{
    		power_manage();
    	}
    	gamestart();
         __WFI();
    }
}


/**
 * @}
 */
