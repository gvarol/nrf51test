#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_adc.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "app_error.h"
#include "app_uart.h"

//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
#include <stdint.h>
#include <stdbool.h>

#define NRF_LOG_INFO printf

#define RANDOM_BUFF_SIZE 8

#define COMPARE_COUNTERTIME  (3UL)                                        /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1);

//Wraps after ~497 days.
static volatile uint32_t uptime;


#define ADC_BUFFER_SIZE 10                                /**< Size of buffer for ADC samples.  */
static nrf_adc_value_t       adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static nrf_drv_adc_channel_t m_channel_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); /**< Channel instance. Default configuration used. */


#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

//ECB-AES128 test vectors
static const uint8_t plain_text[] = {
    0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
    0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
    0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11,0xe5,0xfb,0xc1,0x19,0x1a,0x0a,0x52,0xef,
    0xf6,0x9f,0x24,0x45,0xdf,0x4f,0x9b,0x17,0xad,0x2b,0x41,0x7b,0xe6,0x6c,0x37,0x10
};

static const uint8_t aes_key[] = {
    0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
    0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c
};

static const uint8_t cipher_text[] = {
    0x3a,0xd7,0x7b,0xb4,0x0d,0x7a,0x36,0x60,0xa8,0x9e,0xca,0xf3,0x24,0x66,0xef,0x97,
    0xf5,0xd3,0xd5,0x85,0x03,0xb9,0x69,0x9d,0xe7,0x85,0x89,0x5a,0x96,0xfd,0xba,0xaf,
    0x43,0xb1,0xcd,0x7f,0x59,0x8e,0xce,0x23,0x88,0x1b,0x00,0xe3,0xed,0x03,0x06,0x88,
    0x7b,0x0c,0x78,0x5e,0x27,0xe8,0xad,0x3f,0x82,0x23,0x20,0x71,0x04,0x72,0x5d,0xd4
};

//Array size is defined to avoid '\0'
static const char plain_text_readable[64] =
"NRF51 TEST BLOCK"
"NRF51 BLOCK 0001"
"NRF51 BLOCK 0002"
"NRF51 BLOCK 0003";

static const char aes_key_readable[] = "UNBREAKABLE KEY!";

static const uint8_t cipher_text_readable[] = {
    0x98,0x34,0x44,0x67,0xf5,0xa3,0x9b,0xe7,0xc6,0xbb,0x19,0x20,0xcd,0x2e,0x05,0x13,
    0xc1,0xef,0x52,0xd1,0x6b,0x91,0xfa,0x0d,0xd3,0x58,0xba,0x7b,0x15,0x28,0xb8,0xb6,
    0xe0,0xfe,0xaf,0xac,0x16,0x85,0xed,0xcf,0xc3,0x79,0x5d,0xfd,0x44,0xed,0x68,0x65,
    0x53,0xa6,0xb3,0x4a,0xcd,0x85,0xbf,0x73,0x1d,0x14,0xd4,0x0e,0xc1,0xbb,0xe2,0x84,
};

static uint8_t  ecb_data[48];   ///< ECB data structure for RNG peripheral to access.
static uint8_t* ecb_key;        ///< Key:        Starts at ecb_data
static uint8_t* ecb_cleartext;  ///< Cleartext:  Starts at ecb_data + 16 bytes.
static uint8_t* ecb_ciphertext; ///< Ciphertext: Starts at ecb_data + 32 bytes.

/*
 * These pins are used for a very simple
 * demonstration of GPIO communication
 * between devices.
 */
static const uint8_t comm_pins[] = {
    3,4,5,12,13,14,15,16
};

const static char SZPASS[] = "PASSED";
const static char SZFAIL[] = "FAILED";

static bool toggle_enable;
static bool compare_pass;

#define PIN_CTS (1)
#define PIN_RDY (2)

#define ECB_BLOCKSIZE (16)

#define WFI_COND(cond) do{__WFI();} while((!cond))

static bool nrf_ecb_init(void)
{
  ecb_key = ecb_data;
  ecb_cleartext  = ecb_data + 16;
  ecb_ciphertext = ecb_data + 32;

  NRF_ECB->ECBDATAPTR = (uint32_t)ecb_data;
  return true;
}

static bool nrf_ecb_crypt(uint8_t * dest_buf, const uint8_t * src_buf)
{
   uint32_t counter = 0x1000000;
   if (src_buf != ecb_cleartext)
   {
     memcpy(ecb_cleartext,src_buf,16);
   }
   NRF_ECB->EVENTS_ENDECB = 0;
   NRF_ECB->TASKS_STARTECB = 1;
   while (NRF_ECB->EVENTS_ENDECB == 0)
   {
    counter--;
    if (counter == 0)
    {
      printf("ecb_crypt error\n");
      return false;
    }
   }
   NRF_ECB->EVENTS_ENDECB = 0;
   if (dest_buf != ecb_ciphertext)
   {
     memcpy(dest_buf,ecb_ciphertext,16);
   }
   return true;
}

static void nrf_ecb_set_key(const uint8_t * key)
{
  memcpy(ecb_key,key,16);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


/**
 * @brief ADC interrupt handler.
 */
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
        for (i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_INFO("adc sample: %d\n", p_event->data.done.p_buffer[i]);
        }
    }
}

/**
 * @brief ADC initialization.
 */
static void adc_config(void)
{
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;

    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);

    nrf_drv_adc_channel_enable(&m_channel_config);
}

static uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
    uint32_t err_code;
    uint8_t  available;

    nrf_drv_rng_bytes_available(&available);
    uint8_t length = MIN(size, available);

    err_code = nrf_drv_rng_rand(p_buff, length);
    APP_ERROR_CHECK(err_code);

    return length;
}

//Simple delay function that uses RTC instead of TIMER.
static void delay_tick(uint32_t tick)
{
    uint32_t target = uptime + tick;
    while(target > uptime)
    {
        __WFI();
    }
}

static void rtc1_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
        uptime++;
}

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (!toggle_enable)
        return;

    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        nrf_gpio_pin_toggle(BSP_LED_1);
        compare_pass = true;
        //nrf_drv_rtc_counter_clear(&rtc);
        //nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8,true);
    }
    else if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        //Soft prescaler.
        static int soft_ps;
        if (!(soft_ps & 0x3))
        {
            printf("pin toggle\n");
            nrf_gpio_pin_toggle(BSP_LED_0);
        }
        soft_ps++;
    }
}

/** @brief Function configuring gpio for pin toggling.
 */
static void leds_config(void)
{
    bsp_board_leds_init();
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, COMPARE_COUNTERTIME * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

static void rtc1_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 327; //~100 Hz
    err_code = nrf_drv_rtc_init(&rtc1, &config, rtc1_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc1,true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc1);
}

bool ecb_test(const uint8_t* key, const uint8_t * plain, const uint8_t * cipher, int size)
{
    int i;
    nrf_ecb_set_key(key);
    for(i = 0; i < size; i += ECB_BLOCKSIZE)
    {
        nrf_ecb_crypt(ecb_ciphertext, &plain[i]);
        if(memcmp(&cipher[i], ecb_ciphertext, ECB_BLOCKSIZE)){
            printf("ECB error, block: %u\n", i/ECB_BLOCKSIZE);
            return false;
        }
    }
    return true;
}

static void gpiote_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    printf("gpiote event, action:%u\n",action);
}

void gpio_comm_init(void)
{
    int i;
    for(i = 0; i < sizeof(comm_pins); i++){
        nrf_gpio_cfg_output(comm_pins[i]);
    }

    nrf_gpio_cfg_output(PIN_RDY);
    nrf_gpio_cfg_input(PIN_CTS, NRF_GPIO_PIN_NOPULL);
}

static void gpiote_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_gpiote_init();
    if(NRF_SUCCESS != err_code)
        goto gpiote_fail;

    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &config, gpiote_event_handler);
    if(NRF_SUCCESS != err_code)
    {
gpiote_fail:
        printf("gpiote_init failed\n");
        WFI_COND(false);
    }

    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint8_t p_buff[RANDOM_BUFF_SIZE];
    uint8_t length;
    uint32_t err_code;
    bool ecb_passed = true;
    int i;
    uint8_t byte_data;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    printf("system start\n");

//    NRF_GPIOTE->CONFIG[0] = 0;

    toggle_enable = true;

    gpio_comm_init();

    nrf_ecb_init();

    leds_config();

    lfclk_config();

    rtc_config();

    rtc1_config(); //this rtc is used for delay function.

    adc_config();

    gpiote_init();

    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);

    printf("wait 5 secs\n");
    delay_tick(500);

    WFI_COND(compare_pass);

    toggle_enable = false;
    printf("RTC compare test PASSED\n");

//    while(1) delay_tick(100);
    length = random_vector_generate(p_buff, RANDOM_BUFF_SIZE);

    printf("Random vector size: %u\n", length);

    for(i = 0; i < RANDOM_BUFF_SIZE; i++)
    {
        printf("random: %u\n", p_buff[i]);
    }


    APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE));
    for (i = 0; i < ADC_BUFFER_SIZE; i++)
    {
        // manually trigger ADC conversion
        nrf_drv_adc_sample();
        __WFI();
        delay_tick(10);
    }

    printf("adc sampling done\n");

    ecb_passed = ecb_test(aes_key, plain_text, cipher_text, sizeof(plain_text));

    printf("ECB test with test vector %s\n",
            ecb_passed?"PASSED":"FAILED");

    if(ecb_passed){
        ecb_passed = ecb_test((uint8_t*)aes_key_readable,
                              (uint8_t*)plain_text_readable,
                              cipher_text_readable,
                              sizeof(plain_text_readable));
    }

    printf("ECB test with human readable text %s\n", 
            ecb_passed?"PASSED":"FAILED");

    printf("wait CTS HIGH\n");

    while(!nrf_gpio_pin_read(PIN_CTS)){
        __WFI();
    }

    //Send first item of random array.
    byte_data = p_buff[0];
    printf("Send byte on GPIO: %u\n", byte_data);
    for(i = 0; i < 8; i++, byte_data >>= 1){
        nrf_gpio_pin_write(comm_pins[i], byte_data & 0x1);
        delay_tick(10);
    }

    //Set ready pin so other side can read
    //and transmit data.
    nrf_gpio_pin_set(PIN_RDY);

    delay_tick(100);

    //Put pins in input mode
    for(i = 0; i < sizeof(comm_pins); i++){
        nrf_gpio_cfg_input(comm_pins[i], NRF_GPIO_PIN_NOPULL);
    }

    printf("Wait CTS HIGH\n");
    while(!nrf_gpio_pin_read(PIN_CTS)){
        __WFI();
    }

    byte_data = 0;
    for(i = 7; i >= 0; i--){
        byte_data |= nrf_gpio_pin_read(comm_pins[i]) << i;
    }

    printf("data sent: %u\n"
           "received : %u\n",
           p_buff[0], byte_data);

    printf("GPIO test: %s\n", p_buff[0] == byte_data ? SZPASS:SZFAIL);

    while (true)
    {
        //Device will still keep toggling pin 21
        //__SEV();
        //__WFE();
        __WFI();
    }
}


/**  @} */
