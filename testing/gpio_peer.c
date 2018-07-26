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

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

const nrf_drv_rtc_t rtc1 = NRF_DRV_RTC_INSTANCE(1);

//Wraps after ~497 days.
static volatile uint32_t uptime;

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

/*
 * These pins are used for a very simple
 * demonstration of GPIO communication
 * between devices.
 */
static const uint8_t comm_pins[] = {
    3,4,5,12,13,14,15,16
};

#define WFI_COND(cond) do{__WFI();} while((!cond))

#define PIN_CTS (1)
#define PIN_RDY (2)

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

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
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

void gpio_comm_init(void)
{
    int i;
    for(i = 0; i < sizeof(comm_pins); i++){
        nrf_gpio_cfg_input(comm_pins[i], NRF_GPIO_PIN_NOPULL);
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

    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    err_code = nrf_drv_gpiote_out_init(LED_4, &config);
    if(NRF_SUCCESS != err_code)
    {
gpiote_fail:
        printf("gpiote_init failed\n");
        WFI_COND(false);
    }

    nrf_drv_gpiote_out_task_enable(LED_4);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
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

    rtc1_config(); //this rtc is used for delay function.

    printf("system start\n");

    gpio_comm_init();

    gpiote_init();

    delay_tick(10);

    lfclk_config();

    printf("Set RDY\n");

    nrf_gpio_pin_set(PIN_RDY);

    printf("Wait for CTS HIGH\n");

    while(!nrf_gpio_pin_read(PIN_CTS)){
        __WFI();
    }

    nrf_gpio_pin_clear(PIN_RDY);

    printf("Read GPIO data\n");

    byte_data = 0;
    for(i = 7; i >= 0; i--){
        byte_data |= nrf_gpio_pin_read(comm_pins[i]) << i;
    }

    printf("received data: %u\n", byte_data);

    delay_tick(100);
    //Put pins in output mode
    for(i = 0; i < sizeof(comm_pins); i++){
        nrf_gpio_cfg_output(comm_pins[i]);
    }

    printf("Sending back %u\n", byte_data);

    for(i = 0; i < 8; i++, byte_data >>= 1){
        nrf_gpio_pin_write(comm_pins[i], byte_data & 0x1);
        delay_tick(10);
    }

    delay_tick(200);

    nrf_gpio_pin_set(PIN_RDY);

    delay_tick(300);
    printf("set LED_4 through GPIOTE task\n");
    nrf_drv_gpiote_out_task_trigger(LED_4);
    //nrf_gpio_cfg_output(LED_4);
    //nrf_gpio_pin_set(LED_4);
    while (true)
    {
        //Device will still keep toggling pin 21
        //__SEV();
        //__WFE();
        __WFI();
    }
}


/**  @} */
