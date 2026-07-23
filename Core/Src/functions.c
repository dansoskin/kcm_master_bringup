/*
 * functions.c
 *
 *  Application logic (setup / loop) kept out of the CubeMX-generated main.c.
 *  Initialises the UART DMA idle-interrupt helper on USART6.
 */

#include "functions.h"
#include "main.h"
#include "uart.h"

#include "decode_packet.h"

#include "cFlexyStepper.h"

/* USART6 handle is defined in main.c by CubeMX. */
extern UART_HandleTypeDef huart6;


/* -------------------------------------------------------------------------- */
/* UART buffers                                                               */
/* -------------------------------------------------------------------------- */

#define UART_RING_BUFFER_SIZE 200U
#define UART_DMA_BUFFER_SIZE  100U

myUART_t myUart;
static uint8_t uart_ring_buffer[UART_RING_BUFFER_SIZE];

__attribute__((section(".dma_buffer")))
static uint8_t uart_dma_buffer[UART_DMA_BUFFER_SIZE];


extern TIM_HandleTypeDef htim23;

FlexyStepper stepper;

void uart_listener(void);



void setup(void)
{
    if (!setup_uart(&myUart,
                    &huart6,
                    uart_ring_buffer,
                    sizeof(uart_ring_buffer),
                    uart_dma_buffer,
                    sizeof(uart_dma_buffer)))
    {
        Error_Handler();
    }

    /* ---------------------- Stepper motor (cFlexyStepper) ---------------- */

    FlexyStepper_attach_timer_for_micros(&htim23);
    FlexyStepper_attach_logger(&huart6);

    FlexyStepper_Init(&stepper, "Stepper1");
    FlexyStepper_connectToPins(&stepper, GPIO1_GPIO_Port, GPIO1_Pin,
                                         GPIO2_GPIO_Port, GPIO2_Pin);
    FlexyStepper_connectEnablePin(&stepper, GPIO3_GPIO_Port, GPIO3_Pin, true);

    FlexyStepper_setConversion(&stepper, 3200.0f);  /* steps per unit          */
    FlexyStepper_setAcceleration(&stepper, 0.1);  /* units per second^2      */
    FlexyStepper_setSpeed(&stepper, 1);          /* units per second        */

}



void loop(void)
{
    uart_listener();
    FlexyStepper_loop(&stepper);

    static uint32_t loop_timer = 0;
    if(HAL_GetTick() - loop_timer > 500)
    {
        loop_timer = HAL_GetTick();
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
        // send_uart(&myUart, "Loop tick\n");
    }
}


void uart_listener(void)
{
    uint32_t bytes = bytes_in_buffer(&myUart);
    if (bytes > 0U)
    {
        uint8_t input[100] = {0};
        int len = read_buffer_until(&myUart, '\n', input, sizeof(input));

        if (len > 0)
        {
            decode_uart((char*)input, NULL, len);
            // send_uart(&myUart, "UART<< %s\n", input);
        }
    }
}
