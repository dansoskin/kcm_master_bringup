/*
 * functions.c
 *
 *  Application logic (setup / loop) kept out of the CubeMX-generated main.c.
 *  Initialises the UART DMA idle-interrupt helper on USART6.
 */

#include "functions.h"
#include "main.h"
#include "decode_packet.h"
#include "globals.h"

#include "can.h"
#include "odrive.h"
#include "roulette_sm.h"

/* USART6 / FDCAN1 handles are defined in main.c by CubeMX. */
extern UART_HandleTypeDef huart6;
extern FDCAN_HandleTypeDef hfdcan1;


/* -------------------------------------------------------------------------- */
/* UART buffers                                                               */
/* -------------------------------------------------------------------------- */

#define UART_RING_BUFFER_SIZE 200U
#define UART_DMA_BUFFER_SIZE  100U
static uint8_t uart_ring_buffer[UART_RING_BUFFER_SIZE];
__attribute__((section(".dma_buffer")))
static uint8_t uart_dma_buffer[UART_DMA_BUFFER_SIZE];


extern TIM_HandleTypeDef htim23;

myCAN_t  myCan;

void uart_listener(void);
void can_odrive_listener(void);

/* odrive_send_fn: route library TX through canbus_wrapper on FDCAN1.
 * can_send() takes a non-const payload, so copy; data is NULL for RTR. */
static bool odrv_can_send(uint32_t can_id, const uint8_t *data, uint8_t len, bool rtr)
{
    uint8_t buf[8] = {0};
    if (data != NULL && len > 0U)
        memcpy(buf, data, len);
    return can_send(&myCan, buf, len, can_id, rtr ? 1U : 0U) == HAL_OK;
}

/* Library log sink (heartbeat diffs, command traces, fw warnings). */
static void odrv_log(const char *message)
{
    send_uart(&myUart, "%s\n", message);
}



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

    /* The stepper is the roulette's master azimuth axis, so it works in degrees
     * of azimuth: 3200 steps/rev over 360 deg means one motor revolution is
     * exactly one plate orbit, at 0.1125 deg of azimuth per step.
     * NOTE: the '@' commands therefore speak degrees now, not revolutions.
     * Acceleration is set to reach 90 deg/s after one full orbit:
     *     a = v^2 / 2d = 90^2 / (2 * 360) = 11.25 deg/s^2   (an 8 s spin-up) */
    FlexyStepper_setConversion(&stepper, 3200.0f / 360.0f); /* steps per degree */
    FlexyStepper_setAcceleration(&stepper, 11.25f);         /* deg per second^2 */
    FlexyStepper_setSpeed(&stepper, 90.0f);                 /* deg per second   */

    /* ---------------------- ODrive on FDCAN1 ----------------------------- */

    if (can_setup(&myCan, &hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    const float conversion = 32.0/(360*2);  //the gear is 1:32 and i want degrees on the output the other /2 is the pulley ratio
    const float speed = 45;      //2 rotations per second on the output
    const float acceleration = 100;

    /* can_setup must run first: odrive_init already sends a version request. */
    odrive_init(&odrv0, odrv_can_send, 1, /*conv=*/conversion, /*invert=*/false);
    odrive_set_logger(&odrv0, "odrv0", odrv_log);
    odrive_enable_logging(&odrv0, 1);
    // odrive_set_limits(odrive_t *od, float vel_limit, float current_limit);
    odrive_set_traj_vel_limit(&odrv0, speed);
    odrive_set_traj_accel_limits(&odrv0, acceleration, acceleration);
    odrive_set_controller_mode(&odrv0, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_TRAP_TRAJ);
    
    odrive_init(&odrv1, odrv_can_send, 2, /*conv=*/conversion, /*invert=*/false);
    odrive_set_logger(&odrv1, "odrv1", odrv_log);
    odrive_enable_logging(&odrv1, 1);
    // odrive_set_limits(odrive_t *od, float vel_limit, float current_limit);
    odrive_set_traj_vel_limit(&odrv1, speed);
    odrive_set_traj_accel_limits(&odrv1, acceleration, acceleration);
    odrive_set_controller_mode(&odrv1, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_TRAP_TRAJ);

    x_axis.acceleration = acceleration;
    x_axis.max_speed = speed;

    y_axis.acceleration = acceleration;
    y_axis.max_speed = speed;
}



void loop(void)
{
    uart_listener();
    can_odrive_listener();
    FlexyStepper_loop(&stepper);
    roulette_sm_loop();


    static uint32_t loop_timer = 0;
    if(HAL_GetTick() - loop_timer > 500)
    {
        loop_timer = HAL_GetTick();
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
        HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);

        // send_uart(&myUart, "Loop tick\n");
    }
}


/* Drain the CAN RX ring and feed data frames to the ODrive decoder.
 * odrive_on_can_rx() ignores frames whose node id isn't its own, and it must
 * only see data frames -- skip remote (RTR) frames here. */
void can_odrive_listener(void)
{
    while (can_data_in_buffer(&myCan) >= sizeof(can_rx_packet))
    {
        can_rx_packet rx;
        if (can_get_from_rbbuffer(&myCan, &rx) != sizeof(rx))
            break;
        if (rx._rx_header.RxFrameType == FDCAN_REMOTE_FRAME)
            continue;

        odrive_on_can_rx(&odrv0,  rx._rx_header.Identifier, rx._rx_data, (uint8_t)rx._rx_header.DataLength);
        odrive_on_can_rx(&odrv1,  rx._rx_header.Identifier, rx._rx_data, (uint8_t)rx._rx_header.DataLength);
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
