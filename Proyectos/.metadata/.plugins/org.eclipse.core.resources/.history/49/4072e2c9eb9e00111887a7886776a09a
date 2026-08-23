#include "hc_sr04.h"

static volatile uint32_t t_ini = 0U;
static volatile uint32_t t_end = 0U;
static volatile uint32_t t_time = 0U;
static volatile uint16_t dist = 0U;
static volatile uint8_t flag_captured = 0U;
static volatile uint8_t measurement_done = 0U;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        if (flag_captured == 0U)
        {
            t_ini = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            flag_captured = 1U;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                TIM_CHANNEL_1,
                TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            t_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (t_end >= t_ini)
            {
                t_time = t_end - t_ini;
            }
            else
            {
                t_time = (65536UL - t_ini) + t_end;
            }

            /* TIM1 trabaja a 1 MHz: 1 tick = 1 us. */
            dist = (uint16_t)((t_time * 343UL) / 20000UL);
            measurement_done = 1U;
            flag_captured = 0U;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                TIM_CHANNEL_1,
                TIM_INPUTCHANNELPOLARITY_RISING);

            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        }
    }
}

void HCSR04_Init(void)
{
    HAL_GPIO_WritePin(Trigger_GPIO_Port,
                      Trigger_Pin,
                      GPIO_PIN_RESET);

    flag_captured = 0U;
    measurement_done = 0U;
    dist = 0U;

    __HAL_TIM_SET_CAPTUREPOLARITY(
        &htim1,
        TIM_CHANNEL_1,
        TIM_INPUTCHANNELPOLARITY_RISING);

    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
}

uint16_t HCSR04_Get_Distance(void)
{
    uint32_t timeout;

    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);

    flag_captured = 0U;
    measurement_done = 0U;
    dist = 0U;

    __HAL_TIM_SET_CAPTUREPOLARITY(
        &htim1,
        TIM_CHANNEL_1,
        TIM_INPUTCHANNELPOLARITY_RISING);

    /* TIM1 ya está corriendo a 1 MHz. Se usa para generar 10 us exactos. */
    __HAL_TIM_SET_COUNTER(&htim1, 0U);

    HAL_GPIO_WritePin(Trigger_GPIO_Port,
                      Trigger_Pin,
                      GPIO_PIN_SET);

    while (__HAL_TIM_GET_COUNTER(&htim1) < 10U)
    {
        /* Espera de 10 us medida por hardware. */
    }

    HAL_GPIO_WritePin(Trigger_GPIO_Port,
                      Trigger_Pin,
                      GPIO_PIN_RESET);

    /* La medición del ECHO comienza desde cero, después del TRIGGER. */
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

    timeout = HAL_GetTick();

    while (measurement_done == 0U)
    {
        if ((HAL_GetTick() - timeout) >= 30U)
        {
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);

            flag_captured = 0U;

            __HAL_TIM_SET_CAPTUREPOLARITY(
                &htim1,
                TIM_CHANNEL_1,
                TIM_INPUTCHANNELPOLARITY_RISING);

            return 0U;
        }
    }

    return dist;
}
