#include "hc_sr04.h"

static volatile uint32_t t_ini = 0;
static volatile uint32_t t_end = 0;
static volatile uint32_t t_time = 0;
static volatile uint16_t dist = 0;
static volatile uint8_t flag_captured = 0;
static volatile uint8_t measurement_done = 0;

//cuando llega un flanco de eco se ejecuta esta funcion (de subida y de bajada)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)) //verificamos que sea del timer y canal adecuado
    {
        if (flag_captured == 0) //primer flanco
        {
            t_ini = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            flag_captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }

        else //segundo flanco (de bajada que indica el final del echo)
        {
            t_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (t_end >= t_ini)
            {
                t_time = t_end - t_ini;
            }
            else //si se desborda el contador
            {
                t_time = (65536UL - t_ini) + t_end;
            }

            /* TIM1 trabaja a 1 MHz: 1 tick = 1 us. */
            dist = (uint16_t)((t_time * 343UL) / 20000UL); //distancia en cm = tiempo en µs × 0,0343 / 2

            measurement_done = 1U;
            flag_captured = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        }
    }
}

void HCSR04_Init(void)
{
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET); //triggr en 0 para no recibir un pulso accidentalmente

    //reinicia las variables
    flag_captured = 0;
    measurement_done = 0;
    dist = 0;

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); //configura el canal para detectar flanco de subida

    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK) //inicia el input capture
    {
        Error_Handler();
    }

    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1); //deshabilita temporalmente su interrupcion, se acttivara cuando se realice una medicion
}

uint16_t HCSR04_Get_Distance(void)
{
    uint32_t timeout;

    //reiniciamos para tener una referencia
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
    flag_captured = 0;
    measurement_done = 0;
    dist = 0;

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    /* TIM1 ya está corriendo a 1 MHz. Se usa para generar 10 us exactos. */
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    //generamos UN PULSO ALTO de 10 us
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10) //1 tick = 1 µs; 10 ticks = 10 µs
    {
        /* Espera de 10 us medida por hardware. */
    }
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

    /* La medición del ECHO comienza desde cero (reiniciamos variables), después del TRIGGER. */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

    timeout = HAL_GetTick();
    while (measurement_done == 0)
    {
        if ((HAL_GetTick() - timeout) >= 30U)
        {
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
            flag_captured = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

            return 0; //si no llega el eco, lectura invalida
        }
    }

    return dist; //calculada en la interrupcion
}
