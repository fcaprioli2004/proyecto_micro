#include "hc_sr04.h"

static volatile uint32_t t_ini = 0;
static volatile uint32_t t_end = 0;
static volatile uint32_t t_time = 0;
static volatile uint16_t dist = 0;
static volatile uint8_t flag_captured = 0;
static volatile uint8_t measurement_done = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))  //verificamos de que la interrupcion sea del adecuado
    {
        if (flag_captured == 0)  //primer flanco
        {
            t_ini = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            flag_captured = 1;

            //cambiamos de sentido para ahora detectar flanco de bajada
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            t_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            if (t_end >= t_ini)
            {
                t_time = t_end - t_ini;
            }
            else //si el timer se desborda
            {
                t_time = (65536UL - t_ini) + t_end;
            }

            /* TIM1 trabaja a 1 MHz: 1 tick = 1 us. */
            dist = (uint16_t)((t_time * 343UL) / 20000UL);  //abajo divisimos por 2, porque es ida y vuelta el camino

            measurement_done = 1;

            //volvemos a preparar para el proximo flanco
            flag_captured = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);
        }
    }
}

void HCSR04_Init(void)
{
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);  //asegura inicialmente el Trigger = 0

    //variables auxiliares
    flag_captured = 0;
    measurement_done = 0;
    dist = 0;

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); //prepara TIM1 para recibir un flanco ascendente del ECHO

    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);  //apagamos la interrrupción y despues la vamos a prender en el trigger
}

uint16_t HCSR04_Get_Distance(void)
{
    uint32_t timeout;

    //reinicia la medicion
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
    flag_captured = 0;
    measurement_done = 0;
    dist = 0;
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);


    //Utilizamos el contador del timer que esta a 1MHz para generar el pulso de 10us
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10)
    {
        //esperamos 10us por hardware
    }
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);

    //una vez generado el trigger ahora si podemos activar el timer para esperar el ECHO
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

    timeout = HAL_GetTick();

    while (measurement_done == 0)
    {
        if ((HAL_GetTick() - timeout) >= 40) //timeout para que no quede atrapado
        {
            __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
            __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);

            flag_captured = 0;

            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            return 0;
        }
    }

    return dist;
}
