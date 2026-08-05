/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rs485.h"
#include "protocolo_rs485.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 128

#define E_desactivado 0
#define E_configurando 1
#define E_activado 2
#define E_error 3

//estados para cuando esta en E_activado
#define C_andando 0
#define C_pesando 1
#define C_detenida 2
#define C_esperando_reinicio 3
#define C_acelerando 4
#define C_desacelerando 5
#define C_esperando_maestro 6

#define PWM_C_andando 990
#define PWM_C_detenido 250

#define PULSE_MIN       550
#define PULSE_MAX        2450
#define PULSE_REPOSO    1500
#define PULSE_ABIERTO   800

#define PESO_DETECCION  30000
#define PESO_LIBRE      20000

#define PESO_MIN_1  100000
#define PESO_MAX_1  150000
#define PESO_MIN_2  150001
#define PESO_MAX_2  300000
#define PESO_MIN_3  300001
#define PESO_MAX_3  400000

#define SENSOR_ANTIRREBOTE_MS 1000U
#define FIFO_TAMANO 10
typedef struct
{
    uint8_t datos[FIFO_TAMANO];
    uint8_t entrada;
    uint8_t salida;
    uint8_t cantidad;
} FIFO;


#define RS485_RESPUESTA_DELAY_MS       5U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t Estado;
uint8_t sub_Estado;

uint8_t rx_byte;
volatile uint8_t recibiendo = 0;
volatile uint8_t comando_listo = 0;
volatile uint8_t indice = 0;
char buffer_rx[BUFFER_SIZE];
char buffer_tx[BUFFER_SIZE];

int32_t weight = 0;
uint8_t configuracion_lista=0;

uint16_t valor_pwm_actual = 0;
uint32_t tiempo_fin_pesaje = 0;
uint16_t rampa_pwm_objetivo = 0;
uint16_t rampa_paso = 0;
uint32_t rampa_retardo_ms = 0;
uint32_t rampa_ultimo_tick = 0;
uint8_t sub_Estado_siguiente = 0;
int32_t pesos_estabilidad[10];
uint8_t cantidad_pesajes = 0;
uint32_t objetos_ok = 0;
uint32_t objetos_descarte = 0;
uint32_t objetos_control = 0;

FIFO cola_servo1 = {0};
FIFO cola_servo2 = {0};
FIFO cola_servo3 = {0};

volatile uint8_t eventos_sensor1 = 0;
volatile uint8_t eventos_sensor2 = 0;
volatile uint8_t eventos_sensor3 = 0;
uint32_t ultimo_sensor1 = 0;
uint32_t ultimo_sensor2 = 0;
uint32_t ultimo_sensor3 = 0;

static uint8_t aviso_esperando_pendiente = 0U;
static uint16_t aviso_esperando_secuencia = 0U;

static uint8_t aviso_peso_pendiente = 0U;
static uint16_t aviso_peso_decigramos = 0U;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//void interpretar_comando(void);
//void enviar_estado_uart(void);
void Set_DutyCycle_DC_PWM(uint16_t valor_pwm);
void Servo_Angle(uint8_t servo,uint16_t ang);
void desactivar(void);
uint8_t Verificar_Peso_Por_Pasos(int32_t *peso_promedio);
uint8_t FIFO_Agregar(FIFO *cola, uint8_t valor);
uint8_t FIFO_Sacar(FIFO *cola, uint8_t *valor);
uint8_t Obtener_Destino(int32_t peso);
void Procesar_Sensor1(void);
void Procesar_Sensor2(void);
void Procesar_Sensor3(void);

static void procesar_rs485_cinta(void);
static void uart_enviar_texto(const char *texto);

static HAL_StatusTypeDef cinta_enviar_rs485(uint8_t origen, uint8_t comando, uint8_t param_h, uint8_t param_l);
static void cinta_responder_resultado(uint8_t origen, uint8_t respuesta, uint8_t comando_solicitado, uint8_t codigo);

static void cinta_generar_aviso_esperando_maestro(void);
static void cinta_generar_aviso_peso(int32_t peso_mg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  //MX_IWDG_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //sprintf(buffer_tx, "Iniciando...\r\n");
  //HAL_UART_Transmit(&huart3, (uint8_t*)buffer_tx, strlen(buffer_tx), HAL_MAX_DELAY);


  if (RS485_Init(&huart1, RS485_DE_GPIO_Port, RS485_DE_Pin, ID_CINTA) != HAL_OK)
  {
      Error_Handler();
  }

  //Estado inicial
  HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, Led_OFF_Pin, GPIO_PIN_SET);
  Estado = E_desactivado;

  HX711_Init();

  //apagar motor de la cinta
  HAL_GPIO_WritePin(GPIOB, DC_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DC_IN2_Pin, GPIO_PIN_RESET);
  Set_DutyCycle_DC_PWM(0);

  //inicializar los servos
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PULSE_REPOSO);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PULSE_REPOSO);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PULSE_REPOSO);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  procesar_rs485_cinta();

	  //para la uart local, se podria activar para la verificacion local
	  /*if (comando_listo == 1) {
		  comando_listo = 0;
		  interpretar_comando();
	  }*/
	  switch(Estado){
	  	  case E_desactivado:
	  		  break;
		  case E_configurando:
			  break;
		  case E_activado:
			      if (eventos_sensor1 == 1){
			          eventos_sensor1 = 0;
			          Procesar_Sensor1();
			      }
			      if (eventos_sensor2 == 1){
			          eventos_sensor2 = 0;
			          Procesar_Sensor2();
			      }
			      if (eventos_sensor3 == 1){
			          eventos_sensor3 = 0;
			          Procesar_Sensor3();
			      }
			  switch(sub_Estado){
			  	  case C_detenida:
			  		  //espera a pasar a C_andando
			  		  break;
			  	case C_andando:
			  	{
			  	    HAL_StatusTypeDef estado_hx711;
			  	    estado_hx711 = HX711_WeighNonBlocking(&weight);

			  	    if (estado_hx711 == HAL_TIMEOUT ||estado_hx711 == HAL_ERROR) //si hay un error
			  	    {
			  	        desactivar();
			  	        Estado = E_error;

			  	        //aviso por la uart local
			  	        snprintf(buffer_tx,sizeof(buffer_tx),"ERROR: HX711\r\n");
			  	        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);
			  	        break;
			  	    }
			  	    if (estado_hx711 == HAL_BUSY) //todavia no tiene disponible una lectura nueva
			  	    {
			  	        break;
			  	    }
			  	    if (weight >= PESO_DETECCION) //cuando el peso supera el umbral de deteccion
			  	    {
			  	        cantidad_pesajes = 0;

			  	        rampa_pwm_objetivo = PWM_C_detenido;
			  	        rampa_paso = 10;
			  	        rampa_retardo_ms = 10;
			  	        rampa_ultimo_tick = HAL_GetTick();

			  	        sub_Estado_siguiente = C_esperando_maestro;
			  	        sub_Estado = C_desacelerando;
			  	    }

			  	    break;
			  	}
	 		 	 case C_pesando:
	 		 		switch (Verificar_Peso_Por_Pasos(&weight)){
	 		 		    case 1: //peso estable y terminado
	 		 		    {
	 		 		    	uint8_t destino = Obtener_Destino(weight);

	 		 		    	if (destino != 0)
	 		 		    	{
	 		 		    	    if (FIFO_Agregar(&cola_servo1, destino) == 0)
	 		 		    	    {
	 		 		    	        desactivar();
	 		 		    	        Estado = E_error;

	 		 		    	        //aviso UART LOCAL
	 		 		    	        snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:COLA_LLENA\r\n");
	 		 		    	        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);
	 		 		    	        break;
	 		 		    	    }
	 		 		    	}

	 		 		    	//Aviso uart local
	 		 		        snprintf(buffer_tx,sizeof(buffer_tx),"Peso: %ld\r\n",(long)weight);
	 		 		        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);

                            //genera el evento para el maestro
                            cinta_generar_aviso_peso(weight);

	 		 		        rampa_pwm_objetivo = PWM_C_andando;
	 		 		        rampa_paso = 10; //paso para llegar al objetivo
	 		 		        rampa_retardo_ms = 10; //tiempo por paso
	 		 		        rampa_ultimo_tick = HAL_GetTick();

	 		 		        sub_Estado_siguiente = C_esperando_reinicio;
	 		 		        sub_Estado = C_acelerando;

	 		 		        tiempo_fin_pesaje = HAL_GetTick();
	 		 		        break;
	 		 		    }
	 		 		    case 2:
	 		 		        //Peso inestable
	 		 		        break;
	 		 		    case 3:
	 		 		        //Todavía faltan pesajes
	 		 		        break;
	 		 		    case 0:
	 		 		    default:
	 		 		        desactivar();
	 		 		        Estado = E_error;

	 		 		        //Aviso UART local
		 		 	        snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:\r\n");
		 		 	        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
	 		 		        break;
	 		 		}
	 		 		 break;
	 		 		case C_esperando_reinicio:
	 		 		{
	 		 		    if ((HAL_GetTick() - tiempo_fin_pesaje) >= 1500) //espera 1.5s
	 		 		    {
	 		 		        HAL_StatusTypeDef estado_hx711;
	 		 		        estado_hx711 = HX711_WeighNonBlocking(&weight);

	 		 		        if (estado_hx711 == HAL_TIMEOUT ||estado_hx711 == HAL_ERROR)
	 		 		        {
	 		 		            desactivar();
	 		 		            Estado = E_error;

	 		 		            //Aviso UART local
	 		 		            snprintf(buffer_tx,sizeof(buffer_tx),"ERROR: HX711\r\n");
	 		 		            HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);
	 		 		        }
	 		 		        else if ((estado_hx711 == HAL_OK) &&(weight < PESO_LIBRE)) //si el peso cae por debajo de 20gr toma como que la botella abandono el luga
	 		 		        {
	 		 		            sub_Estado = C_andando;
	 		 		        }
	 		 		    }
	 		 		    break;
	 		 		}

	 		 	//los dos estados usan el mismo codigo (acelerando o desacelerando)
				case C_desacelerando:
				case C_acelerando:
					if (HAL_GetTick() - rampa_ultimo_tick >= rampa_retardo_ms)
					{
						if (valor_pwm_actual < rampa_pwm_objetivo) //aceleramos
						{
							if ((rampa_pwm_objetivo - valor_pwm_actual) <= rampa_paso) //si estamos a menos del paso del objetivo, establecemos la velocidad objetivo
							{
								Set_DutyCycle_DC_PWM(rampa_pwm_objetivo);
							} else
							{
								Set_DutyCycle_DC_PWM(valor_pwm_actual + rampa_paso); //aumentamos de a 10
							}
						}
						else if (valor_pwm_actual > rampa_pwm_objetivo) //frena
						{
							if ((valor_pwm_actual - rampa_pwm_objetivo) <= rampa_paso)
							{
								Set_DutyCycle_DC_PWM(rampa_pwm_objetivo);
							} else
							{
								Set_DutyCycle_DC_PWM(valor_pwm_actual - rampa_paso);
							}
						}

						rampa_ultimo_tick = HAL_GetTick();

						if (valor_pwm_actual == rampa_pwm_objetivo)
						{
							if (rampa_pwm_objetivo == PWM_C_detenido)
							{
								Set_DutyCycle_DC_PWM(0);


			 		 	        snprintf(buffer_tx,sizeof(buffer_tx),":DETENIDO\r\n");
			 		 	        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
							}

							sub_Estado = sub_Estado_siguiente;
                            if (sub_Estado == C_esperando_maestro)
                            {
                                cinta_generar_aviso_esperando_maestro();
                            }
						}
					}
					break;
				case C_esperando_maestro:
					break;

				default:
				    desactivar();
				    Estado = E_error;
				    break;
	 		 }
	 		 break;
	 	 case E_error:
	 		 break;
	 	 default:
	 		 desactivar();
	 		 Estado=E_error;

	 	     snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:\r\n");
	 	     HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
	 		 break;
  	  }
	 /* if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	  {
	      Error_Handler();
	  }*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//cuando la cinta se frena porque llega una botella
static void cinta_generar_aviso_esperando_maestro(void)
{
    aviso_esperando_secuencia++; //para diferenciar una botella nueva de un reintento

    if (aviso_esperando_secuencia == 0)  //evita utilizar cero después del desbordamiento de 65535 a 0.
	{
		aviso_esperando_secuencia = 1;
	}

    aviso_esperando_pendiente = 1; //evento guardado hasta que el maestro consulte
}

static void cinta_generar_aviso_peso(int32_t peso_mg)
{
    uint32_t peso_decigramos;

    if (peso_mg <= 0)
    {
        peso_decigramos = 0;
    }
    else
    {
        peso_decigramos = ((uint32_t)peso_mg + 50U) / 100U;  //convertimos de miligramos a decimas de gramo
    }

    if (peso_decigramos > 65535U)
    {
        peso_decigramos = 65535U;
    }

    aviso_peso_decigramos = (uint16_t)peso_decigramos;
    aviso_peso_pendiente = 1;

}

static HAL_StatusTypeDef cinta_enviar_rs485(uint8_t origen, uint8_t comando, uint8_t param_h, uint8_t param_l)
{
    HAL_Delay(RS485_RESPUESTA_DELAY_MS); //le da tiempo al maestro de terminar la transmision

    return RS485_Send_Packet(origen, comando, param_h, param_l); //origen es a donde lo mandamos
}

//para responder con ACK o NACK
static void cinta_responder_resultado(uint8_t origen, uint8_t respuesta, uint8_t comando_solicitado,  uint8_t codigo)
{
    if (cinta_enviar_rs485(origen, respuesta, comando_solicitado, codigo) != HAL_OK)
    {
        uart_enviar_texto("ERR,RS485,NO_SE_PUDO_ENVIAR_RESULTADO\r\n");
    }
}

static void procesar_rs485_cinta(void)
{
    uint8_t origen;
    uint8_t comando;
    uint8_t param_h;
    uint8_t param_l;
    uint16_t valor;

    if (rs485_paquete_listo == 0)
    {
        return;
    }

    origen = rs485_rx_origen;
    comando = rs485_rx_cmd;
    param_h = rs485_rx_param_h;
    param_l = rs485_rx_param_l;

    rs485_paquete_listo = 0;

    if (origen != ID_MAESTRO)
    {
        return;
    }

    valor = (uint16_t)(((uint16_t)param_h << 8U) | param_l);

    switch (comando)
    {
        case CMD_PING:
            if (cinta_enviar_rs485(origen, CMD_PONG, param_h, param_l) != HAL_OK)
            {
                //aviso local
            	uart_enviar_texto("ERROR: RS485,NO_SE_PUDO_ENVIAR_PONG\r\n");
            }
            break;

        case CMD_PEDIR_ESTADO_CINTA:
            if (cinta_enviar_rs485(origen, CMD_RESP_ESTADO_CINTA, Estado, sub_Estado) != HAL_OK)
            {
                uart_enviar_texto("ERROR: RS485,NO_SE_PUDO_ENVIAR_ESTADO\r\n");
            }
            break;

        case CMD_PEDIR_EVENTO_CINTA:

            if (aviso_esperando_pendiente != 0) //prioridad 1: botella esperando maesto
            {
                if (cinta_enviar_rs485(origen, CMD_EVENTO_CINTA_ESPERANDO_MAESTRO, (uint8_t)(aviso_esperando_secuencia >> 8U),
                		(uint8_t)(aviso_esperando_secuencia & 0xFFU)) != HAL_OK)
                {
                    uart_enviar_texto("ERROR: RS485,NO_SE_PUDO_ENVIAR_EVENTO_CINTA\r\n");
                }
            }

            else if (aviso_peso_pendiente != 0) //prioridad 2: pesaje terminado
            {
                if (cinta_enviar_rs485(origen, CMD_EVENTO_CINTA_PESAJE_COMPLETO, (uint8_t)(aviso_peso_decigramos >> 8U),
                        (uint8_t)(aviso_peso_decigramos & 0xFFU)) != HAL_OK)
                {
                    uart_enviar_texto("ERROR: RS485,NO_SE_PUDO_ENVIAR_EVENTO_PESO\r\n");
                }
            }
            else
            {
                if (cinta_enviar_rs485(origen, CMD_RESP_SIN_EVENTO, 0, 0) != HAL_OK)
                {
                    uart_enviar_texto("ERROR: RS485,NO_SE_PUDO_RESPONDER_SIN_EVENTO\r\n");
                }
            }

            break;

        case CMD_CINTA_CONFIGURAR:
            if ((param_h != 0) || (param_l != 1))
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_PARAMETRO_INVALIDO);
            }
            else if (Estado != E_desactivado)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_ESTADO_INVALIDO);
            }
            else
            {
                Estado = E_configurando;
                configuracion_lista = 0;

                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
            }
            break;

        case CMD_CINTA_TARA:
        	HAL_GPIO_WritePin(Led_ON_GPIO_Port, Led_ON_Pin, SET);
        	HAL_GPIO_WritePin(Led_OFF_GPIO_Port, Led_OFF_Pin, RESET);

            if ((param_h != 0) || (param_l != 1)) //solo acepta :CT1
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando,NACK_PARAMETRO_INVALIDO);
            }
            else if (Estado != E_configurando)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_ESTADO_INVALIDO);
            }
            else if (HX711_Tare(50U) != HAL_OK) //toma 50 muestras para calcular el valor del tara
            {
                configuracion_lista = 0;
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando,NACK_SENSOR);

                HAL_GPIO_WritePin(Led_OFF_GPIO_Port, Led_OFF_Pin, SET);
            }
            else
            {
                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
            }

            HAL_GPIO_WritePin(Led_ON_GPIO_Port, Led_ON_Pin, RESET);
            break;

        case CMD_CINTA_CALIBRAR:
        	HAL_GPIO_WritePin(Led_ON_GPIO_Port, Led_ON_Pin, SET);
        	HAL_GPIO_WritePin(Led_OFF_GPIO_Port, Led_OFF_Pin, RESET);

            if (Estado != E_configurando)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_ESTADO_INVALIDO);
            }
            else if (tare == 0)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_SIN_TARA);
            }
            else if (valor == 0)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_FUERA_DE_RANGO);
            }
            else if (HX711_Calibrate((float)valor * 1000.0f, 50U) != HAL_OK) //multiplica por 100 porque trabaja en miligramos
            {
                configuracion_lista = 0;

                cinta_responder_resultado(origen, CMD_RESP_NACK, comando,NACK_SENSOR);
                HAL_GPIO_WritePin(Led_OFF_GPIO_Port, Led_OFF_Pin, SET);
            }
            else
            {
                configuracion_lista = 1;

                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
                HAL_GPIO_WritePin(Led_ON_GPIO_Port, Led_ON_Pin, RESET);
            }
            break;

        case CMD_CINTA_ACTIVAR:
            if ((param_h != 0) || (param_l > 1))
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_PARAMETRO_INVALIDO);
            }
            else if (param_l == 0)
            {
                desactivar();
                Estado = E_desactivado;

                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
            }
            else if (configuracion_lista == 0)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_SIN_CONFIGURACION);
            }
            else if ((Estado != E_configurando) && (Estado != E_desactivado))
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_ESTADO_INVALIDO);
            }
            else
            {
                HAL_GPIO_WritePin(Led_ON_GPIO_Port, Led_ON_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(Led_OFF_GPIO_Port, Led_OFF_Pin, GPIO_PIN_RESET);

                Set_DutyCycle_DC_PWM(0);
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(DC_IN1_GPIO_Port, DC_IN1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(DC_IN2_GPIO_Port, DC_IN2_Pin, GPIO_PIN_RESET);

                Estado = E_activado;
                sub_Estado = C_detenida;

                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
            }
            break;

        case CMD_CINTA_CONTROL_BANDA:
            if ((param_h != 0) || (param_l > 2)) //acepta hasta :CB2 unicamente
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_PARAMETRO_INVALIDO);
            }
            else if (Estado != E_activado)
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_ESTADO_INVALIDO);
            }
            else if (param_l == 1)
            {
                if (sub_Estado != C_detenida)
                {
                    cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_OCUPADO);
                }
                else
                {
                	//parametros para arrancar
                    rampa_pwm_objetivo = PWM_C_andando;
                    rampa_paso = 10;
                    rampa_retardo_ms = 10;
                    rampa_ultimo_tick = HAL_GetTick();

                    sub_Estado_siguiente = C_andando;
                    sub_Estado = C_acelerando;

                    cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
                }
            }
            else if (param_l == 0)
            {
                if (valor_pwm_actual <= PWM_C_detenido)
                {
                    sub_Estado = C_detenida;
                    Set_DutyCycle_DC_PWM(0);
                }
                else
                {
                    rampa_pwm_objetivo = PWM_C_detenido;
                    rampa_paso = 10;
                    rampa_retardo_ms = 10;
                    rampa_ultimo_tick = HAL_GetTick();

                    sub_Estado_siguiente = C_detenida;
                    sub_Estado = C_desacelerando;
                }

                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
            }
            else //:CB2
            {
                if (valor_pwm_actual != 0)
                {
                    cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_OCUPADO);
                }
                else
                {
                    cantidad_pesajes = 0;
                    sub_Estado = C_pesando;

                    cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
                }
            }
            break;

        case CMD_CONFIRMAR_EVENTO_CINTA:
            if ((aviso_esperando_pendiente != 0) && (valor == aviso_esperando_secuencia)) //evento pendiente?
            	//¿La secuencia confirmada coincide?
            {
                aviso_esperando_pendiente = 0;
            }
            break;

        case CMD_CONFIRMAR_EVENTO_CINTA_PESAJE:
            if ((aviso_peso_pendiente != 0) && (valor == aviso_peso_decigramos))
            {
                aviso_peso_pendiente = 0;
            }
            break;

        case CMD_CINTA_CONTINUAR_PESAJE: //:CD1
            if ((param_h != 0) || (param_l != 1))
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_PARAMETRO_INVALIDO);
            }
            else if ((Estado != E_activado) || (sub_Estado != C_esperando_maestro))
            {
                cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_ESTADO_INVALIDO);
            }
            else
            {
            	//reseteamos variables del pesado
                aviso_esperando_pendiente = 0;
                cantidad_pesajes = 0;
                sub_Estado = C_pesando;

                cinta_responder_resultado(origen, CMD_RESP_ACK, comando, ACK_OK);
            }
            break;

        default: //comando desconocido
            cinta_responder_resultado(origen, CMD_RESP_NACK, comando, NACK_COMANDO_NO_SOPORTADO);
            break;
    }
}

static void uart_enviar_texto(const char *texto) //funcion para mostrar mensajes por la UART local
{
    if (texto == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart3, (uint8_t *)texto, (uint16_t)strlen(texto), 100U);
}

/*void enviar_estado_uart(void)
{
    int longitud;

    longitud = snprintf(
        buffer_tx,
        sizeof(buffer_tx),
        ":Q,%u,%u,%lu,%lu,%ld\r\n",
        (unsigned int)Estado,
        (unsigned int)sub_Estado,
        (unsigned long)objetos_ok,
        (unsigned long)objetos_descarte,
        (long)weight
    );

    if ((longitud > 0) && (longitud < (int)sizeof(buffer_tx)))
    {
        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,(uint16_t)longitud,HAL_MAX_DELAY
        );
    }
}

void interpretar_comando(void){
	switch (buffer_rx[0]){
		case 'A':
			if(buffer_rx[1]=='1'){
		        if (configuracion_lista == 1){
				HAL_GPIO_WritePin(GPIOA,Led_ON_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA,Led_OFF_Pin, GPIO_PIN_RESET);
				Estado= E_activado;

				//ACTIVAR MOTOR_dc
				Set_DutyCycle_DC_PWM(0);
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				HAL_GPIO_WritePin(GPIOB, DC_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, DC_IN2_Pin, GPIO_PIN_RESET);
				sub_Estado = C_detenida;
				snprintf(buffer_tx,sizeof(buffer_tx),"ACTIVADO\r\n");
				HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);
		        }
			}else if(buffer_rx[1]=='0') {
				desactivar();
				Estado= E_desactivado;
				snprintf(buffer_tx,sizeof(buffer_tx),"DESACTIVADO\r\n");
				HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);
			}
			break;
		case 'C':
		    if (Estado == E_desactivado){
		        Estado = E_configurando;
		        configuracion_lista = 0;
				snprintf(buffer_tx,sizeof(buffer_tx),"CONFIGURANDO...\r\n");
				HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx,strlen(buffer_tx),HAL_MAX_DELAY);
		    }
			break;
		case 'T':
			if (Estado==E_configurando){
				  HAL_GPIO_WritePin(GPIOA, Led_OFF_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_SET);
				  if (HX711_Tare(50) != HAL_OK){
					  HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, Led_OFF_Pin, GPIO_PIN_SET);
				      configuracion_lista = 0;
					  snprintf(buffer_tx,sizeof(buffer_tx),"ERROR: HX711\r\n");
					  HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
				      break;
				  }
				  HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_RESET);
		          snprintf(buffer_tx, sizeof(buffer_tx),"Tare: %ld\r\n",tare);
		          HAL_UART_Transmit(&huart3, (uint8_t*)buffer_tx, strlen(buffer_tx), HAL_MAX_DELAY);
			}
		break;
		case 'F':
			if (Estado==E_configurando){
				if(tare != 0){
					int pesoGramos = atoi(&buffer_rx[1]);
					if (pesoGramos > 0){
						float pesoMiligramos =(float)pesoGramos * 1000.0f;
						HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, Led_OFF_Pin, GPIO_PIN_RESET);
						if (HX711_Calibrate(pesoMiligramos, 50) == HAL_OK){
						    configuracion_lista = 1;
						}else{
							HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOA, Led_OFF_Pin, GPIO_PIN_SET);
						    configuracion_lista = 0;
		 		 	        snprintf(buffer_tx,sizeof(buffer_tx),"ERROR: HX711\r\n");
		 		 	        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
						    break;
						}
						HAL_GPIO_WritePin(GPIOA,Led_ON_Pin | Led_OFF_Pin,GPIO_PIN_RESET);
						snprintf(buffer_tx, sizeof(buffer_tx),"Factor: %.6f\r\n",calibrationFactor);
						HAL_UART_Transmit(&huart3, (uint8_t*)buffer_tx, strlen(buffer_tx), HAL_MAX_DELAY);
					}
				}else{
					Estado=E_desactivado;
					desactivar();
					configuracion_lista=0;
 		 	        snprintf(buffer_tx,sizeof(buffer_tx),"ERROR: NO TARA\r\n");
 		 	        HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
				}
			}
		break;
		case 'B':
		    if (buffer_rx[1] == '1'){
		        if ((Estado == E_activado) &&(sub_Estado == C_detenida)){
		        	//enciendo motor
		        	rampa_pwm_objetivo = PWM_C_andando;
					rampa_paso = 10;
					rampa_retardo_ms = 10;
					rampa_ultimo_tick = HAL_GetTick();
					sub_Estado_siguiente = C_andando;
					sub_Estado = C_acelerando;
		        }
		    }else if (buffer_rx[1] == '0'){
		        if (Estado == E_activado){
		            if (valor_pwm_actual <= PWM_C_detenido){
		                sub_Estado = C_detenida;
						Set_DutyCycle_DC_PWM(0);
		            }else{
		                rampa_pwm_objetivo = PWM_C_detenido;
		                rampa_paso = 10;
		                rampa_retardo_ms = 10;
		                rampa_ultimo_tick = HAL_GetTick();
		                sub_Estado_siguiente = C_detenida;
		                sub_Estado = C_desacelerando;
		            }
		        }
		    }else if (buffer_rx[1] == '2'){
		        if ((Estado == E_activado) &&(valor_pwm_actual == 0)) {
		            cantidad_pesajes = 0;
		            sub_Estado = C_pesando;
		        }
		    }
		    break;
		case 'D':
			if(buffer_rx[1]=='1'){
	            if ((Estado == E_activado) &&(sub_Estado == C_esperando_maestro)){
	                cantidad_pesajes = 0;
	                sub_Estado = C_pesando;
	            }
			}
			break;
		case 'Q':
		    enviar_estado_uart();
		    break;
		default:
			break;
	}
}*/

void Set_DutyCycle_DC_PWM(uint16_t valor_pwm)
{
	if (valor_pwm <= 1000) //usa una escala de 0 a 1000
	{
		//el contador del timer trabaja de 0 a 99
	    TIM1->CCR1 = (uint16_t)(((uint32_t)valor_pwm * TIM1->ARR) / 1000); //convertimos en la esccala del timer
	    valor_pwm_actual = valor_pwm;
	}
}

void desactivar(void){
	//prende led de apagado
	HAL_GPIO_WritePin(GPIOA, Led_ON_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Led_OFF_Pin, GPIO_PIN_SET);

	Set_DutyCycle_DC_PWM(0);
	HAL_GPIO_WritePin(GPIOB,DC_IN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,DC_IN2_Pin,GPIO_PIN_RESET);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1, PULSE_REPOSO);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,PULSE_REPOSO);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,PULSE_REPOSO);

    cantidad_pesajes = 0;
    weight = 0;
    objetos_ok = 0;
    objetos_descarte = 0;
    objetos_control = 0;

    cola_servo1.entrada = 0;
    cola_servo1.salida = 0;
    cola_servo1.cantidad = 0;

    cola_servo2.entrada = 0;
    cola_servo2.salida = 0;
    cola_servo2.cantidad = 0;

    cola_servo3.entrada = 0;
    cola_servo3.salida = 0;
    cola_servo3.cantidad = 0;

    eventos_sensor1 = 0;
    eventos_sensor2 = 0;
    eventos_sensor3 = 0;

    aviso_esperando_pendiente = 0U;
    aviso_peso_pendiente = 0U;
}

uint8_t Verificar_Peso_Por_Pasos(int32_t *peso_promedio)
{
    int64_t suma = 0;
    int32_t peso_minimo;
    int32_t peso_maximo;
    float desviacion;
    int32_t nueva_muestra;
    HAL_StatusTypeDef estado;

    estado = HX711_WeighNonBlocking(&nueva_muestra);

    if (estado == HAL_BUSY)
    {
        return 3;  //no hay una nueva medición disponible.
    }
    if (estado != HAL_OK)
    {
        cantidad_pesajes = 0;  //error
        return 0;
    }

    pesos_estabilidad[cantidad_pesajes] = nueva_muestra; //armamos las 10
    cantidad_pesajes++;

    if (cantidad_pesajes < 10)
    {
        return 3; //3 = todavía faltan muestras
    }

    peso_minimo = pesos_estabilidad[0];
    peso_maximo = pesos_estabilidad[0];

    for (uint8_t i = 0; i < 10; i++)
    {
        suma += pesos_estabilidad[i];
        if (pesos_estabilidad[i] < peso_minimo)
        {
            peso_minimo = pesos_estabilidad[i]; //buscamos la mas pequeña
        }

        if (pesos_estabilidad[i] > peso_maximo)
        {
            peso_maximo = pesos_estabilidad[i]; //buscamos la mas grande
        }
    }

    *peso_promedio = (int32_t)(suma / 10);

    cantidad_pesajes = 0;
    desviacion = ((float)(peso_maximo - peso_minimo) * 100.0f) /(float)(*peso_promedio);
    if (desviacion > 5.0f)
    {
        return 2; // desviacion alta por peso inestable
    }

    return 1; //pesaje completado
}

uint8_t FIFO_Agregar(FIFO *cola, uint8_t valor)
{
    if (cola->cantidad >= FIFO_TAMANO) //si la cola esta llena
    {
        return 0;
    }

    cola->datos[cola->entrada] = valor;
    cola->entrada++; //mueve el indice de entrada

    if (cola->entrada >= FIFO_TAMANO) //reiniciamos, arreglo circular
    {
        cola->entrada = 0;
    }

    cola->cantidad++;
    return 1; //el dato se agrego correctamente
}

uint8_t FIFO_Sacar(FIFO *cola, uint8_t *valor)
{
    if (cola->cantidad == 0) //si esta vacia
    {
        return 0;
    }

    *valor = cola->datos[cola->salida]; //copia el dat mas antiguo en la variable valor
    cola->salida++;

    if (cola->salida >= FIFO_TAMANO)
    {
        cola->salida = 0;
    }

    cola->cantidad--;
    return 1;
}

uint8_t Obtener_Destino(int32_t peso) //determina la salida del objeto
{
    if ((peso >= PESO_MIN_1) && (peso <= PESO_MAX_1))
    {
    	objetos_ok ++;
        return 1; //destino 1

    } else if ((peso >= PESO_MIN_2) &&(peso <= PESO_MAX_2))
    {
    	objetos_ok ++;
    	return 2;
    } else if ((peso >= PESO_MIN_3) &&(peso <= PESO_MAX_3))
    {
    	objetos_ok ++;
        return 3;
    }

    objetos_descarte ++;
    return 4;
}

void Procesar_Sensor1(void)
{
    uint8_t destino;  //variable que guarda en que servo tiene que abrir 1,2,3

    if (FIFO_Sacar(&cola_servo1, &destino) == 0)
    {
    	//mensaje UART local
		snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:OBJETO NO CLASIFICADO1\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
    	return;
    }

    if (destino == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PULSE_ABIERTO);
        objetos_control ++;
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PULSE_REPOSO);
        if (FIFO_Agregar(&cola_servo2, destino) == 0) //si la cola esta llena
        {
            desactivar();
            Estado = E_error;

			snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:COLA_LLENA\r\n");
			HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
        }
    }
}

void Procesar_Sensor2(void)
{
    uint8_t destino;

    if (FIFO_Sacar(&cola_servo2, &destino) == 0)
    {
		snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:OBJETO NO CLASIFICADO2\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
    	return;
    }

    if (destino == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,PULSE_ABIERTO);
        objetos_control ++;
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,PULSE_REPOSO);
        if (FIFO_Agregar(&cola_servo3, destino) == 0)
        {
            desactivar();
            Estado = E_error;

			snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:COLA_LLENA\r\n");
			HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
        }
    }
}

void Procesar_Sensor3(void)
{
    uint8_t destino;
    if (FIFO_Sacar(&cola_servo3, &destino) == 0)
    {
		snprintf(buffer_tx,sizeof(buffer_tx),"ERROR:OBJETO NO CLASIFICADO3\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t *)buffer_tx, strlen(buffer_tx),HAL_MAX_DELAY);
    	return;
    }

    if (destino == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,PULSE_ABIERTO);
        objetos_control ++;
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,PULSE_REPOSO);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t ahora = HAL_GetTick();

    if (GPIO_Pin == Sensor_1_Pin)
    {
        if ((ahora - ultimo_sensor1) >= SENSOR_ANTIRREBOTE_MS)
        {
            eventos_sensor1 = 1;
            ultimo_sensor1 = ahora;
        }
    }
    else if (GPIO_Pin == Sensor_2_Pin)
    {
        if ((ahora - ultimo_sensor2) >= SENSOR_ANTIRREBOTE_MS)
        {
            eventos_sensor2 = 1;
            ultimo_sensor2 = ahora;
        }
    }
    else if (GPIO_Pin == Sensor_3_Pin)
    {
        if ((ahora - ultimo_sensor3) >= SENSOR_ANTIRREBOTE_MS)
        {
            eventos_sensor3 = 1;
            ultimo_sensor3 = ahora;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RS485_Rx_Callback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RS485_Error_Callback(huart);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  for (volatile int i = 0; i < 1000000; i++)
	  {
	  }
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
