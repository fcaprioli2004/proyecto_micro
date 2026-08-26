/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rs485.h"
#include "protocolo_rs485.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	M_INACTIVO = 0,
	M_ACTIVO = 1,
	M_ALARMA = 2
} EstadoMaestro;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PC_CMD_BUFFER_SIZE       24

#define RESPUESTA_TIMEOUT_MS          1000
#define RESPUESTA_TIMEOUT_LARGO_MS    10000 //para las operaciones de la cinta que pueden tardar mas

#define PING_PARAM_H             0x12
#define PING_PARAM_L             0x34

#define POLLING_PERIODO_MS            50
#define POLLING_TIMEOUT_MS            100

#define SUPERVISION_PERIODO_MS 500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static EstadoMaestro estado_maestro = M_INACTIVO;

//variables para recibir comandos por la USART1
static uint8_t pc_rx_byte = 0;
static char pc_cmd_buffer[PC_CMD_BUFFER_SIZE];
static volatile uint8_t pc_cmd_indice = 0;
static volatile uint8_t pc_recibiendo = 0;
static volatile uint8_t pc_comando_listo = 0;

//variables para el RS485
static uint8_t esperando_respuesta = 0; //1 → ya mandamos algo
static uint8_t nodo_esperado = 0;
static uint8_t comando_pendiente = 0;
static uint16_t valor_pendiente = 0; //guarda el valor de 16 bits que hay en PARAM_L y PARAM_H
static uint32_t tick_respuesta = 0;
static uint32_t timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;
static char mensaje_pc[160]; //buffer auxiliar para construir textos con snprintf() y mostrar por pantalla

//variables manejo de eventos nodos
static uint8_t ack_evento_cinta_pendiente = 0;
static uint16_t ack_evento_cinta_secuencia = 0;
static uint16_t ultima_secuencia_evento_cinta = 0;
static uint8_t ultima_secuencia_evento_cinta_valida = 0;  //para no ejecutar dos veces la misma accion automática

static uint8_t ack_evento_peso_cinta_pendiente = 0;
static uint16_t ack_evento_peso_cinta_valor = 0;
static uint8_t peso_evento_ciclo_actual_mostrado = 0;

static uint8_t ack_evento_tanque_pendiente = 0;
static uint16_t ack_evento_tanque_secuencia = 0;
static uint16_t ultima_secuencia_evento_tanque = 0;
static uint8_t ultima_secuencia_evento_tanque_valida = 0;

static uint8_t inicio_dosif_automatico_pendiente = 0;
static uint8_t continuar_cinta_automatico_pendiente = 0;
static uint8_t ciclo_esperando_receta = 0;

static uint8_t nodo_polling_siguiente = ID_CINTA;
static uint32_t tick_polling = 0;

static uint32_t tick_supervision = 0;
static uint8_t nodo_supervision_siguiente = ID_TANQUE;
static uint8_t flag_consulta_supervision = 0;

static uint8_t parada_sistema_etapa = 0; //convierte la parada global en una pequeña secuencia

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void PC_Enviar(const char *texto);
static uint8_t PC_Parsear_UInt16(const char *texto, uint16_t *valor);
static void PC_Procesar_Comando(void);

static void Maestro_Enviar_Trama(uint8_t destino, uint8_t comando, uint8_t param_h, uint8_t param_l, const char *mensaje);
static void Maestro_Procesar_RS485(void);
static void Maestro_Procesar_Polling(void);
static void Maestro_Procesar_Timeout(void);
static uint8_t Maestro_Procesar_ACK_NACK(uint8_t origen,uint8_t comando,uint8_t param_h,uint8_t param_l);

static const char *Maestro_Nombre_Comando(uint8_t comando);
static const char *Maestro_Descripcion_Error(uint8_t codigo);
static const char *Maestro_Nombre_Alarma_Tanque(uint8_t alarma);
static const char *Maestro_Nombre_Estado_Dosificador(uint8_t estado);
static const char *Maestro_Nombre_Presencia_Botella(uint8_t presente);
static const char *Maestro_Nombre_Estado_Cinta(uint8_t estado);
static const char *Maestro_Nombre_Subestado_Cinta(uint8_t subestado);

static void Maestro_Confirmar_Evento_Cinta(void);
static void Maestro_Confirmar_Evento_Peso_Cinta(void);
static void Maestro_Confirmar_Evento_Tanque(void);
static void Maestro_Procesar_Inicio_Dosif_Automatico(void);
static void Maestro_Procesar_Continuacion_Cinta_Automatica(void);

static void Maestro_Entrar_Alarma(const char *motivo);
static void Maestro_Procesar_Parada_Sistema(void);
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    if (RS485_Init(&huart3, RS485_DE_GPIO_Port, RS485_DE_Pin, ID_MAESTRO) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1) != HAL_OK)
    {
        Error_Handler();
    }

    PC_Enviar("\r\nMAESTRO RS485 INICIADO\r\n");

    PC_Enviar(":MN    = ESTADO GENERAL DEL MAESTRO\r\n");
    PC_Enviar(":MA1   = ACTIVAR SISTEMA\r\n");
    PC_Enviar(":MA0   = DESACTIVAR SISTEMA\r\n");
    PC_Enviar(":MR1   = RESETEAR ALARMA\r\n");

    PC_Enviar(":TP    = PING TANQUE\r\n");
    PC_Enviar(":TN    = ESTADO TANQUE\r\n");
    PC_Enviar(":TM    = ESTADO DOSIFICADOR\r\n");
    PC_Enviar(":TS15  = SETPOINT TANQUE 15 cm\r\n");
    PC_Enviar(":TR500 = RECETA 500 ml\r\n");
    PC_Enviar(":TI1   = INICIAR DOSIFICACION\r\n");
    PC_Enviar(":TI0   = ABORTAR DOSIFICACION\r\n");
    PC_Enviar(":TA1   = RESETEAR ALARMA\r\n");

    PC_Enviar(":CP    = PING CINTA\r\n");
    PC_Enviar(":CN    = ESTADO CINTA\r\n");
    PC_Enviar(":CC1   = ENTRAR EN CONFIGURACION CINTA\r\n");
    PC_Enviar(":CT1   = HACER TARA\r\n");
    PC_Enviar(":CF500 = CALIBRAR CON 500 g\r\n");
    PC_Enviar(":CA1   = ACTIVAR CINTA\r\n");
    PC_Enviar(":CA0   = DESACTIVAR CINTA\r\n");
    PC_Enviar(":CB1   = ARRANCAR BANDA\r\n");
    PC_Enviar(":CB0   = DETENER BANDA\r\n");
    PC_Enviar(":CD1   = CONTINUAR PESAJE\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        Maestro_Procesar_RS485(); //recepcion de tramas
        Maestro_Procesar_Timeout();

        Maestro_Procesar_Parada_Sistema();  //si hay hay que realizar una parada lo hacemos

        Maestro_Confirmar_Evento_Cinta();
        Maestro_Procesar_Inicio_Dosif_Automatico();

        Maestro_Confirmar_Evento_Tanque();
        Maestro_Procesar_Continuacion_Cinta_Automatica();

        Maestro_Confirmar_Evento_Peso_Cinta();

        //procesar comando del usuario si el RS485 está libre y no hay acciones automáticas importantes pendientes.
        if ((pc_comando_listo != 0) && (esperando_respuesta == 0) && (ack_evento_cinta_pendiente == 0) &&
        		(ack_evento_peso_cinta_pendiente == 0) && (ack_evento_tanque_pendiente == 0) && (inicio_dosif_automatico_pendiente == 0) &&
                (continuar_cinta_automatico_pendiente == 0))
		{
			pc_comando_listo = 0;
			PC_Procesar_Comando();
		}

        Maestro_Procesar_Polling();
  /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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

static void PC_Enviar(const char *texto)
{
    if (texto == NULL)
    {
        return;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)texto, (uint16_t)strlen(texto), 100U);
}

static uint8_t PC_Parsear_UInt16(const char *texto, uint16_t *valor) //para convertir texto a numeros "1234" → 1234
{
    uint32_t acumulado = 0; //auxiliar para detectar si se pasa del tamaño

    if ((texto == NULL) || (valor == NULL) || (*texto == '\0'))
    {
        return 0;
    }

    while (*texto != '\0') //recorrer la cadena hasta el final
    {
        uint32_t digito;

        if ((*texto < '0') || (*texto > '9')) //comprobamos que sea un numero
        {
            return 0;
        }

        digito = (uint32_t)(*texto - '0');  //conversion a numero

        if (acumulado > ((65535 - digito) / 10)) //para no superar el tamañno maximo permitido
        {
            return 0;
        }

        acumulado = (acumulado * 10) + digito;
        texto++;
    }

    *valor = (uint16_t)acumulado;
    return 1;
}

static void Maestro_Entrar_Alarma(const char *motivo)
{
    if (estado_maestro == M_ALARMA) //si ya esta en alarma
    {
        return;
    }

    estado_maestro = M_ALARMA;

    //cancela automatismos
    inicio_dosif_automatico_pendiente = 0;
    continuar_cinta_automatico_pendiente = 0;
    ciclo_esperando_receta = 0;

    parada_sistema_etapa = 1;    //bandera para iniciar la parada

    PC_Enviar("\r\n*** SISTEMA EN ALARMA ***\r\n");

    if (motivo != NULL)
    {
        (void)snprintf(mensaje_pc, sizeof(mensaje_pc), "CAUSA: %s\r\n", motivo);
        PC_Enviar(mensaje_pc);
    }
}

static void Maestro_Procesar_Parada_Sistema(void)
{
    if (parada_sistema_etapa == 0)  //si hay una secuencia de parada pendiente procesala, sino sali
    {
        return;
    }

    if (esperando_respuesta != 0) //si hay un mensaje esperando, volvems
    {
        return;
    }

    switch (parada_sistema_etapa)
    {
        case 1: //detener dosificador

            if (RS485_Send_Packet(ID_TANQUE, CMD_TANQUE_CONTROL_DOSIF, 0, 0) == HAL_OK)
            {
                esperando_respuesta = 1;
                nodo_esperado = ID_TANQUE;
                comando_pendiente = CMD_TANQUE_CONTROL_DOSIF;
                valor_pendiente = 0;

                tick_respuesta = HAL_GetTick();
                timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;

                parada_sistema_etapa = 2;
                PC_Enviar("PARADA -> TANQUE: ABORTAR DOSIFICACION\r\n");
            }

            break;

        case 2: //desactivar cinta

            if (RS485_Send_Packet(ID_CINTA, CMD_CINTA_ACTIVAR, 0, 0) == HAL_OK)
            {
                esperando_respuesta = 1;
                nodo_esperado = ID_CINTA;
                comando_pendiente = CMD_CINTA_ACTIVAR;
                valor_pendiente = 0;

                tick_respuesta = HAL_GetTick();
                timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;

                parada_sistema_etapa = 3;
                PC_Enviar("PARADA -> CINTA: DESACTIVAR\r\n");
            }

            break;

        case 3:

            parada_sistema_etapa = 0;
            PC_Enviar("*** SECUENCIA DE PARADA FINALIZADA ***\r\n");

            break;

        default:
            break;
    }
}

static void Maestro_Enviar_Trama(uint8_t destino, uint8_t comando, uint8_t param_h, uint8_t param_l, const char *mensaje)
{
    HAL_StatusTypeDef estado;

    if (esperando_respuesta != 0) //ver si esperaba una rta
    {
        PC_Enviar("ERROR: RS485 OCUPADO, ESPERE LA RESPUESTA ACTUAL\r\n");
        return;
    }

    //comprobamos que no existan confirmaciones o acciones AUTOM. pendientes
    if ((ack_evento_cinta_pendiente != 0) ||
        (ack_evento_peso_cinta_pendiente != 0) ||
        (ack_evento_tanque_pendiente != 0) ||
        (inicio_dosif_automatico_pendiente != 0) ||
        (continuar_cinta_automatico_pendiente != 0))
    {
        PC_Enviar("ERROR: ACCION AUTOMATICA PENDIENTE, ESPERE\r\n");
        return;
    }

    estado = RS485_Send_Packet(destino, comando, param_h, param_l);

    if (estado != HAL_OK)
    {
        PC_Enviar("ERROR: NO SE PUDO TRANSMITIR POR RS485\r\n");
        return;
    }

    //si se envia correctamente, guardamos todos estos datos en memoria para comprobar y sabemos perfectamente que respuesta esperamos
    esperando_respuesta = 1;
    nodo_esperado = destino;
    comando_pendiente = comando;
    valor_pendiente = (uint16_t)(((uint16_t)param_h << 8) | param_l);
    tick_respuesta = HAL_GetTick();

    if ((comando == CMD_CINTA_TARA) || (comando == CMD_CINTA_CALIBRAR))
    {
        timeout_respuesta_ms = RESPUESTA_TIMEOUT_LARGO_MS;
    }
    else
    {
        timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;
    }

    if (mensaje != NULL)
    {
        PC_Enviar(mensaje);
    }
}

static void PC_Procesar_Comando(void)
{
    char nodo;
    char comando;
    uint16_t valor = 0U;
    const char *parametro;

    if ((pc_cmd_buffer[0] == '\0') ||(pc_cmd_buffer[1] == '\0')){ //VERFICAMOS MENSAJE
        PC_Enviar("ERROR: COMANDO INCOMPLETO\r\n");
        return;
    }

    nodo = pc_cmd_buffer[0];
    comando = pc_cmd_buffer[1];
    parametro = &pc_cmd_buffer[2];

    if ((nodo >= 'a') && (nodo <= 'z')){ //convertimos a mayuscula
        nodo = (char)(nodo - ('a' - 'A'));
    }
    if ((comando >= 'a') && (comando <= 'z')){
        comando = (char)(comando - ('a' - 'A'));
    }

    switch (nodo) //identificamos nodo
    {
        case 'M': //maestro
            switch (comando)
            {
                case 'N': //estado de maestro
                    if (*parametro != '\0') {
                        PC_Enviar( "ERROR: :MN NO RECIBE PARAMETROS\r\n");
                        return;
                    }
                    switch (estado_maestro)
                    {
                        case M_INACTIVO:
                            PC_Enviar("MAESTRO: INACTIVO\r\n");
                            break;

                        case M_ACTIVO:
                            PC_Enviar("MAESTRO: ACTIVO\r\n");
                            break;

                        case M_ALARMA:
                            PC_Enviar("MAESTRO: ALARMA\r\n");
                            break;

                        default:
                            PC_Enviar(
                                "MAESTRO: ESTADO DESCONOCIDO\r\n"
                            );
                            break;
                    }
                    break;

                case 'A': //activar/desactivar sistema
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0) ||(valor > 1))
                    {
                        PC_Enviar("ERROR: USE :MA0 O :MA1\r\n" ); //verificamos pametro correcto
                        return;
                    }
                    if (valor == 0) { //desactivar
                        if (estado_maestro == M_ALARMA)  {
                            PC_Enviar("ERROR: SISTEMA EN ALARMA.\r\n");
                            return;
                        }
                        inicio_dosif_automatico_pendiente = 0; //cancelar automaticos pendientes
                        continuar_cinta_automatico_pendiente = 0;
                        ciclo_esperando_receta = 0;

                        estado_maestro = M_INACTIVO;
                        parada_sistema_etapa = 1; //inicia secuencia de pararda
                        PC_Enviar("MAESTRO: SISTEMA INACTIVO\r\n");
                    } else { //activar
                        if (estado_maestro == M_ALARMA){
                            PC_Enviar( "ERROR: SISTEMA EN ALARMA.\r\n");
                            return;
                        }
                        estado_maestro = M_ACTIVO;

                        tick_polling = HAL_GetTick();//reiniciar polling
                        nodo_polling_siguiente = ID_CINTA;

                        PC_Enviar("MAESTRO: SISTEMA ACTIVO\r\n" );
                    }
                    break;

                case 'R': //reset alarma maestro
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0) ||(valor != 1)) {
                        PC_Enviar( "ERROR: USE :MR1\r\n" );
                        return;
                    }
                    if (estado_maestro != M_ALARMA) {
                        PC_Enviar( "MAESTRO: NO HAY ALARMA ACTIVA\r\n" );
                        return;
                    }
                    parada_sistema_etapa = 0; //cancelamos secuencia de parada
                    estado_maestro = M_INACTIVO;

                    PC_Enviar("MAESTRO: ALARMA RESETEADA. SISTEMA INACTIVO\r\n"  );
                    break;

                default: //no existe comando
                    PC_Enviar("ERROR: COMANDO INVALIDO PARA MAESTRO\r\n" );
                    break;
            }
            break;

        case 'T': // tanque
            switch (comando)
            {
                case 'P': //ping pong
                    if (*parametro != '\0') {
                        PC_Enviar( "ERROR: :TP NO RECIBE PARAMETROS\r\n" );
                        return;
                    }
                    Maestro_Enviar_Trama(ID_TANQUE,CMD_PING,PING_PARAM_H,PING_PARAM_L,
                    		"TX -> TANQUE: PING\r\n");
                    break;

                case 'N': //consulta estado tanque
                    if (*parametro != '\0'){
                        PC_Enviar("ERROR: :TN NO RECIBE PARAMETROS\r\n" );
                        return;
                    }
                    Maestro_Enviar_Trama( ID_TANQUE, CMD_PEDIR_ESTADO_TANQUE, 0, 0,
                        "TX -> TANQUE: PEDIR ESTADO\r\n");
                    break;

                case 'M': //consulta estado dosificador
                    if (*parametro != '\0') {
                        PC_Enviar( "ERROR: :TM NO RECIBE PARAMETROS\r\n");
                        return;
                    }
                    Maestro_Enviar_Trama(ID_TANQUE,CMD_PEDIR_ESTADO_DOSIF,0,0,
                        "TX -> TANQUE: PEDIR ESTADO DOSIFICADOR\r\n");
                    break;

                case 'S': //configurar setpoint

                    if (PC_Parsear_UInt16(parametro, &valor) == 0U) {
                        PC_Enviar("ERROR: SETPOINT INVALIDO\r\n");
                        return;
                    }
                    Maestro_Enviar_Trama( ID_TANQUE, CMD_TANQUE_SETPOINT, (uint8_t)(valor >> 8U), (uint8_t)(valor & 0xFFU),
                        "TX -> TANQUE: CONFIGURAR SETPOINT\r\n" );
                    break;

                case 'R': //configurar receta
                    if (PC_Parsear_UInt16(parametro, &valor) == 0U){
                        PC_Enviar( "ERROR: RECETA INVALIDA\r\n");
                        return;
                    }

                    Maestro_Enviar_Trama( ID_TANQUE, CMD_TANQUE_RECETA, (uint8_t)(valor >> 8U), (uint8_t)(valor & 0xFFU),
                        "TX -> TANQUE: CONFIGURAR RECETA\r\n");
                    break;

                case 'I': //abortar,iniciar dosificacion
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0U) || (valor > 1U)){
                        PC_Enviar( "ERROR: USE :TI0 O :TI1\r\n");
                        return;
                    }

                    if ((valor == 1) && (estado_maestro != M_ACTIVO)) { //inicia si M_ACTIVO
                        PC_Enviar( "ERROR: EL SISTEMA NO ESTA ACTIVO\r\n");
                        return;
                    }
                    Maestro_Enviar_Trama( ID_TANQUE,CMD_TANQUE_CONTROL_DOSIF,0,(uint8_t)valor,(valor == 1)
                            ? "TX -> TANQUE: INICIAR DOSIFICACION\r\n"
                            : "TX -> TANQUE: ABORTAR DOSIFICACION\r\n");
                    break;

                case 'A': //resetea alarma de tanque
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0) ||(valor != 1)){
                        PC_Enviar( "ERROR: USE :TA1\r\n");
                        return;
                    }
                    Maestro_Enviar_Trama(ID_TANQUE,  CMD_TANQUE_RESET_ALARMA,0,1,
                        "TX -> TANQUE: RESETEAR ALARMA\r\n"  );

                    break;

                default: //no existe comando
                    PC_Enviar( "ERROR: COMANDO INVALIDO PARA TANQUE\r\n");
                    break;
            }
            break;

        case 'C': //cinta
            switch (comando)
            {
                case 'P': //ping pong
                    if (*parametro != '\0') {
                        PC_Enviar( "ERROR: :CP NO RECIBE PARAMETROS\r\n" );
                        return;
                    }

                    Maestro_Enviar_Trama( ID_CINTA, CMD_PING, PING_PARAM_H, PING_PARAM_L,
                    		"TX -> CINTA: PING\r\n");
                    break;

                case 'N': //consultar estado cinta
                    if (*parametro != '\0'){
                        PC_Enviar("ERROR: :CN NO RECIBE PARAMETROS\r\n");
                        return;
                    }
                    Maestro_Enviar_Trama( ID_CINTA, CMD_PEDIR_ESTADO_CINTA, 0, 0,
                        "TX -> CINTA: PEDIR ESTADO\r\n");
                    break;

                case 'C': // :CC1 configuracion balanza
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0U) ||(valor != 1U)) {
                        PC_Enviar("ERROR: USE :CC1\r\n");
                        return;
                    }
                    Maestro_Enviar_Trama(ID_CINTA,CMD_CINTA_CONFIGURAR, 0, 1,
                        "TX -> CINTA: ENTRAR EN CONFIGURACION\r\n");
                    break;

                case 'T': //tara :CT1
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0U) ||(valor != 1U)) {
                        PC_Enviar("ERROR: USE :CT1\r\n" );
                        return;
                    }
                    Maestro_Enviar_Trama( ID_CINTA, CMD_CINTA_TARA, 0, 1,
                        "TX -> CINTA: HACER TARA\r\n");
                    break;

                case 'F': //calibracion con peso patron
                    if (PC_Parsear_UInt16(parametro, &valor) == 0) {
                        PC_Enviar("ERROR: PESO DE CALIBRACION INVALIDO\r\n" );
                        return;
                    }
                    Maestro_Enviar_Trama(ID_CINTA,CMD_CINTA_CALIBRAR,(uint8_t)(valor >> 8U),(uint8_t)(valor & 0xFFU),
                        "TX -> CINTA: CALIBRAR HX711\r\n");
                    break;

                case 'A': // Desactivar / activar cinta
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0) || (valor > 1)){
                        PC_Enviar("ERROR: USE :CA0 O :CA1\r\n");
                        return;
                    }
                    if ((valor == 1U) &&(estado_maestro != M_ACTIVO)) {
                    	PC_Enviar( "ERROR: EL SISTEMA NO ESTA ACTIVO\r\n" );
                        return;
                    }
                    Maestro_Enviar_Trama(ID_CINTA,CMD_CINTA_ACTIVAR, 0, (uint8_t)valor, (valor == 1)
                            ? "TX -> CINTA: ACTIVAR\r\n"
                            : "TX -> CINTA: DESACTIVAR\r\n");
                    break;
                case 'B':// :CB0 / :CB1 Detener / arrancar banda
                    if ((PC_Parsear_UInt16(parametro, &valor) == 0U) ||(valor > 1U)) {
                        PC_Enviar("ERROR: USE :CB0 O :CB1\r\n" );
                        return;
                    }
                    if ((valor == 1U) && (estado_maestro != M_ACTIVO))  {
                        PC_Enviar( "ERROR: EL SISTEMA NO ESTA ACTIVO\r\n"  );
                        return;
                    }

                    Maestro_Enviar_Trama( ID_CINTA,  CMD_CINTA_CONTROL_BANDA, 0, (uint8_t)valor, (valor == 1)
                            ? "TX -> CINTA: ARRANCAR BANDA\r\n"
                            : "TX -> CINTA: DETENER BANDA\r\n");
                    break;

                case 'D': // CD1 continuar pesaje

                    if ((PC_Parsear_UInt16(parametro, &valor) == 0U) || (valor != 1U)){ PC_Enviar(
                            "ERROR: USE :CD1\r\n" );
                        return;
                    }
                    if (estado_maestro != M_ACTIVO){
                        PC_Enviar( "ERROR: EL SISTEMA NO ESTA ACTIVO\r\n" );
                        return;
                    }
                    Maestro_Enviar_Trama( ID_CINTA, CMD_CINTA_CONTINUAR_PESAJE, 0, 1,
                        "TX -> CINTA: CONTINUAR PESAJE\r\n" );
                    break;

                default: //comandos no existentes
                    PC_Enviar("ERROR: COMANDO INVALIDO PARA CINTA\r\n"  );
                    break;
            }
            break;

        default: //nodo invalido
            PC_Enviar("ERROR: NODO INVALIDO. USE T, C o M\r\n" );
            break;
    }
}
static void Maestro_Confirmar_Evento_Cinta(void) //confirma que el maestro recibió el evento para que la cinta no lo mande mas
{
	//Hay un evento de la cinta pendiente de confirmar y el maestro no está esperando la respuesta de otra solicitud.
    if ((ack_evento_cinta_pendiente == 0) || (esperando_respuesta == 1))
    {
        return;
    }

    HAL_Delay(5);

    if (RS485_Send_Packet(ID_CINTA, CMD_CONFIRMAR_EVENTO_CINTA, (uint8_t)(ack_evento_cinta_secuencia >> 8),
            (uint8_t)(ack_evento_cinta_secuencia & 0xFF)) == HAL_OK)
    {
        ack_evento_cinta_pendiente = 0;
    }
}

static void Maestro_Procesar_Inicio_Dosif_Automatico(void)
{
	if (estado_maestro != M_ACTIVO)
	{
		return;
	}

    HAL_StatusTypeDef estado;

    if (inicio_dosif_automatico_pendiente == 0)
    {
        return;
    }

    if ((ack_evento_cinta_pendiente != 0) || (esperando_respuesta != 0)) //espera que se haya enviado el ACK y que este libre
    {
        return;
    }

    HAL_Delay(5);

    //mandamos comando :TI1
    estado = RS485_Send_Packet(ID_TANQUE, CMD_TANQUE_CONTROL_DOSIF, 0, 1);

    if (estado != HAL_OK)
    {
        inicio_dosif_automatico_pendiente = 0;
        PC_Enviar("ERROR: NO SE PUDO INICIAR AUTOMATICAMENTE "
            "EL DOSIFICADOR. USE :TI1\r\n");
        return;
    }

    esperando_respuesta = 1;
    nodo_esperado = ID_TANQUE;
    comando_pendiente = CMD_TANQUE_CONTROL_DOSIF;
    valor_pendiente = 1;

    tick_respuesta = HAL_GetTick();
    timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;

    PC_Enviar("TX AUTOMATICO -> TANQUE: INICIAR DOSIFICACION\r\n");
}

static void Maestro_Confirmar_Evento_Tanque(void)
{
    if ((ack_evento_tanque_pendiente == 0) || (esperando_respuesta == 1))
    {
        return;
    }

    HAL_Delay(5);

    if (RS485_Send_Packet(ID_TANQUE, CMD_CONFIRMAR_EVENTO_TANQUE, (uint8_t)(ack_evento_tanque_secuencia >> 8),
            (uint8_t)(ack_evento_tanque_secuencia & 0xFF)) == HAL_OK)
    {
        ack_evento_tanque_pendiente = 0;  //se envia la confirmacion y se elimina el flag
    }
}

static void Maestro_Procesar_Continuacion_Cinta_Automatica(void)
{
	if (estado_maestro != M_ACTIVO)
	    {
	        return;
	    }

    HAL_StatusTypeDef estado;

    if (continuar_cinta_automatico_pendiente == 0)
    {
        return;
    }

    if ((ack_evento_tanque_pendiente != 0) || (esperando_respuesta != 0))
    {
        return;
    }

    HAL_Delay(5);

    //mandamos :CD1
    estado = RS485_Send_Packet(ID_CINTA, CMD_CINTA_CONTINUAR_PESAJE, 0, 1);

    if (estado != HAL_OK)
    {
        continuar_cinta_automatico_pendiente = 0;
        PC_Enviar("ERROR: NO SE PUDO CONTINUAR AUTOMATICAMENTE "
            "LA CINTA. USE :CD1\r\n");
        return;
    }

    continuar_cinta_automatico_pendiente = 0;

    esperando_respuesta = 1;
    nodo_esperado = ID_CINTA;
    comando_pendiente = CMD_CINTA_CONTINUAR_PESAJE;
    valor_pendiente = 1;

    tick_respuesta = HAL_GetTick();
    timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;

    PC_Enviar("TX AUTOMATICO -> CINTA: CONTINUAR PESAJE\r\n");
}

static void Maestro_Confirmar_Evento_Peso_Cinta(void)
{
    if ((ack_evento_peso_cinta_pendiente == 0) || (esperando_respuesta != 0))
    {
        return;
    }

    HAL_Delay(5);

    if (RS485_Send_Packet(ID_CINTA, CMD_CONFIRMAR_EVENTO_CINTA_PESAJE, (uint8_t)(ack_evento_peso_cinta_valor >> 8),
            (uint8_t)(ack_evento_peso_cinta_valor & 0xFF)) == HAL_OK)
    {
        ack_evento_peso_cinta_pendiente = 0;
    }
}

static void Maestro_Procesar_Polling(void)
{
    uint32_t ahora;
    uint8_t destino;
    uint8_t comando;

    if (parada_sistema_etapa != 0) //si estamos procesando una parada le damos prioridad
    {
        return;
    }

    if (estado_maestro == M_ALARMA)
    {
        return;
    }

    if (esperando_respuesta != 0)
    {
        return;
    }

    if (pc_comando_listo != 0)
    {
        return;
    }

    uint8_t consulta_supervision = 0;
    ahora = HAL_GetTick();

    //primero → supervisión de estado y alarmas general
    if ((ahora - tick_supervision) >= SUPERVISION_PERIODO_MS)
    {
        tick_supervision = ahora;
        consulta_supervision = 1;

        if (nodo_supervision_siguiente == ID_TANQUE)
        {
            destino = ID_TANQUE;
            comando = CMD_PEDIR_ESTADO_TANQUE;
        }
        else
        {
            destino = ID_CINTA;
            comando = CMD_PEDIR_ESTADO_CINTA;
        }
    } else //polling de eventos automáticos
    {
        //se realizan solo con el sistema en activo
        if (estado_maestro != M_ACTIVO)
        {
            return;
        }

        // las acciones del proceso automático tienen prioridad
        if ((ack_evento_cinta_pendiente != 0) || (ack_evento_peso_cinta_pendiente != 0) || (ack_evento_tanque_pendiente != 0) ||
            (inicio_dosif_automatico_pendiente != 0) || (continuar_cinta_automatico_pendiente != 0))
        {
            return;
        }

        if ((ahora - tick_polling) < POLLING_PERIODO_MS) //si todavia no paso el tiempo suficiente, salir
        {
            return;
        }

        tick_polling = ahora;

        if (nodo_polling_siguiente == ID_CINTA)
        {
            destino = ID_CINTA;
            comando = CMD_PEDIR_EVENTO_CINTA;
        }
        else
        {
            destino = ID_TANQUE;
            comando = CMD_PEDIR_EVENTO_TANQUE;
        }
    }

    //envio comun de la trama
    if (RS485_Send_Packet(destino, comando, 0, 0) != HAL_OK)
    {
        return;
    }

    esperando_respuesta = 1;
    nodo_esperado = destino;
    comando_pendiente = comando;
    valor_pendiente = 0;

    flag_consulta_supervision = consulta_supervision;

    tick_respuesta = HAL_GetTick();
    timeout_respuesta_ms = POLLING_TIMEOUT_MS;

    //cambiamos de nodo
    if (consulta_supervision == 1) //si se realizo la consulta de supervisión
    {
        if (nodo_supervision_siguiente == ID_TANQUE)
        {
            nodo_supervision_siguiente = ID_CINTA;
        }
        else
        {
            nodo_supervision_siguiente = ID_TANQUE;
        }
    }
    else
    {
        if (nodo_polling_siguiente == ID_CINTA)
        {
            nodo_polling_siguiente = ID_TANQUE;
        }
        else
        {
            nodo_polling_siguiente = ID_CINTA;
        }
    }
}

static void Maestro_Procesar_RS485(void)
{

    if (rs485_paquete_listo == 0U){ //verificamos recepcion paquete
        return;
    }
    uint8_t origen = rs485_rx_origen;
    uint8_t comando = rs485_rx_cmd;
    uint8_t param_h = rs485_rx_param_h;
    uint8_t param_l = rs485_rx_param_l;
    uint8_t respuesta_aceptada = 0;

    rs485_paquete_listo = 0;

    if (esperando_respuesta == 0U) {
        (void)snprintf( mensaje_pc,  sizeof(mensaje_pc),"RX NO SOLICITADO: ORIGEN=%u CMD=0x%02X\r\n",origen,comando );
        PC_Enviar(mensaje_pc);
        return;
    }

    if (origen != nodo_esperado){
        (void)snprintf( mensaje_pc,sizeof(mensaje_pc), "RTA INESPERADA: ORIGEN=%u CMD=0x%02X\r\n",origen,comando);
        PC_Enviar(mensaje_pc);
        return;
    }

    switch (origen) //identifico el nodo
    {
        case ID_TANQUE:
            switch (comando)
            {
                case CMD_EVENTO_TANQUE_LLENADO_COMPLETO: //finalizo la dosificacion
                    if (comando_pendiente == CMD_PEDIR_EVENTO_TANQUE){
                        uint16_t secuencia_evento;
                        secuencia_evento = (uint16_t)( ((uint16_t)param_h << 8) |param_l);
                        ack_evento_tanque_pendiente = 1;
                        ack_evento_tanque_secuencia = secuencia_evento;

                        if ((ultima_secuencia_evento_tanque_valida == 0U) || (secuencia_evento != ultima_secuencia_evento_tanque)) {
                            ultima_secuencia_evento_tanque_valida = 1;
                            ultima_secuencia_evento_tanque = secuencia_evento;

                            if (estado_maestro == M_ACTIVO) {
                                continuar_cinta_automatico_pendiente = 1;
                                PC_Enviar(  "\r\n*** AVISO TANQUE: ""LLENADO COMPLETADO ***\r\n");
                                PC_Enviar( "El maestro ordenara automaticamente ""a la cinta continuar con el pesaje.\r\n" );
                            }
                        }
                        respuesta_aceptada = 1U;
                    }
                    break;

                case CMD_RESP_SIN_EVENTO: //no hay eventos
                    if (comando_pendiente == CMD_PEDIR_EVENTO_TANQUE){
                        respuesta_aceptada = 1U;
                    }
                    break;

                case CMD_PONG:
                    if ((comando_pendiente == CMD_PING) &&(param_h == PING_PARAM_H) && (param_l == PING_PARAM_L)){
                        PC_Enviar( "RX <- TANQUE: PONG CORRECTO\r\n" );
                        respuesta_aceptada = 1U;
                    }
                    break;

                case CMD_RESP_ESTADO_TANQUE: //estado tanque
                    if (comando_pendiente ==CMD_PEDIR_ESTADO_TANQUE) {
                        if (flag_consulta_supervision == 0) { // consulta manual
                            (void)snprintf( mensaje_pc, sizeof(mensaje_pc), "TANQUE: NIVEL=%u cm,ALARMA=%s\r\n", param_l, Maestro_Nombre_Alarma_Tanque(param_h));
                            PC_Enviar(mensaje_pc);
                        }
                        if (param_h != 0U) // alarma
                        {
                            Maestro_Entrar_Alarma( Maestro_Nombre_Alarma_Tanque(param_h) );
                        }
                        respuesta_aceptada = 1;
                    }
                    break;

                case CMD_RESP_ESTADO_DOSIF:
                    if (comando_pendiente ==CMD_PEDIR_ESTADO_DOSIF){
                        (void)snprintf(mensaje_pc, sizeof(mensaje_pc), "DOSIF: ESTADO=%s,BOTELLA=%s\r\n",Maestro_Nombre_Estado_Dosificador(param_h),Maestro_Nombre_Presencia_Botella(param_l));
                        PC_Enviar(mensaje_pc);
                        respuesta_aceptada = 1;
                    }
                    break;

                case CMD_RESP_ACK:
                case CMD_RESP_NACK:
                    respuesta_aceptada =Maestro_Procesar_ACK_NACK(origen,comando, param_h, param_l);
                    break;

                default:
                    break;
            }

            break;

        case ID_CINTA:
            switch (comando)
            {
                case CMD_EVENTO_CINTA_ESPERANDO_MAESTRO: //botella esperando maestro
                    if (comando_pendiente ==CMD_PEDIR_EVENTO_CINTA) {
                        uint16_t secuencia_evento;
                        secuencia_evento =(uint16_t)(((uint16_t)param_h << 8) | param_l);
                        ack_evento_cinta_pendiente = 1; //confirmamos evento
                        ack_evento_cinta_secuencia =secuencia_evento;

                        if ((ultima_secuencia_evento_cinta_valida == 0) || (secuencia_evento != ultima_secuencia_evento_cinta)) {
                            ultima_secuencia_evento_cinta_valida = 1; //evitamos eventos repetidos
                            ultima_secuencia_evento_cinta = secuencia_evento;
                            peso_evento_ciclo_actual_mostrado = 0;

                            if (estado_maestro == M_ACTIVO){
                                inicio_dosif_automatico_pendiente = 1;
                                PC_Enviar(  "\r\n*** AVISO CINTA: " "OBJETO DETECTADO, " "ESPERANDO AL DOSIFICADOR ***\r\n"  );
                                PC_Enviar( "El maestro iniciara automaticamente " "el llenado.\r\n" );
                            }
                        }
                        respuesta_aceptada = 1;
                    }

                    break;

                case CMD_EVENTO_CINTA_PESAJE_COMPLETO: // se completo el pesaje
                    if (comando_pendiente == CMD_PEDIR_EVENTO_CINTA)  {
                        uint16_t peso_decigramos;
                        peso_decigramos = (uint16_t)(((uint16_t)param_h << 8) | param_l );
                        ack_evento_peso_cinta_pendiente = 1; //confirmamos el evento
                        ack_evento_peso_cinta_valor = peso_decigramos;

                        if (peso_evento_ciclo_actual_mostrado == 0) {
                        	peso_evento_ciclo_actual_mostrado = 1U;
                            (void)snprintf(mensaje_pc,sizeof(mensaje_pc), "\r\n*** PESAJE COMPLETADO: " "%u,%u g ***\r\n", (unsigned int)(peso_decigramos / 10U),(unsigned int)(peso_decigramos % 10U));
                            PC_Enviar(mensaje_pc);
                        }
                        respuesta_aceptada = 1;
                    }
                    break;

                case CMD_RESP_SIN_EVENTO: //no hay eventos
                    if (comando_pendiente ==CMD_PEDIR_EVENTO_CINTA) {
                        respuesta_aceptada = 1;
                    }
                    break;

                case CMD_PONG:
                    if ((comando_pendiente == CMD_PING) &&(param_h == PING_PARAM_H) &&(param_l == PING_PARAM_L)) {
                        PC_Enviar("RX <- CINTA: PONG CORRECTO\r\n" );
                        respuesta_aceptada = 1;
                    }
                    break;

                case CMD_RESP_ESTADO_CINTA:
                    if (comando_pendiente ==CMD_PEDIR_ESTADO_CINTA){
                        if (flag_consulta_supervision == 0){ //consulta manual
                            (void)snprintf(  mensaje_pc, sizeof(mensaje_pc), "CINTA: ESTADO=%s,SUBESTADO=%s\r\n",
                            		Maestro_Nombre_Estado_Cinta(param_h),  Maestro_Nombre_Subestado_Cinta(param_l) );
                            PC_Enviar(mensaje_pc);
                        }

                        if (param_h == 3U){ //error en la cinta
                            Maestro_Entrar_Alarma( "ERROR CINTA" );
                        }
                        respuesta_aceptada = 1U;
                    }
                    break;

                case CMD_RESP_ACK:
                case CMD_RESP_NACK:
                    respuesta_aceptada = Maestro_Procesar_ACK_NACK(origen,comando,param_h,param_l );
                    break;

                default: //comando invalido
                    break;
            }
            break;

        default:// nodo invalido
            (void)snprintf( mensaje_pc, sizeof(mensaje_pc), "ERROR: ORIGEN RS485 INVALIDO=%u\r\n", origen );
            PC_Enviar(mensaje_pc);
            return;
    }
    if (respuesta_aceptada != 0) { //respuesta valida
        esperando_respuesta = 0U;
        nodo_esperado = 0U;
        comando_pendiente = 0U;
        valor_pendiente = 0U;

        flag_consulta_supervision = 0U;

        timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;
    } else { //comando no valido
        (void)snprintf( mensaje_pc, sizeof(mensaje_pc),"COMANDO NO ESPERADO: ORIGEN=%u CMD=0x%02X H=0x%02X L=0x%02X\r\n",
            origen,comando, param_h, param_l );
        PC_Enviar(mensaje_pc);
    }
}

static uint8_t Maestro_Procesar_ACK_NACK(uint8_t origen,uint8_t comando,uint8_t param_h,uint8_t param_l)
{

    if ((comando != CMD_RESP_ACK) && (comando != CMD_RESP_NACK)){
        return 0;
    }

    if (param_h != comando_pendiente){ //se verifica que se recibio el comando correcto
        return 0;
    }

    if (comando == CMD_RESP_ACK) {
        switch (param_h)
        {
            case CMD_TANQUE_SETPOINT:
                (void)snprintf( mensaje_pc, sizeof(mensaje_pc),"ACK <- TANQUE: SETPOINT=%u cm\r\n",(unsigned int)valor_pendiente);
                break;

            case CMD_TANQUE_RECETA:
                if (ciclo_esperando_receta != 0U) {
                    inicio_dosif_automatico_pendiente = 1U;
                    (void)snprintf(mensaje_pc, sizeof(mensaje_pc),"ACK <- TANQUE: RECETA=%u ml\r\n"
                        "Se reintentara automaticamente el llenado de la botella pendiente.\r\n", (unsigned int)valor_pendiente );
                }  else {
                    (void)snprintf( mensaje_pc, sizeof(mensaje_pc),"ACK <- TANQUE: RECETA=%u ml\r\n", (unsigned int)valor_pendiente );
                }
                break;

            case CMD_TANQUE_CONTROL_DOSIF:
                inicio_dosif_automatico_pendiente = 0U;
                ciclo_esperando_receta = 0U;

                if (valor_pendiente == 0U){
                    (void)snprintf(mensaje_pc, sizeof(mensaje_pc), "ACK <- TANQUE: DOSIFICACION ABORTADA\r\n" );
                } else if (param_l == ACK_ESPERANDO_BOTELLA)  {
                    (void)snprintf( mensaje_pc, sizeof(mensaje_pc), "ACK <- TANQUE: ESPERANDO BOTELLA\r\n");
                } else {
                    (void)snprintf(mensaje_pc, sizeof(mensaje_pc),"ACK <- TANQUE: DOSIFICACION INICIADA\r\n"  );
                }
                break;

            case CMD_TANQUE_RESET_ALARMA:
                (void)snprintf(mensaje_pc, sizeof(mensaje_pc), "ACK <- TANQUE: ALARMA RESETEADA\r\n");
                break;

            case CMD_CINTA_CONFIGURAR:
                (void)snprintf(mensaje_pc, sizeof(mensaje_pc),"ACK <- CINTA: MODO CONFIGURACION\r\n" );
                break;

            case CMD_CINTA_TARA:
                (void)snprintf( mensaje_pc, sizeof(mensaje_pc), "ACK <- CINTA: TARA COMPLETADA\r\n");
                break;

            case CMD_CINTA_CALIBRAR:
                (void)snprintf(mensaje_pc,sizeof(mensaje_pc), "ACK <- CINTA: CALIBRADA CON %u g\r\n", (unsigned int)valor_pendiente);
                break;

            case CMD_CINTA_ACTIVAR:
                (void)snprintf( mensaje_pc,sizeof(mensaje_pc),(valor_pendiente == 1U)? "ACK <- CINTA: ACTIVADA\r\n": "ACK <- CINTA: DESACTIVADA\r\n"  );
                break;

            case CMD_CINTA_CONTROL_BANDA:
                if (valor_pendiente == 0U) {
                	(void)snprintf(mensaje_pc,sizeof(mensaje_pc),"ACK <- CINTA: BANDA DETENIENDOSE\r\n");
                } else {
                    (void)snprintf( mensaje_pc, sizeof(mensaje_pc), "ACK <- CINTA: BANDA ARRANCANDO\r\n" );
                }
                break;

            case CMD_CINTA_CONTINUAR_PESAJE:
                (void)snprintf( mensaje_pc,sizeof(mensaje_pc),"ACK <- CINTA: CONTINUAR PESAJE\r\n" );
                break;

            default:
                (void)snprintf( mensaje_pc,sizeof(mensaje_pc),"ACK <- NODO %u: CMD=0x%02X CODIGO=%u\r\n",origen, param_h,param_l );
                break;
        }
    } else { //llego un NACK

        if ((origen == ID_TANQUE) &&(param_h == CMD_TANQUE_CONTROL_DOSIF) && //no habia receta
            (param_l == NACK_SIN_RECETA) &&(inicio_dosif_automatico_pendiente != 0))  {

            inicio_dosif_automatico_pendiente = 0;
            ciclo_esperando_receta = 1;
            (void)snprintf(mensaje_pc, sizeof(mensaje_pc),"\r\nERROR: HAY UNA BOTELLA ESPERANDO. "
                "CONFIGURE LA RECETA CON :TRxxx\r\n" );
        } else { //fallo inicio de dosificacion automatico, aborto
            if ((origen == ID_TANQUE) && (param_h == CMD_TANQUE_CONTROL_DOSIF) &&
                (inicio_dosif_automatico_pendiente != 0U)){
                inicio_dosif_automatico_pendiente = 0;
                ciclo_esperando_receta = 0;
            }

            (void)snprintf(  mensaje_pc, sizeof(mensaje_pc), "NACK <- %s: %s,ERROR=%s\r\n",(origen == ID_TANQUE) ? "TANQUE" : "CINTA",
                Maestro_Nombre_Comando(param_h), Maestro_Descripcion_Error(param_l));
        }
    }
    PC_Enviar(mensaje_pc);
    return 1;
}

static void Maestro_Procesar_Timeout(void)
{
    if (esperando_respuesta == 0U){
        return;
    }

    if ((HAL_GetTick() - tick_respuesta) < timeout_respuesta_ms){
        return;
    }
//hay timeout
    if (flag_consulta_supervision != 0U){
    	//era por polling,no hacer nada
    } else if (parada_sistema_etapa != 0U) {
            PC_Enviar("ERROR: TIMEOUT DURANTE LA PARADA DEL SISTEMA\r\n");
    } else {
        switch (comando_pendiente)
        {
            case CMD_PEDIR_EVENTO_CINTA: // era por polling,no hacer nada
            case CMD_PEDIR_EVENTO_TANQUE:
                break;

            case CMD_TANQUE_CONTROL_DOSIF:
                if (valor_pendiente == 1U){
                    inicio_dosif_automatico_pendiente = 0U;
                    PC_Enviar("ERROR: TIMEOUT AL INICIAR DOSIFICACION. " "REINTENTE CON :TI1\r\n");
                } else {
                    PC_Enviar( "ERROR: TIMEOUT AL ABORTAR DOSIFICACION. " "REINTENTE CON :TI0\r\n");
                }
                break;

            case CMD_CINTA_CONTINUAR_PESAJE:
                continuar_cinta_automatico_pendiente = 0U;
                PC_Enviar( "ERROR: TIMEOUT AL CONTINUAR EL PESAJE. ""REINTENTE CON :CD1\r\n" );
                break;

            default:
                PC_Enviar("ERROR: TIMEOUT, EL NODO NO RESPONDIO\r\n" );
                break;
        }
    }
    esperando_respuesta = 0U;
    nodo_esperado = 0U;
    comando_pendiente = 0U;
    valor_pendiente = 0U;
    flag_consulta_supervision = 0U;

    timeout_respuesta_ms = RESPUESTA_TIMEOUT_MS;
}



static const char *Maestro_Nombre_Comando(uint8_t comando) //convertir números binarios del protocolo en textos
//no controlan la máquina ni modifican la comunicación. Solo traducen códigos a texto.
{
    switch (comando) //recibe el codigo numerico de un comando del protocolo
    {
        case CMD_TANQUE_SETPOINT:
            return "SETPOINT";

        case CMD_TANQUE_RECETA:
            return "RECETA";

        case CMD_TANQUE_CONTROL_DOSIF:
            return "DOSIFICACION";

        case CMD_TANQUE_RESET_ALARMA:
            return "RESET_ALARMA";

        case CMD_CINTA_CONFIGURAR:
            return "CONFIGURAR_CINTA";

        case CMD_CINTA_TARA:
            return "TARA";

        case CMD_CINTA_CALIBRAR:
            return "CALIBRACION";

        case CMD_CINTA_ACTIVAR:
            return "ACTIVAR_CINTA";

        case CMD_CINTA_CONTROL_BANDA:
            return "CONTROL_BANDA";

        case CMD_CINTA_CONTINUAR_PESAJE:
            return "CONTINUAR_PESAJE";

        default:
            return "DESCONOCIDO";
    }
}

static const char *Maestro_Descripcion_Error(uint8_t codigo) //el esclavo recibió correctamente la trama, pero no pudo o no quiso ejecutar la orden.
{
    switch (codigo)
    {
        case NACK_PARAMETRO_INVALIDO:
            return "PARAMETRO_INVALIDO";

        case NACK_FUERA_DE_RANGO:
            return "FUERA_DE_RANGO";

        case NACK_OCUPADO:
            return "OCUPADO";

        case NACK_ALARMA_ACTIVA:
            return "ALARMA_ACTIVA";

        case NACK_SIN_RECETA:
            return "SIN_RECETA";

        case NACK_RETIRE_BOTELLA:
            return "RETIRE_BOTELLA";

        case NACK_COMANDO_NO_SOPORTADO:
            return "COMANDO_NO_SOPORTADO";

        case NACK_ESTADO_INVALIDO:
            return "ESTADO_INVALIDO";

        case NACK_SENSOR:
            return "ERROR_SENSOR";

        case NACK_SIN_TARA:
            return "SIN_TARA";

        case NACK_SIN_CONFIGURACION:
            return "SIN_CONFIGURACION";

        default:
            return "ERROR_DESCONOCIDO";
    }
}

static const char *Maestro_Nombre_Alarma_Tanque(uint8_t alarma)
{
    switch (alarma)
    {
        case 0:
            return "SIN_ERROR";

        case 1:
            return "MARCHA_EN_SECO";

        case 2:
            return "REBOSE";

        case 3:
            return "ERROR_SENSOR_NIVEL";

        default:
            return "ALARMA_DESCONOCIDA";
    }
}

static const char *Maestro_Nombre_Estado_Dosificador(uint8_t estado)
{
    switch (estado)
    {
        case 0:
            return "ESPERANDO_BOTELLA";

        case 1:
            return "ESPERANDO_INICIO";

        case 2:
            return "LLENANDO";

        case 3:
            return "BOTELLA_LLENA";

        default:
            return "ESTADO_DESCONOCIDO";
    }
}

static const char *Maestro_Nombre_Presencia_Botella(uint8_t presente)
{
    switch (presente)
    {
        case 0:
            return "AUSENTE";

        case 1:
            return "PRESENTE";

        default:
            return "DESCONOCIDA";
    }
}

static const char *Maestro_Nombre_Estado_Cinta(uint8_t estado)
{
    switch (estado)
    {
        case 0:
            return "DESACTIVADA";

        case 1:
            return "CONFIGURANDO";

        case 2:
            return "ACTIVADA";

        case 3:
            return "ERROR";

        default:
            return "ESTADO_DESCONOCIDO";
    }
}

static const char *Maestro_Nombre_Subestado_Cinta(uint8_t subestado)
{
    switch (subestado)
    {
        case 0:
            return "ANDANDO";

        case 1:
            return "PESANDO";

        case 2:
            return "DETENIDA";

        case 3:
            return "ESPERANDO_REINICIO";

        case 4:
            return "ACELERANDO";

        case 5:
            return "DESACELERANDO";

        case 6:
            return "ESPERANDO_MAESTRO";

        default:
            return "SUBESTADO_DESCONOCIDO";
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        char dato = (char)pc_rx_byte;

        if (dato == ':') //si llega ':'
        {
            pc_cmd_indice = 0;
            pc_recibiendo = 1;
            pc_comando_listo = 0;
            pc_cmd_buffer[0] = '\0';  //no guardamos los ':'
        }
        else if (pc_recibiendo == 1)
        {
            if ((dato == '\r') || (dato == '\n'))
            {
                pc_cmd_buffer[pc_cmd_indice] = '\0';
                pc_recibiendo = 0;
                pc_comando_listo = 1;
            }
            else if (pc_cmd_indice < (PC_CMD_BUFFER_SIZE - 1)) //dejamos un lugar para guardar el '\0'
            {
                pc_cmd_buffer[pc_cmd_indice++] = dato;
            }
            else
            {
                pc_cmd_indice = 0;
                pc_recibiendo = 0;
                pc_comando_listo = 0;
            }
        }

        (void)HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1);  //volvemos a activar para prcesar el siguiente caracter
    }
    else if (huart->Instance == USART3)
    {
        RS485_Rx_Callback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        __HAL_UNLOCK(huart);

        pc_recibiendo = 0U;
        pc_cmd_indice = 0U;

        (void)HAL_UART_Receive_IT(&huart1, &pc_rx_byte, 1U);
    }
    else if (huart->Instance == USART3)
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
