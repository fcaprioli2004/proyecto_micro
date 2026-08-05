#include "rs485.h"

/* =========================================================
 * CONTEXTO DE HARDWARE
 * ========================================================= */

typedef struct
{
    UART_HandleTypeDef *uart;
    GPIO_TypeDef *ctrl_port;
    uint16_t ctrl_pin;
    uint8_t id_local;
    uint8_t inicializado;
} RS485_Context_t;

static RS485_Context_t rs485_ctx = {0};

/* =========================================================
 * BUFFERS Y ESTADO INTERNO
 * ========================================================= */

static uint8_t rs485_tx_buffer[RS485_FRAME_SIZE];
static uint8_t rs485_rx_buffer[RS485_FRAME_SIZE];
static uint8_t rs485_rx_byte = 0U;
static uint8_t rs485_rx_index = 0U;
static uint8_t rs485_recibiendo = 0U;

/* =========================================================
 * ÚLTIMO PAQUETE VÁLIDO RECIBIDO
 * ========================================================= */

volatile uint8_t rs485_paquete_listo = 0U;
volatile uint8_t rs485_rx_destino = 0U;
volatile uint8_t rs485_rx_origen = 0U;
volatile uint8_t rs485_rx_cmd = 0U;
volatile uint8_t rs485_rx_param_h = 0U;
volatile uint8_t rs485_rx_param_l = 0U;

/* =========================================================
 * FUNCIONES INTERNAS
 * ========================================================= */

static void RS485_Reset_Parser(void)
{
    rs485_rx_index = 0U;
    rs485_recibiendo = 0U;
}

/* CRC-8, polinomio 0x07, valor inicial 0x00. */
static uint8_t RS485_Calculate_CRC8(
    const uint8_t *datos,
    uint8_t longitud
)
{
    uint8_t crc = 0x00U;

    if (datos == NULL)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < longitud; i++)
    {
        crc ^= datos[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0x07U);
            }
            else
            {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

/* =========================================================
 * INICIALIZACIÓN
 * ========================================================= */

HAL_StatusTypeDef RS485_Init(
    UART_HandleTypeDef *huart,
    GPIO_TypeDef *ctrl_port,
    uint16_t ctrl_pin,
    uint8_t id_local
)
{
    if ((huart == NULL) || (ctrl_port == NULL) || (ctrl_pin == 0U))
    {
        return HAL_ERROR;
    }

    rs485_ctx.uart = huart;
    rs485_ctx.ctrl_port = ctrl_port;
    rs485_ctx.ctrl_pin = ctrl_pin;
    rs485_ctx.id_local = id_local;
    rs485_ctx.inicializado = 1U;

    RS485_Reset_Parser();
    rs485_paquete_listo = 0U;

    /* DE = 0 y /RE = 0: MAX485 en modo recepción. */
    HAL_GPIO_WritePin(
        rs485_ctx.ctrl_port,
        rs485_ctx.ctrl_pin,
        GPIO_PIN_RESET
    );

    return HAL_UART_Receive_IT(
        rs485_ctx.uart,
        &rs485_rx_byte,
        1U
    );
}

/* =========================================================
 * TRANSMISIÓN
 * ========================================================= */

HAL_StatusTypeDef RS485_Send_Packet(
    uint8_t id_destino,
    uint8_t comando,
    uint8_t param_h,
    uint8_t param_l
)
{
    HAL_StatusTypeDef estado;

    if ((rs485_ctx.inicializado == 0U) ||
        (rs485_ctx.uart == NULL) ||
        (rs485_ctx.ctrl_port == NULL))
    {
        return HAL_ERROR;
    }

    rs485_tx_buffer[0] = RS485_START_BYTE;
    rs485_tx_buffer[1] = id_destino;
    rs485_tx_buffer[2] = rs485_ctx.id_local;
    rs485_tx_buffer[3] = comando;
    rs485_tx_buffer[4] = param_h;
    rs485_tx_buffer[5] = param_l;

    /* CRC sobre DESTINO + ORIGEN + CMD + PARAM_H + PARAM_L. */
    rs485_tx_buffer[6] = RS485_Calculate_CRC8(
        &rs485_tx_buffer[1],
        5U
    );

    rs485_tx_buffer[7] = RS485_END_BYTE;

    /* DE = 1 y /RE = 1: transmitir. */
    HAL_GPIO_WritePin(
        rs485_ctx.ctrl_port,
        rs485_ctx.ctrl_pin,
        GPIO_PIN_SET
    );

    /* HAL_UART_Transmit espera a que termine el último byte. */
    estado = HAL_UART_Transmit(
        rs485_ctx.uart,
        rs485_tx_buffer,
        RS485_FRAME_SIZE,
        20U
    );

    /* Volver siempre a recepción. */
    HAL_GPIO_WritePin(
        rs485_ctx.ctrl_port,
        rs485_ctx.ctrl_pin,
        GPIO_PIN_RESET
    );

    return estado;
}

/* =========================================================
 * RECEPCIÓN POR INTERRUPCIÓN
 * ========================================================= */

void RS485_Rx_Callback(UART_HandleTypeDef *huart)
{
    if ((rs485_ctx.inicializado == 0U) ||
        (huart == NULL) ||
        (rs485_ctx.uart == NULL) ||
        (huart->Instance != rs485_ctx.uart->Instance))
    {
        return;
    }

    /* Fuera de una trama solo buscamos START. */
    if (rs485_recibiendo == 0U)
    {
        if (rs485_rx_byte == RS485_START_BYTE)
        {
            rs485_rx_index = 0U;
            rs485_rx_buffer[rs485_rx_index++] = rs485_rx_byte;
            rs485_recibiendo = 1U;
        }
    }
    else
    {
        /* Dentro de la trama, 0x3A y 0x0D son datos válidos. */
        if (rs485_rx_index < RS485_FRAME_SIZE)
        {
            rs485_rx_buffer[rs485_rx_index++] = rs485_rx_byte;
        }
        else
        {
            RS485_Reset_Parser();
        }

        if (rs485_rx_index >= RS485_FRAME_SIZE)
        {
            uint8_t crc_calculado;
            uint8_t destino;

            RS485_Reset_Parser();

            if ((rs485_rx_buffer[0] == RS485_START_BYTE) &&
                (rs485_rx_buffer[7] == RS485_END_BYTE))
            {
                crc_calculado = RS485_Calculate_CRC8(
                    &rs485_rx_buffer[1],
                    5U
                );

                if (crc_calculado == rs485_rx_buffer[6])
                {
                    destino = rs485_rx_buffer[1];

                    /* Solo publicar paquetes dirigidos a este nodo. */
                    if ((destino == rs485_ctx.id_local) ||
                        (destino == ID_BROADCAST))
                    {
                        /* No sobrescribir un paquete aún no procesado. */
                        if (rs485_paquete_listo == 0U)
                        {
                            rs485_rx_destino = destino;
                            rs485_rx_origen = rs485_rx_buffer[2];
                            rs485_rx_cmd = rs485_rx_buffer[3];
                            rs485_rx_param_h = rs485_rx_buffer[4];
                            rs485_rx_param_l = rs485_rx_buffer[5];

                            /* Debe escribirse al final. */
                            rs485_paquete_listo = 1U;
                        }
                    }
                }
            }
        }
    }

    /* Rearmar la recepción del próximo byte. */
    (void)HAL_UART_Receive_IT(
        rs485_ctx.uart,
        &rs485_rx_byte,
        1U
    );
}

/* =========================================================
 * RECUPERACIÓN ANTE ERRORES UART
 * ========================================================= */

void RS485_Error_Callback(UART_HandleTypeDef *huart)
{
    if ((rs485_ctx.inicializado == 0U) ||
        (huart == NULL) ||
        (rs485_ctx.uart == NULL) ||
        (huart->Instance != rs485_ctx.uart->Instance))
    {
        return;
    }

    RS485_Reset_Parser();

    /* En STM32F1, leer SR/DR limpia ORE, FE, NE y PE. */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* Recuperar la máquina de estados HAL para volver a escuchar. */
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_READY;
    __HAL_UNLOCK(huart);

    (void)HAL_UART_Receive_IT(
        rs485_ctx.uart,
        &rs485_rx_byte,
        1U
    );
}
