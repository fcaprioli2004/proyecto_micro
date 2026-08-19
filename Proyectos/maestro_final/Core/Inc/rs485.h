#ifndef INC_RS485_H_
#define INC_RS485_H_

#include "main.h"

/* =========================================================
 * DIRECCIONES DE LOS NODOS
 * ========================================================= */

#define ID_MAESTRO          0x00U
#define ID_CINTA            0x01U
#define ID_TANQUE           0x02U
#define ID_BROADCAST        0xFFU  //mensaje dirigido a todos los nodos

/* =========================================================
 * FORMATO DE LA TRAMA
 * =========================================================
 *
 * Byte 0: START
 * Byte 1: ID destino
 * Byte 2: ID origen
 * Byte 3: comando
 * Byte 4: parámetro alto
 * Byte 5: parámetro bajo
 * Byte 6: CRC-8
 * Byte 7: END
 */

//por ejemplo para enviar el 500 decimal = 0x01F4
/*PARAM_H = 0x01
PARAM_L = 0xF4*/

//trama completa: : DESTINO ORIGEN COMANDO PARAM_H PARAM_L CRC \r

#define RS485_FRAME_SIZE    8U   //trama de 8 bytes
#define RS485_START_BYTE    0x3AU   /* corresponde al ASCII ':'  */
#define RS485_END_BYTE      0x0DU   /* corresponde al ASCII '\r' */

extern volatile uint8_t rs485_paquete_listo;  // 0 = NO hay paquete nuevo, 1 = SI hay
extern volatile uint8_t rs485_rx_destino;
extern volatile uint8_t rs485_rx_origen;
extern volatile uint8_t rs485_rx_cmd;
extern volatile uint8_t rs485_rx_param_h;
extern volatile uint8_t rs485_rx_param_l;

/* =========================================================
 * API
 * ========================================================= */

HAL_StatusTypeDef RS485_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *ctrl_port, uint16_t ctrl_pin, uint8_t id_local);

HAL_StatusTypeDef RS485_Send_Packet(uint8_t id_destino, uint8_t comando, uint8_t param_h, uint8_t param_l);

void RS485_Rx_Callback(UART_HandleTypeDef *huart);
void RS485_Error_Callback(UART_HandleTypeDef *huart);

#endif /* INC_RS485_H_ */
