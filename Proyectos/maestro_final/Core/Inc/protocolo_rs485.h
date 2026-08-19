/*
 * protocolo_rs485.h
 *
 * Comandos compartidos por maestro, tanque y cinta.
 */

#ifndef PROTOCOLO_RS485_H
#define PROTOCOLO_RS485_H

#include <stdint.h>

/* Comandos generales. */
#define CMD_PING                         0x01U
#define CMD_PONG                         0x81U

/* Consultas del tanque/dosificador. */
#define CMD_PEDIR_ESTADO_TANQUE          0x10U
#define CMD_PEDIR_ESTADO_DOSIF           0x11U

/* Escrituras en el tanque/dosificador. */
#define CMD_TANQUE_SETPOINT              0x12U
#define CMD_TANQUE_RECETA                0x13U
#define CMD_TANQUE_CONTROL_DOSIF         0x14U
#define CMD_TANQUE_RESET_ALARMA          0x15U
#define CMD_PEDIR_EVENTO_TANQUE          0x16U

/* Consulta de la cinta. */
#define CMD_PEDIR_ESTADO_CINTA           0x20U

/* Órdenes de la cinta. */
#define CMD_CINTA_CONFIGURAR             0x21U
#define CMD_CINTA_TARA                   0x22U
#define CMD_CINTA_CALIBRAR               0x23U
#define CMD_CINTA_ACTIVAR                0x24U
#define CMD_CINTA_CONTROL_BANDA          0x25U
#define CMD_CINTA_CONTINUAR_PESAJE       0x26U
#define CMD_PEDIR_EVENTO_CINTA           0x27U

/* Evento espontáneo de la cinta y su confirmación. */
#define CMD_EVENTO_CINTA_ESPERANDO_MAESTRO 0x70U
#define CMD_CONFIRMAR_EVENTO_CINTA          0x71U

/* Evento espontáneo del tanque al completar el llenado. */
#define CMD_EVENTO_TANQUE_LLENADO_COMPLETO  0x72U
#define CMD_CONFIRMAR_EVENTO_TANQUE         0x73U

/*
 * Evento espontáneo de la cinta al terminar el pesaje.
 * PARAM_H:PARAM_L transportan el peso en décimas de gramo.
 * Ejemplo: 1234 representa 123,4 g.
 */
#define CMD_EVENTO_CINTA_PESAJE_COMPLETO    0x74U
#define CMD_CONFIRMAR_EVENTO_CINTA_PESAJE   0x75U

#define CMD_RESP_SIN_EVENTO                 0x76U

/* Respuestas de estado. */
#define CMD_RESP_ESTADO_CINTA            0x80U
#define CMD_RESP_ESTADO_TANQUE           0x90U
#define CMD_RESP_ESTADO_DOSIF            0x91U

/* Confirmaciones de órdenes de escritura. */
#define CMD_RESP_ACK                     0xA0U
#define CMD_RESP_NACK                    0xA1U

/* Códigos positivos incluidos en un ACK. */
#define ACK_OK                           0x00U
#define ACK_ESPERANDO_BOTELLA            0x01U

/* Motivos incluidos en un NACK. */
#define NACK_PARAMETRO_INVALIDO          0x01U
#define NACK_FUERA_DE_RANGO              0x02U
#define NACK_OCUPADO                     0x03U
#define NACK_ALARMA_ACTIVA               0x04U
#define NACK_SIN_RECETA                  0x05U
#define NACK_RETIRE_BOTELLA              0x06U
#define NACK_COMANDO_NO_SOPORTADO        0x07U
#define NACK_ESTADO_INVALIDO             0x08U
#define NACK_SENSOR                      0x09U
#define NACK_SIN_TARA                    0x0AU
#define NACK_SIN_CONFIGURACION           0x0BU

#endif /* PROTOCOLO_RS485_H */
