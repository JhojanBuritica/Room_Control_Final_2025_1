#include "command_parser.h"
#include <string.h>
#include <stdio.h>
#include "room_control.h"
#include "main.h"

// Tamaño máximo del buffer para comandos recibidos
#define COMMAND_BUFFER_SIZE 64

// Buffer para almacenar los caracteres del comando recibido desde el ESP-01
static char esp01_buffer[COMMAND_BUFFER_SIZE];
static uint8_t esp01_index = 0;

// Buffer para comandos recibidos desde el puerto UART de depuración
static char debug_buffer[COMMAND_BUFFER_SIZE];
static uint8_t debug_index = 0;

// UARTs declaradas externamente (configuradas en main.c)
extern UART_HandleTypeDef huart2; // UART para depuración local
extern UART_HandleTypeDef huart3; // UART conectada al ESP-01

// Instancia global del sistema de control de habitación
extern room_control_t room_system;

// Prototipos de funciones internas
static void process_command(const char *cmd, UART_HandleTypeDef *source_uart);
static void send_response(UART_HandleTypeDef *uart, const char *msg);

// Función principal del parser (para ESP-01)

/**
 * @brief Procesa byte a byte los datos recibidos desde el ESP-01.
 *      If it detects the end of a line, it interprets the entire command.
 */
void command_parser_process_esp01(uint8_t byte) {
    if (byte == '\n' || byte == '\r') {
        if (esp01_index > 0) {
            esp01_buffer[esp01_index] = '\0'; // Finaliza la cadena
            process_command(esp01_buffer, &huart3);
            esp01_index = 0; // Reinicia el índice para el próximo comando
        }
    } else if (esp01_index < COMMAND_BUFFER_SIZE - 1) {
        esp01_buffer[esp01_index++] = byte; // Almacena el byte recibido
    }
}

// Función principal del parser (para debug UART)
/**
 * @brief Procesa byte a byte los datos recibidos desde UART2 (depuración).
 *        Similar a ESP-01 pero por otro canal UART.
 */
void command_parser_process_debug(uint8_t byte) {
    if (byte == '\n' || byte == '\r') {
        if (debug_index > 0) {
            debug_buffer[debug_index] = '\0';
            process_command(debug_buffer, &huart2);
            debug_index = 0;
        }
    } else if (debug_index < COMMAND_BUFFER_SIZE - 1) {
        debug_buffer[debug_index++] = byte;
    }
}

// Envía una línea terminada en \r\n
/**
 * @brief Envía una respuesta al cliente, agregando terminación de línea estándar (\r\n).
 * @param uart UART por donde se debe enviar la respuesta
 * @param msg  Cadena de texto a enviar
 */
static void send_response(UART_HandleTypeDef *uart, const char *msg) {
    HAL_UART_Transmit(uart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_UART_Transmit(uart, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}

// Parser real de comandos
/**
 * @brief Analiza el comando recibido y ejecuta la acción correspondiente.
 * @param cmd Cadena con el comando a interpretar
 * @param source_uart UART desde donde se recibió el comando (para responder por el mismo canal)
 */
static void process_command(const char *cmd, UART_HandleTypeDef *source_uart) {
    if (strncmp(cmd, "GET_TEMP", 8) == 0) {
        // Devuelve la temperatura como un entero redondeado
        int temp = (int)(room_control_get_temperature(&room_system) + 0.5f);
        char msg[32];
        snprintf(msg, sizeof(msg), "TEMP: %d C", temp);
        send_response(source_uart, msg);

    } else if (strncmp(cmd, "GET_STATUS", 10) == 0) {
        // Devuelve el estado actual y nivel del ventilador
        room_state_t state = room_control_get_state(&room_system);
        fan_level_t fan = room_control_get_fan_level(&room_system);
        char msg[64];
        snprintf(msg, sizeof(msg), "STATUS: %s, FAN=%d",
                 state == ROOM_STATE_LOCKED ? "LOCKED" :
                 state == ROOM_STATE_UNLOCKED ? "UNLOCKED" :
                 state == ROOM_STATE_UNLOCKED ? "ACCESS_GRANTED" :
                 state == ROOM_STATE_ACCESS_DENIED ? "ACCESS_DENIED" : "UNKNOWN",
                 fan);
        send_response(source_uart, msg);

    } else if (strncmp(cmd, "SET_PASS:", 9) == 0) {
        // Cambia la contraseña del sistema si es válida (4 caracteres)
        const char *new_pass = cmd + 9;
        if (strlen(new_pass) == 4) {
            room_control_change_password(&room_system, new_pass);
            send_response(source_uart, "Password changed");
        } else {
            send_response(source_uart, "Invalid password format");
        }

    } else if (strncmp(cmd, "FORCE_FAN:", 10) == 0) {
        // Fuerza el nivel del ventilador manualmente (0-3)
        int level = cmd[10] - '0';
        if (level >= 0 && level <= 3) {
            room_control_force_fan_level(&room_system, (fan_level_t)level);
            send_response(source_uart, "Fan level forced");
        } else {
            send_response(source_uart, "Invalid fan level");
        }

    } else {
        // Comando desconocido
        send_response(source_uart, "Unknown command");
    }
}