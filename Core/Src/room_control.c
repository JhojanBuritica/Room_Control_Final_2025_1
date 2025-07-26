#include "room_control.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "led.h" 

// Declaración de periféricos externos
extern TIM_HandleTypeDef htim3;
extern led_handle_t heartbeat_led; 
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;  
extern UART_HandleTypeDef huart2;
extern led_handle_t status_led;

// Default password
static const char DEFAULT_PASSWORD[] = "1234";

// Temperature thresholds for automatic fan control
static const float TEMP_THRESHOLD_LOW = 25.0f;
static const float TEMP_THRESHOLD_MED = 28.0f;  
static const float TEMP_THRESHOLD_HIGH = 31.0f;

// Timeouts in milliseconds
static const uint32_t INPUT_TIMEOUT_MS = 10000;  // 10 seconds
static const uint32_t ACCESS_DENIED_TIMEOUT_MS = 3000;  // 3 seconds

// Private function prototypes
static void room_control_change_state(room_control_t *room, room_state_t new_state);
static void room_control_update_display(room_control_t *room);
static void room_control_update_door(room_control_t *room);
static void room_control_update_fan(room_control_t *room);
static fan_level_t room_control_calculate_fan_level(float temperature);
static void room_control_clear_input(room_control_t *room);

// Inicializa los parámetros y estado inicial del sistema
void room_control_init(room_control_t *room) {
    // Initialize room control structure
    room->current_state = ROOM_STATE_LOCKED;
    strcpy(room->password, DEFAULT_PASSWORD);
    room_control_clear_input(room);
    room->last_input_time = 0;
    room->state_enter_time = HAL_GetTick();
    
    // Initialize door control
    room->door_locked = true;// Puerta cerrada
    
    // Initialize temperature and fan
    room->current_temperature = 22.0f;  // Default room temperature
    room->current_fan_level = FAN_LEVEL_OFF;
    room->manual_fan_override = false;
    
    // Display
    room->display_update_needed = true;
}

// Actualiza el estado general según el tiempo y la lógica de control
void room_control_update(room_control_t *room) {
    uint32_t current_time = HAL_GetTick();

    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            // Mostrar mensaje y asegurar puerta cerrada
            room->door_locked = true;
            // Espera a que se presione una tecla (se maneja en process_key)
            break;

        case ROOM_STATE_INPUT_PASSWORD:
            // Timeout: volver a LOCKED si no hay input en 10s
            if (current_time - room->last_input_time > INPUT_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_UNLOCKED:
            room->door_locked = false;
            break;

        case ROOM_STATE_ACCESS_DENIED:
            // Esperar 3 segundos y volver a LOCKED
            if (current_time - room->state_enter_time > ACCESS_DENIED_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_EMERGENCY:
            // Lógica de emergencia (opcional)
            break;
    }

    room_control_update_door(room);
    room_control_update_fan(room);

    if (room->display_update_needed) {
        room_control_update_display(room);
        room->display_update_needed = false;
    }
}

// Procesa una tecla según el estado actual
void room_control_process_key(room_control_t *room, char key) {
    room->last_input_time = HAL_GetTick();

    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            // Iniciar ingreso de contraseña
            room_control_clear_input(room);
            if (room->input_index < PASSWORD_LENGTH && key >= '0' && key <= '9') {
                room->input_buffer[room->input_index++] = key;
                room_control_change_state(room, ROOM_STATE_INPUT_PASSWORD);
            }
            break;

        case ROOM_STATE_INPUT_PASSWORD:
            if (key >= '0' && key <= '9' && room->input_index < PASSWORD_LENGTH) {
                room->input_buffer[room->input_index++] = key;
            }
            // Si ya se ingresaron 4 dígitos, validar
            if (room->input_index == PASSWORD_LENGTH) {
                if (strncmp(room->input_buffer, room->password, PASSWORD_LENGTH) == 0) {
                    room_control_change_state(room, ROOM_STATE_UNLOCKED);
                } else {
                    room_control_change_state(room, ROOM_STATE_ACCESS_DENIED);
                }
            }
            break;

        case ROOM_STATE_UNLOCKED:
            // Permitir volver a LOCKED con '*'
            if (key == '*') {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        default:
            break;
    }

    room->display_update_needed = true;
}

// Asigna temperatura leída y actualiza el nivel de ventilador si está en modo a
void room_control_set_temperature(room_control_t *room, float temperature) {
    room->current_temperature = temperature;
    
    // Update fan level automatically if not in manual override
    if (!room->manual_fan_override) {
        fan_level_t new_level = room_control_calculate_fan_level(temperature);
        if (new_level != room->current_fan_level) {
            room->current_fan_level = new_level;
            room->display_update_needed = true;
        }
    }
    room->display_update_needed = true;
}

// Fuerza un nivel de ventilador manualmente (sobrescribe automático)
void room_control_force_fan_level(room_control_t *room, fan_level_t level) {
    if (level != FAN_LEVEL_OFF && level != FAN_LEVEL_LOW &&
        level != FAN_LEVEL_MED && level != FAN_LEVEL_HIGH) {
        return;
    }

    room->manual_fan_override = true;
    room->current_fan_level = level;

    room_control_update_fan(room);          
    room->display_update_needed = true;
}

// Cambia la contraseña si tiene la longitud correcta
void room_control_change_password(room_control_t *room, const char *new_password) {
    if (strlen(new_password) == PASSWORD_LENGTH) {
        strcpy(room->password, new_password);
    }
}

// Funciones de acceso (getters)
// Status getters
room_state_t room_control_get_state(room_control_t *room) {
    return room->current_state;
}

bool room_control_is_door_locked(room_control_t *room) {
    return room->door_locked;
}

fan_level_t room_control_get_fan_level(room_control_t *room) {
    return room->current_fan_level;
}

float room_control_get_temperature(room_control_t *room) {
    return room->current_temperature;
}

// Private functions
// Cambia de estado y actualiza LED, mensajes, etc.
static void room_control_change_state(room_control_t *room, room_state_t new_state) {
    room->current_state = new_state;
    room->state_enter_time = HAL_GetTick();
    room->display_update_needed = true;
    
    // State entry actions
    switch (new_state) {
        case ROOM_STATE_LOCKED:
            room->door_locked = true;
            room_control_clear_input(room);
            led_off(&status_led); //APAGA EL LED de estado
            break;
            
        case ROOM_STATE_UNLOCKED:
            room->door_locked = false;
            room->manual_fan_override = false;  // Reset manual override
            led_on(&status_led); //ENCIENDE EL LED de estado
            break;
            
        case ROOM_STATE_ACCESS_DENIED:
            room_control_clear_input(room);
            led_off(&heartbeat_led); //APAGA EL LED

            char alert_msg[] = "POST /alert HTTP/1.1\r\n"
                           "Host: mi-servidor.com\r\n"
                           "\r\nAcceso denegado detectado\r\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)alert_msg, strlen(alert_msg), 1000);
            break;
            
        default:
            break;
    }
}

// Convierte el nivel de ventilador a porcentaje para mostrar
static uint8_t fan_level_to_percent(fan_level_t level) {
    switch (level) {
        case FAN_LEVEL_LOW: return 33;
        case FAN_LEVEL_MED: return 66;
        case FAN_LEVEL_HIGH: return 99;
        case FAN_LEVEL_OFF:
        default: return 0;
    }
}

// Muestra estado, temperatura y ventilador en pantalla OLED
static void room_control_update_display(room_control_t *room) {
    char display_buffer[32];

    ssd1306_Fill(Black);

    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("SISTEMA", Font_7x10, White);
            ssd1306_SetCursor(10, 25);
            ssd1306_WriteString("BLOQUEADO", Font_7x10, White);
            break;

        case ROOM_STATE_INPUT_PASSWORD:
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("CLAVE:", Font_7x10, White);
            ssd1306_SetCursor(10, 25);
            // Mostrar asteriscos según input_index
            for (uint8_t i = 0; i < room->input_index; i++) {
                ssd1306_WriteString("*", Font_7x10, White);
            }
            break;

        case ROOM_STATE_UNLOCKED:
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("ACCESO OK", Font_7x10, White);

            //Mostrar temperatura
            snprintf(display_buffer, sizeof(display_buffer), "Temp: %d C", (int)(room->current_temperature));
            ssd1306_SetCursor(10, 25);
            ssd1306_WriteString(display_buffer, Font_7x10, White);

            snprintf(display_buffer, sizeof(display_buffer), "Fan: %d%%", fan_level_to_percent(room->current_fan_level));
            ssd1306_SetCursor(10, 40);
            ssd1306_WriteString(display_buffer, Font_7x10, White);
            break;
        

        case ROOM_STATE_ACCESS_DENIED:
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("ACCESO", Font_7x10, White);
            ssd1306_SetCursor(10, 25);
            ssd1306_WriteString("DENEGADO", Font_7x10, White);
            break;

        default:
            break;
    }

    ssd1306_UpdateScreen();
}

// Control físico de la puerta
static void room_control_update_door(room_control_t *room) {
    // Ejemplo usando el pin DOOR_STATUS:
    if (room->door_locked) {
        // HAL_GPIO_WritePin(DOOR_STATUS_GPIO_Port, DOOR_STATUS_Pin, GPIO_PIN_RESET);
    } else {
        // HAL_GPIO_WritePin(DOOR_STATUS_GPIO_Port, DOOR_STATUS_Pin, GPIO_PIN_SET);
    }
}

// Control del ventilador usando PWM
static void room_control_update_fan(room_control_t *room) {
    char msg[32];
    uint32_t pwm_value = 0;

    switch (room->current_fan_level) {
        case FAN_LEVEL_OFF:
            pwm_value = 0;
            break;
        case FAN_LEVEL_LOW:
            pwm_value = 30;  // directo, 30%
            break;
        case FAN_LEVEL_MED:
            pwm_value = 70;  // directo, 70%
            break;
        case FAN_LEVEL_HIGH:
            pwm_value = 99;  // 100%
            break;
        default:
            snprintf(msg, sizeof(msg), "Invalid fan level: %d\r\n", room->current_fan_level);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
            return;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
}

// Lógica de control automático de ventilador según temperatura
static fan_level_t room_control_calculate_fan_level(float temperature) {
    if (temperature < TEMP_THRESHOLD_LOW) {
        return FAN_LEVEL_OFF;
    } else if (temperature < TEMP_THRESHOLD_MED) {
        return FAN_LEVEL_LOW;
    } else if (temperature < TEMP_THRESHOLD_HIGH) {
        return FAN_LEVEL_MED;
    } else {
        return FAN_LEVEL_HIGH;
    }
}

// Limpia el buffer de entrada de clave
static void room_control_clear_input(room_control_t *room) {
    memset(room->input_buffer, 0, sizeof(room->input_buffer));
    room->input_index = 0;
}   