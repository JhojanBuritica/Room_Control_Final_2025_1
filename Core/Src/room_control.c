#include "room_control.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include "led.h"

// Contraseña por defecto
static const char DEFAULT_PASSWORD[] = "1234";

// Parpadeo rápido del LED LD2 (6 veces)
static void blink_alarm_led(void) {
    extern led_handle_t heartbeat_led; // Usa el handle global definido en main.c
    for (int i = 0; i < 6; i++) {
        led_on(&heartbeat_led);
        HAL_Delay(100);
        led_off(&heartbeat_led);
        HAL_Delay(100);
    }
}


// Umbrales de temperatura 
static const float TEMP_THRESHOLD_LOW = 25.0f;
static const float TEMP_THRESHOLD_MED = 28.0f;
static const float TEMP_THRESHOLD_HIGH = 31.0f;

// Tiempo de espera para input y acceso denegado
static const uint32_t INPUT_TIMEOUT_MS = 10000;  // 10 segundos
static const uint32_t ACCESS_DENIED_TIMEOUT_MS = 3000;  // 3 segundos

// Callback para mensajes seriales
static room_control_serial_callback_t serial_callback = NULL;

// Private function prototypes
static void room_control_change_state(room_control_t *room, room_state_t new_state);
static void room_control_update_display(room_control_t *room);
static void room_control_update_door(room_control_t *room);
static void room_control_update_fan(room_control_t *room);
static fan_level_t room_control_calculate_fan_level(float temperature);
static void room_control_clear_input(room_control_t *room);

void room_control_set_serial_callback(room_control_serial_callback_t cb) {
    serial_callback = cb;
}

void room_control_init(room_control_t *room) {
    // Control de habitacion 
    room->current_state = ROOM_STATE_LOCKED;
    strcpy(room->password, DEFAULT_PASSWORD);
    room_control_clear_input(room);
    room->last_input_time = 0;
    room->state_enter_time = HAL_GetTick();

    // Initialize door control
    room->door_locked = true;

    // Initialize temperature and fan
    room->current_temperature = 22.0f;  // Temperatura de habitacion por defecto
    room->current_fan_level = FAN_LEVEL_OFF;
    room->manual_fan_override = false;

    // Display
    room->display_update_needed = true;
}

void room_control_update(room_control_t *room) {
    uint32_t current_time = HAL_GetTick();

    // State machine
    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            // Espera tecla
            break;

        case ROOM_STATE_INPUT_PASSWORD:
            // Timeout: volver a LOCKED después de 10 segundos sin input
            if (current_time - room->last_input_time > INPUT_TIMEOUT_MS) {
                if (serial_callback) serial_callback("Timeout. Sistema bloqueado\r\n");
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_UNLOCKED:
            room->door_locked = false;
            room->manual_fan_override = false;  // Reset manual override
            room_control_update_fan(room); // fuerza actualización PWM si hace falta
            room_control_update_display(room); // refresca display con valores actualizados
            room->display_update_needed = false; // ya se actualizó el display
            break;

        case ROOM_STATE_ACCESS_DENIED:
            // Espera 3 segundos y vuelve a LOCKED
            if (current_time - room->state_enter_time > ACCESS_DENIED_TIMEOUT_MS) {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        case ROOM_STATE_EMERGENCY:
            // Lógica de emergencia (opcional)
            break;
    }

    // Update subsystems
    room_control_update_door(room);
    room_control_update_fan(room);

    if (room->display_update_needed) {
        room_control_update_display(room);
        room->display_update_needed = false;
    }
}

void room_control_process_key(room_control_t *room, char key) {
    room->last_input_time = HAL_GetTick();

    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            // Start password input
            room_control_clear_input(room);
            room->input_buffer[0] = key;
            room->input_index = 1;
            room_control_change_state(room, ROOM_STATE_INPUT_PASSWORD);
            // Mostrar primer asterisco en serial
            if (serial_callback) serial_callback("*\r\n");
            break;

        case ROOM_STATE_INPUT_PASSWORD:
            if (room->input_index < PASSWORD_LENGTH) {
                room->input_buffer[room->input_index++] = key;
            }
            // Mostrar asteriscos en serial
            if (serial_callback) {
                char ast[PASSWORD_LENGTH + 3];
                memset(ast, '*', room->input_index);
                ast[room->input_index] = '\r';
                ast[room->input_index + 1] = '\n';
                ast[room->input_index + 2] = 0;
                serial_callback(ast);
            }
            if (room->input_index == PASSWORD_LENGTH) {
                room->input_buffer[PASSWORD_LENGTH] = 0;
                if (strcmp(room->input_buffer, room->password) == 0) {
                    room_control_change_state(room, ROOM_STATE_UNLOCKED);
                    if (serial_callback) serial_callback("ACCESO CONCEDIDO\r\n");
                } else {
                    blink_alarm_led(); // Parpadeo rápido del LED LD2
                    room_control_change_state(room, ROOM_STATE_ACCESS_DENIED);
                    if (serial_callback) serial_callback("ACCESO DENEGADO\r\n");
                }
            }
            break;

        case ROOM_STATE_UNLOCKED:
            // Volver a estado bloqueado si se presiona '*'
            if (key == '*') {
                room_control_change_state(room, ROOM_STATE_LOCKED);
            }
            break;

        default:
            break;
    }

    room->display_update_needed = true;
}

void room_control_set_temperature(room_control_t *room, float temperature) {
    room->current_temperature = temperature;

    // Update fan level automatically if not in manual override
    if (!room->manual_fan_override) {
        fan_level_t new_level = room_control_calculate_fan_level(temperature);
        if (new_level != room->current_fan_level) {
            room->current_fan_level = new_level;
            room->display_update_needed = true;
        }
    } else {
        room->display_update_needed = true;
    }
}

void room_control_force_fan_level(room_control_t *room, fan_level_t level) {
    room->manual_fan_override = true;
    room->current_fan_level = level;
    room->display_update_needed = true;
}

void room_control_change_password(room_control_t *room, const char *new_password) {
    if (strlen(new_password) == PASSWORD_LENGTH) {
        strcpy(room->password, new_password);
    }
}

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
static void room_control_change_state(room_control_t *room, room_state_t new_state) {
    room->current_state = new_state;
    room->state_enter_time = HAL_GetTick();
    room->display_update_needed = true;


    // State entry actions
    switch (new_state) {
        case ROOM_STATE_LOCKED:
            room->door_locked = true;
            room_control_clear_input(room);
            extern led_handle_t heartbeat_led;
            led_off(&heartbeat_led); // Apaga el LED al bloquear
            break;

        case ROOM_STATE_UNLOCKED:
            room->door_locked = false;
            room->manual_fan_override = false;  // Reset manual override
            break;

        case ROOM_STATE_ACCESS_DENIED:
            room_control_clear_input(room);
            break;

        default:
            break;
    }
}

static void room_control_update_display(room_control_t *room) {
    char display_buffer[32];

    ssd1306_Fill(Black);

    switch (room->current_state) {
        case ROOM_STATE_LOCKED:
            ssd1306_SetCursor(20, 10);
            ssd1306_WriteString("Bienvenido !!", Font_7x10, White);
            ssd1306_SetCursor(10, 25);
            ssd1306_WriteString("El sistema esta", Font_7x10, White);
            ssd1306_SetCursor(30, 40);
            ssd1306_WriteString("BLOQUEADO", Font_7x10, White);
            break;

        case ROOM_STATE_INPUT_PASSWORD:
            ssd1306_SetCursor(10, 10);
            ssd1306_WriteString("CLAVE:", Font_7x10, White);
            ssd1306_SetCursor(60, 10);
            for (uint8_t i = 0; i < room->input_index; i++) {
                ssd1306_WriteString("*", Font_7x10, White);
            }
            break;

        case ROOM_STATE_UNLOCKED:
            ssd1306_SetCursor(30, 10);
            ssd1306_WriteString("ACCESO OK", Font_7x10, White);

            // Conversion para mostrar 1 decimal SIEMPRE, incluso si tu printf no soporta %.1f
            int temp_int = (int) room->current_temperature;
            int temp_dec = (int) ((room->current_temperature - temp_int) * 10);
            if (temp_dec < 0) temp_dec = -temp_dec; // Evita signo negativo en decimal

            snprintf(display_buffer, sizeof(display_buffer), "Temp: %d.%d C", temp_int, temp_dec);
            ssd1306_SetCursor(10, 25);
            ssd1306_WriteString(display_buffer, Font_7x10, White);

            snprintf(display_buffer, sizeof(display_buffer), "Fan: %d%%", room->current_fan_level);
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

static void room_control_update_door(room_control_t *room) {
    // Control físico de la puerta
    if (room->door_locked) {
        HAL_GPIO_WritePin(DOOR_STATUS_GPIO_Port, DOOR_STATUS_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(DOOR_STATUS_GPIO_Port, DOOR_STATUS_Pin, GPIO_PIN_SET);
    }
}

extern TIM_HandleTypeDef htim3;

static void room_control_update_fan(room_control_t *room) {
    // Control PWM del ventilador (PA6, TIM3 CH1)
    // Periodo = 100
    uint32_t pwm_value = room->current_fan_level; // PWM DIRECTO
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
}

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

static void room_control_clear_input(room_control_t *room) {
    memset(room->input_buffer, 0, sizeof(room->input_buffer));
    room->input_index = 0;
}
