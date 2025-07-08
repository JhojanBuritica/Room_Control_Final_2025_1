#include "keypad.h"

static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

void keypad_init(keypad_handle_t* keypad) 
{
    //Configurar los pines de las filas como salidas
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // Inicializar en alto
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = keypad->row_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad->row_ports[i], &GPIO_InitStruct);
    }

    //Configurar los pines de las columnas como entradas con pull-up
    for (int j = 0; j < KEYPAD_COLS; j++) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = keypad->col_pins[j];
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Configurar como interrupción por flanco de bajada
        GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up para evitar lecturas flotantes
        HAL_GPIO_Init(keypad->col_ports[j], &GPIO_InitStruct);
    }
}

char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
    char key_pressed = '\0';
    
    static uint32_t last_press_time = 0;
    if (HAL_GetTick() - last_press_time < 100) {
        // Debounce: si la última pulsación fue hace menos de 100 ms, ignorar
        return key_pressed;
    }
    last_press_time = HAL_GetTick();

    switch (col_pin) {
      case KEYPAD_C1_Pin:
      for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // Activar fila
            if (HAL_GPIO_ReadPin(keypad->col_ports[0], keypad->col_pins[0]) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][0];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET); // Desactivar fila
        }
        break;
      case KEYPAD_C2_Pin:
      for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // Activar fila
            if (HAL_GPIO_ReadPin(keypad->col_ports[1], keypad->col_pins[1]) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][1];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET); // Desactivar fila
        }
        break;
      case KEYPAD_C3_Pin:
      for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // Activar fila
            if (HAL_GPIO_ReadPin(keypad->col_ports[2], keypad->col_pins[2]) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][2];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET); // Desactivar fila
        }
        break;
      case KEYPAD_C4_Pin:
      for (int i = 0; i < KEYPAD_ROWS; i++) {
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET); // Activar fila
            if (HAL_GPIO_ReadPin(keypad->col_ports[3], keypad->col_pins[3]) == GPIO_PIN_RESET) {
                key_pressed = keypad_map[i][3];
            }
            HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET); // Desactivar fila
        }
        break;
    }
    keypad_init(keypad);

    return key_pressed;
}