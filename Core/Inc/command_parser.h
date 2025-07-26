#ifndef __COMMAND_PARSER_H
#define __COMMAND_PARSER_H

#include <stdint.h>
#include <stdbool.h>

// Funciones públicas
void command_parser_process_esp01(uint8_t byte);
void command_parser_process_debug(uint8_t byte);

#endif // __COMMAND_PARSER_H