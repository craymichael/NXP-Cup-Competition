/* Copyright 2017 Carmichael, Lindberg
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* file: serial.h
 *
 * Initializes and communicates with UART0 using interrupts for serial
 * communication. Uses GPIO pins to communicate with BT module if specified
 * during compilation.
 */
#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "MK64F12.h"

#define BAUD_RATE     (9600u)      // Default baud rate 
#define UART_FLUSH_TX {while(!(UART0_S1 & UART_S1_TDRE_MASK));}
#define RX_BUF_SZ     (128u)

// Commands
#define CSTART  ("start")
#define CSTOP   ("stop")
#define CPID    ("pid")
#define CSPEEDS ("speed")
#define CBRAKE  ("brake")

void uart_init(void);

uint8_t uart_getchar(void);
void uart_putchar(uint8_t ch);

void uart_put(uint8_t* ptr_str);
void uart_get(uint8_t* ptr_str);
void putnumU(uint32_t i);
uint32_t uart_hasdata(void);

#endif
