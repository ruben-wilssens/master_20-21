/*
 * library.h
 *
 *  Created on: Mar 15, 2020
 *      Author: jelle
 */

#ifndef INC_CBUF_H_
#define INC_CBUF_H_

#ifdef __cplusplus
extern C {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdbool.h"
#include "assert.h"
#include "stm32f4xx_hal.h"
#include "main.h"

/* Defines -------------------------------------------------------------------*/
typedef struct circular_buf_t circular_buf_t;
typedef circular_buf_t* cbuf_handle_t;

/* Functions -----------------------------------------------------------------*/
cbuf_handle_t circular_buf_init(uint16_t* buffer, size_t size);
void circular_buf_free(cbuf_handle_t cbuf);
void circular_buf_reset(cbuf_handle_t cbuf);
void circular_buf_put_overwrite(cbuf_handle_t cbuf, uint16_t data);
int circular_buf_put_reject(cbuf_handle_t cbuf, uint16_t data);
int circular_buf_get(cbuf_handle_t cbuf, uint16_t * data);
bool circular_buf_empty(cbuf_handle_t cbuf);
bool circular_buf_full(cbuf_handle_t cbuf);
size_t circular_buf_capacity(cbuf_handle_t cbuf);
size_t circular_buf_size(cbuf_handle_t cbuf);

#ifdef __cplusplus
}
#endif

#endif /* INC_CBUF_H_ */
