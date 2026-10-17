/*
 * cs_struct.h
 *
 *  Created on: Jun 15, 2026
 *      Author: sil
 */

#ifndef INC_HELPERS_CS_STRUCT_H_
#define INC_HELPERS_CS_STRUCT_H_

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} CS_Pin_t;

#endif /* INC_HELPERS_CS_STRUCT_H_ */
