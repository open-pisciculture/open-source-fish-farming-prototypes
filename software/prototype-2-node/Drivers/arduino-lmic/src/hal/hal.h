/*******************************************************************************
 * Copyright (c) 2015-2016 Matthijs Kooijman
 * Copyright (c) 2016-2018 MCCI Corporation
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/
#ifndef _hal_hal_h_
#define _hal_hal_h_

#ifdef __cplusplus
extern "C" {
#endif

#include "arduino_lmic_hal_configuration.h"

struct hal_s {
	uint8_t overflow;
};

// interrupt handlers, to be called in ISR
void hal_isrPin0(void);
void hal_isrPin1(void);
void hal_isrPin2(void);

struct hal_s hal_get_persistence(void);
uint8_t hal_get_overflow();
void hal_set_persistence(struct hal_s hal_inst);
void hal_set_overflow(uint8_t overflow_inst);

#ifdef __cplusplus
}
#endif

#endif // _hal_hal_h_
