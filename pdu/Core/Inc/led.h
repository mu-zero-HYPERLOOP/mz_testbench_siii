/*
 * led.h
 *
 *  Created on: Nov 17, 2020
 *      Author: Felix
 */

#ifndef SRC_LED_H_
#define SRC_LED_H_

#ifdef __cplusplus
extern "C" {
#endif

void LED_RGB_Init();
void LED_RGB_Write(uint8_t green, uint8_t orange, uint8_t red);

void LED_Green_Write(uint8_t green);
void LED_Orange_Write(uint8_t orange);
void LED_Red_Write(uint8_t red);


#ifdef __cplusplus
}
#endif

#endif /* SRC_LED_H_ */
