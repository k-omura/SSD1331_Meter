/*
 * bitmap.h
 *
 *  Created on: 2020/05/04
 *      Author: k-omura
 */

#ifndef SRC_BITMAP_H_
#define SRC_BITMAP_H_

#define OLED_HEIGHT2 64

#include "stdio.h"

void addPixel(int, int, int, uint8_t *_bitmap);
void addLine(int, int, int, int, int, uint8_t *_bitmap);
void drawLinerMode(int16_t, int16_t, int16_t, uint8_t , uint8_t *_bitmap);
void drawCircleMode(int16_t, int16_t, int16_t, uint8_t , uint8_t *_bitmap);
void stringBitmap(uint8_t, uint8_t, const char _character[], uint8_t, uint8_t *_bitmap, uint8_t);
void characterBitmap8(uint8_t, uint8_t, char, uint8_t *_bitmap, uint8_t);
void characterBitmap5(uint8_t, uint8_t, char, uint8_t *_bitmap, uint8_t);

//---------------------------------
extern const unsigned char FONT8x8[][8];
extern const unsigned char FONT5x3[][2];

#endif /* SRC_BITMAP_H_ */
