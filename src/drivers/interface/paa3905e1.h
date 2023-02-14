/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * pmw3901.h - Header file for pmw3901 driver
 */

#ifndef PAA3905_H_
#define PAA3905_H_

#include <stdint.h>
#include "deck.h"

#define BRIGHT_MODE                       0 
#define LOW_LIGHT_MODE                    1
#define SUPER_LOW_LIGHT_MODE              2

#define BRIGHT_MODE_MIN_SQUAL             0x19
#define LOW_LIGHT_MODE_MIN_SQUAL          0x46
#define SUPER_LOW_LIGHT_MODE_MIN_SQUAL    0x55

#define BRIGHT_MODE_MAX_SHUTTER           0x00FF80
#define LOW_LIGHT_MODE_MAX_SHUTTER        0x00FF80
#define SUPER_LOW_LIGHT_MODE_MAX_SHUTTER  0x025998

typedef struct motionBurst3905_s {
  union {
    uint8_t motion;
    struct {
      uint8_t frameFrom0    : 1;
      uint8_t runMode       : 2;
      uint8_t reserved1     : 1;
      uint8_t rawFrom0      : 1;
      uint8_t reserved2     : 2;
      uint8_t motionOccured : 1;
    };
  };

  uint8_t observation;
  int16_t deltaX;
  int16_t deltaY;

  uint8_t reserved;
  uint8_t squal;

  uint8_t rawDataSum;
  uint8_t maxRawData;
  uint8_t minRawData;

  uint8_t shutter_h;
  uint8_t shutter_m;
  uint8_t shutter_l;
} __attribute__((packed)) motionBurst3905_t;

/**
 * Initialize the PMW3901 sensor
 *
 * @param csPin Chip Select pin as defined in deck pinout driver.
 *
 * @return  true if init successful else false.
 */
bool paa3905Init(const deckPin_t csPin);

/**
 * Read the current accumulated motion.
 *
 * @param csPin   Chip Select pin as defined in deck pinout driver.
 * @param motion  A filled in motionBurst_t structure with the latest motion information
 */
void paa3905ReadMotion(const deckPin_t csPin, motionBurst3905_t * motion);

float paa3905ReadRaw(const deckPin_t csPin);


#endif /* PMW3901_H_ */