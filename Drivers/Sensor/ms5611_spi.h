/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#pragma once

///////////////////////////////////////////////////////////////////////////////
extern float Temperature, Altitude, Pressure;

extern uint32_t d1;

extern uint32_t d1Value;

extern uint32_t d2;

extern uint32_t d2Value;

extern int32_t ms5611Temperature;

extern uint8_t newPressureReading;

extern uint8_t newTemperatureReading;

#define BARO_CS_LOW       HAL_GPIO_WritePin(GPIOE, BARO_CS_Pin, GPIO_PIN_RESET)
#define BARO_CS_HIGH      HAL_GPIO_WritePin(GPIOE, BARO_CS_Pin, GPIO_PIN_SET)
///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperatureRequestPressure(void);

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestPressure(void);

///////////////////////////////////////////////////////////////////////////////
// Read Pressure Request Temperature
///////////////////////////////////////////////////////////////////////////////

void readPressureRequestTemperature(void);

///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

void calculateTemperature(void);

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(void);

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

void readMag(void *unused);

///////////////////////////////////////////////////////////////////////////////
