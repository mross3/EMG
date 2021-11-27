/**
 * \file CoAmpCAN.h
 * \brief Library for Co-Amp CAN message communication.
 * TODO: Should be an interface implemented as an abstract class.
 *
 * \author Nathan Brantly
 * \bug
 */

#ifndef COAMPCAN_H
#define COAMPCAN_H
/*
// INCLUDES
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif
*/
#include <CAN.h>
//#include <SPI.h>          // Required to resolve #define conflicts

// CONSTANTS/MACROS
#define NUM_EMG_CHANNELS    8
#define DEFAULT_COAMP_GAIN	8
#define COAMP_ONLY 			0
#define CONTROLLER_CONNECTED 1
#define COAMP_FRAME_INCREMENT 50

// TYPES

// CLASSES

// FUNCTION PROTOTYPES
void CoAmpInitialize( uint8_t controllerConnected, uint8_t allChannelsGain);
void TransmitGainCANMsg( uint8_t allChannelsGain );
void TransmitStartCANMsg( void );
uint8_t CoAmpCANAvailable( uint16_t *a_EMGSample, uint8_t *p_numEMGSamples );

#endif /* COAMPCAN_H */