#include <Arduino.h>
// INCLUDES
#include "CoAmpCAN.h"

//Defined counter and pointer based on CoAmpCANAvailiable function param declaration comments
uint8_t counter = 0;
uint8_t *p_numEMGSamples = &counter;
uint16_t *a_EMGSample = (uint16_t*) malloc(8*sizeof(uint16_t));

//Controller connected and allChannelsGain
uint8_t allChannelsGain = DEFAULT_COAMP_GAIN;
uint8_t controllerConnected = CONTROLLER_CONNECTED;
        

void setup() {

CoAmpInitialize(controllerConnected, allChannelsGain);

}

void loop() {
  // put your main code here, to run repeatedly:

 CoAmpCANAvailable( a_EMGSample,  p_numEMGSamples );

}

/**
 * \file        CoAmpCAN.cpp
 * \brief       Library for Co-Amp CAN message communication.
 *
 * \authors     Nathan Brantly, Frank Cummins
 * \bug
 */



// PRIVATE VARIABLES - NONE

// PRIVATE FUNCTION PROTOTYPES - NONE

// PUBLIC FUNCTION PROTOTYPES

/**
 * \brief High level method that initializes Co-Amp with default gain
 *
 * \param controllerConnected boolean to set gain and send start message when appropriate
 */
void CoAmpInitialize( uint8_t controllerConnected, uint8_t allChannelsGain)
{
        CAN.begin(CAN_BPS_1M);
        delay(2500);
        if(!controllerConnected)
        {
                TransmitGainCANMsg(allChannelsGain);
                TransmitStartCANMsg();
        }
}


/**
 * \brief Transmits gain CAN message to Co-Amp to set all channel gains
 *
 * \param allChannelsGain Gain value to set for all eight channels (i.e., 4, 6, 8, 10, or 12)
 */
void TransmitGainCANMsg( uint8_t allChannelsGain )
{
        // Construct the gain message (2 header messages + 8 channel messages)
        for ( uint8_t i = 0; i < NUM_EMG_CHANNELS + 2; i++ ) 
        {
                // Instantiate gain CAN message object to transmit to Co-Amp
                CAN_Frame gainCANMsg;

                gainCANMsg.id = 0x150;      // Message ID
                gainCANMsg.valid = true;
                gainCANMsg.rtr = 0;
                gainCANMsg.length = 5;      // Length

                // Set each data byte in gain CAN message object
                gainCANMsg.data[0] = 0x04;

                switch(i)
                {
                        case 0:
                                gainCANMsg.data[1] = 0x02;
                                gainCANMsg.data[2] = 0xFE;
                                gainCANMsg.data[3] = 0x03;
                                gainCANMsg.data[4] = 0xE8;
                                break;
                
                        case 1:
                                gainCANMsg.data[1] = 0x03;
                                gainCANMsg.data[2] = 0xFE;
                                gainCANMsg.data[3] = 0x00;
                                gainCANMsg.data[4] = 0x10;
                                break;

                        default:
                                gainCANMsg.data[1] = 0x01;
                                gainCANMsg.data[2] = i - 1;             // Channel index
                                gainCANMsg.data[3] = 0x00;
                                gainCANMsg.data[4] = allChannelsGain;   // Channel gain
                                break;
                }

                // Transmit Co-Amp gain CAN message
                CAN.write( gainCANMsg );
        }
}

/**
 * \brief Transmit start CAN message to Co-Amp to begin streaming EMG data
 */
void TransmitStartCANMsg( void )
{
        // Instantiate start CAN message object to transmit to Co-Amp
        CAN_Frame startCANMsg; 

        startCANMsg.id = 0x150;         // Message ID
        startCANMsg.valid = true;
        startCANMsg.rtr = 0;
        startCANMsg.length = 2;         // Length
        startCANMsg.data[0] = 0x50;     // Start message
        startCANMsg.data[1] = 0x01;

        // Transmit Co-Amp start CAN message
        CAN.write( startCANMsg );
}

/**
 * \brief Receive CAN EMG messages from Co-Amp
 *
 * \param a_EMGSample Pointer to EMG sample array
 * \param p_numEMGSamps Pointer to number of EMG samples received counter to increment
 * \return 1 if all three EMG sample Co-Amp CAN messages received, 0 if not
 */
uint8_t CoAmpCANAvailable( uint16_t *a_EMGSample, uint8_t *p_numEMGSamples )
{
        uint8_t available = 0;

        // If there is a CAN message in the buffer
        if (CAN.available())
        {
                // Instantiate EMG CAN message object to read from Co-Amp
                CAN_Frame emgCANMsg;    

                // Read Co-Amp EMG CAN message
                emgCANMsg = CAN.read();

                // Load the message into the EMG sample array
                switch ( emgCANMsg.id )
                {
                case 0x451:
                        a_EMGSample[0] = ( (uint16_t)emgCANMsg.data[1] << 8 ) | emgCANMsg.data[2];
                        a_EMGSample[1] = ( (uint16_t)emgCANMsg.data[3] << 8 ) | emgCANMsg.data[4];
                        a_EMGSample[2] = ( (uint16_t)emgCANMsg.data[5] << 8 ) | emgCANMsg.data[6];
                        break;

                case 0x452:
                        a_EMGSample[3] = ( (uint16_t)emgCANMsg.data[1] << 8 ) | emgCANMsg.data[2];
                        a_EMGSample[4] = ( (uint16_t)emgCANMsg.data[3] << 8 ) | emgCANMsg.data[4];
                        a_EMGSample[5] = ( (uint16_t)emgCANMsg.data[5] << 8 ) | emgCANMsg.data[6];
                        break;

                case 0x453:
                        a_EMGSample[6] = ( (uint16_t)emgCANMsg.data[1] << 8 ) | emgCANMsg.data[2];
                        a_EMGSample[7] = ( (uint16_t)emgCANMsg.data[3] << 8 ) | emgCANMsg.data[4];

                        // Increment the counter for timing purposes
                        (*p_numEMGSamples)++;
                        
                        // Raise the available flag to indicate that a new packet has been received
                        available = 1;
                        break;
                }
        }

        return available;
}