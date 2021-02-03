/*  Cheerson CX-10 integrated RF rate mode firmware.
     - Red PCB protocol implementation.

    This source contains components from Felix Niessen's original
    firmware (GPLv3), see the following thread for details:

    http://www.rcgroups.com/forums/showpost.php?p=30045580&postcount=1.

    The YD717 (Skywalker) protocol, and Beken 2423 initialisation
    routines were derived from the DeviationTX project (GPLv3), in
    particular the source file yd717_nrf24l01.c from:

    https://bitbucket.org/PhracturedBlue/

    All original components are provided under the GPLv3:

    Copyright (C) 2015, Samuel Powell.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*--------------------------------------------------------------------------*/
/*  Notes:                                                                  */
/*  The original nRF24 code should be functionally the same as found at     */
/*  https://github.com/bartslinger/cx10_fnrf site.                          */
/*                                                                          */
/*  The addition of the XN297 code is unique to this module and is taken    */
/*  from various other projects.                                            */
/*                                                                          */
/*                                                                          */
/*                                                                          */
/*--------------------------------------------------------------------------*/


#include "config.h"

extern uint8_t failsave;
bool bind = false;
extern int16_t RXcommands[6];

char rxbuffer[PAYLOADSIZE] = {0};

bool flashstate = false;
uint32_t flashtime;

/*--------------------------------------------------------------------------*/
/*  Dump all registers                                                      */
/*--------------------------------------------------------------------------*/
void dump_regs(void)
{
    for (int reg=0; reg <= REGS_END; reg++) {
        switch(reg) {
            case 0x18: // not a reg
            case 0x1A: // not a reg
            case 0x1B: // not a reg
                break;
            default:
                LOG(0, "reg[%02x] 0x%02x\n", reg, nrfRead1Reg(reg));
                break;
        }
    }
}

#if defined(RF_XN297)
/*--------------------------------------------------------------------------*/
/* Initialize XN297                                                         */
/*--------------------------------------------------------------------------*/
void rfchip_init(void)
{
    const char rf_addr_bind[5] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

    //
    // Initialize XN297 special regs
    //
    const unsigned char bb_cal[] = {0x4C, 0x84, 0x67, 0x9C, 0x20};
    const unsigned char rf_cal[] = {0xC9, 0x9A, 0xB0, 0x61, 0xBB, 0xAB, 0x9C};
    const unsigned char demod[]  = {0x0B, 0xDF, 0xC4, 0xA7, 0x03};

    nrfWriteReg(REG_BB_CAL, (char*) bb_cal, sizeof(bb_cal));
    nrfWriteReg(REG_RF_CAL, (char*) rf_cal, sizeof(rf_cal));
    nrfWriteReg(REG_DEM_CAL, (char*) demod, sizeof(demod));

    //
    // Power down by resetting NRF24_PWR_UP
    //
    nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_PRIM_RX));

    //
    // Configuration
    //
    nrfWrite1Reg(REG_EN_AA, 0x00);                  // No auto-ack on all pipes
    nrfWrite1Reg(REG_EN_RXADDR, 0x01);              // Enable only pipe 0
    nrfWrite1Reg(REG_SETUP_AW, NRF24_AW_5_BYTES);   // 5-byte TX/RX adddress
    nrfWrite1Reg(REG_SETUP_RETR, 0x00);             // no auto retransmissions

    nrfWrite1Reg(REG_RF_CH, RF_CHANNEL);            // Channel  
    nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);     // Clear status

    nrfWrite1Reg(REG_RF_SETUP, XN297_PWR_10dBm + XN297_LNA);  // 1Mbps, 10dBm, LNA-High-Current

    //LOG(0, "CH %d\n", RF_CHANNEL);

    nrfWrite1Reg(REG_RX_PW_P0, PAYLOADSIZE);        // Set payload size on all RX pipes
    nrfWrite1Reg(REG_RX_PW_P1, 0);
    nrfWrite1Reg(REG_RX_PW_P2, 0);
    nrfWrite1Reg(REG_RX_PW_P3, 0);
    nrfWrite1Reg(REG_RX_PW_P4, 0);
    nrfWrite1Reg(REG_RX_PW_P5, 0);

    nrfWrite1Reg(REG_FIFO_STATUS, 0x00);            // Clear FIFO bits (unnesseary?)

    //
    // We we need to activate feature before we do this, 
    // presubaly we can delete the next two lines
    //
    nrfWrite1Reg(REG_DYNPD,   0x00);                // Disable dynamic payload
    nrfWrite1Reg(REG_FEATURE, 0x00);                // No payloads ACKs
    nrfActivate();                                  // Activate feature  (why here?)
    nrfWrite1Reg(REG_DYNPD,   0x00);                // Disable dynamic payload length
    nrfWrite1Reg(REG_FEATURE, 0x00);                // Set feature bits on

    nrfActivate();

    // Flush the tranmit and receive buffer
    nrfFlushRx();
    nrfFlushTx();

    // Set device to bind address
    nrfWriteReg(REG_RX_ADDR_P0, (char*) rf_addr_bind, sizeof(rf_addr_bind));
    nrfWriteReg(REG_TX_ADDR,    (char*) rf_addr_bind, sizeof(rf_addr_bind));

    //dump_regs();

#if defined(TEST_TRANSMIT)
    {
        // transmit data -- used HackRF + gqrx set at 2402MHz
        const static uint8_t txdata[] = 
            {0xAA, 0x30, 0x31, 0x32, 0x33, 0x40, 0x41, 0x42, 0x3, 0x00, 0x00};

        LOG(0, "Transmit test\n");

        while(1) {
            nrfSetEnable(ENABLE_SEND);
            nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_PWR_UP));
            nrfWrite1Reg(REG_RF_CH, RF_CHANNEL);
            nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
            nrfFlushTx();
            nrfWriteTX((char*)txdata, sizeof(txdata));

            // Toggle CE line
            nrfSetEnable(ENABLE_RECEIVE);
            for (int i=0; i <6000; i++) {/*spin*/}  // delay ~2ms
        }
    }
#endif
}

#else  // defined(RF_XN297)

/*--------------------------------------------------------------------------*/
/* Initialize nRF24L01 and BK2423                                           */
/*--------------------------------------------------------------------------*/
void rfchip_init(void)
{
    const char rf_addr_bind[5] = {0x65, 0x65, 0x65, 0x65, 0x65};

    // Power down
    nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC  | NRF24_PRIM_RX));  

    // Configuration
    nrfWrite1Reg(REG_EN_AA, NRF24_ENAA_PA);          // Enable auto-ack on all pipes
    nrfWrite1Reg(REG_EN_RXADDR, NRF24_ERX_PA);       // Enable all pipes
    nrfWrite1Reg(REG_SETUP_AW, NRF24_AW_5_BYTES);    // 5-byte TX/RX adddress

    nrfWrite1Reg(REG_SETUP_RETR, 0x1A);             // 500uS timeout, 10 retries
    nrfWrite1Reg(REG_RF_CH, RF_CHANNEL);            // Channel 0x3C
    nrfWrite1Reg(REG_RF_SETUP, NRF24_PWR_0dBm);     // 1Mbps, 0dBm
    nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);     // Clear status

    nrfWrite1Reg( REG_RX_PW_P0, PAYLOADSIZE);       // Set payload size on all RX pipes
    nrfWrite1Reg( REG_RX_PW_P1, PAYLOADSIZE);
    nrfWrite1Reg( REG_RX_PW_P2, PAYLOADSIZE);
    nrfWrite1Reg( REG_RX_PW_P3, PAYLOADSIZE);
    nrfWrite1Reg( REG_RX_PW_P4, PAYLOADSIZE);
    nrfWrite1Reg( REG_RX_PW_P5, PAYLOADSIZE);

    nrfWrite1Reg( REG_FIFO_STATUS, 0x00);           // Clear FIFO bits (unnesseary?)

    // We we need to activate feature before we do this, presubaly we can delete the next two lines
    nrfWrite1Reg( REG_DYNPD, 0x3F);                  // Enable dynamic payload (all pipes)
    nrfWrite1Reg( REG_FEATURE, 0x07);                // Payloads with ACK, noack command

    // Maybe required
    nrfRead1Reg(REG_FEATURE);
    nrfActivate();                                  // Activate feature register
    nrfRead1Reg(REG_FEATURE);
    nrfWrite1Reg(REG_DYNPD, 0x3F);                   // Enable dynamic payload length on all pipes
    nrfWrite1Reg(REG_FEATURE, 0x07);                 // Set feature bits on

#if defined(RF_BK2423)
    // Check for Beken BK2421/BK2423 chip
    // It is done by using Beken specific activate code, 0x53
    // and checking that status register changed appropriately
    // There is no harm to run it on nRF24L01 because following
    // closing activate command changes state back even if it
    // does something on nRF24L01
    nrfActivateBK2423();

    if (nrfRead1Reg(REG_STATUS) & 0x80) {

        // RF is Beken 2423
        nrfWriteReg(0x00, (char *) "\x40\x4B\x01\xE2", 4);
        nrfWriteReg(0x01, (char *) "\xC0\x4B\x00\x00", 4);
        nrfWriteReg(0x02, (char *) "\xD0\xFC\x8C\x02", 4);
        nrfWriteReg(0x03, (char *) "\x99\x00\x39\x21", 4);
        nrfWriteReg(0x04, (char *) "\xD9\x96\x82\x1B", 4);
        nrfWriteReg(0x05, (char *) "\x24\x06\x7F\xA6", 4);
        nrfWriteReg(0x0C, (char *) "\x00\x12\x73\x00", 4);
        nrfWriteReg(0x0D, (char *) "\x46\xB4\x80\x00", 4);
        nrfWriteReg(0x04, (char *) "\xDF\x96\x82\x1B", 4);
        nrfWriteReg(0x04, (char *) "\xD9\x96\x82\x1B", 4);
    }

    nrfActivateBK2423(); 
#else
    nrfActivate(); 
#endif  // defined(RF_BK2423)

    // Flush the tranmit and receive buffer
    nrfFlushRx();
    nrfFlushTx();

    // Set device to bind address
    nrfWriteReg( REG_RX_ADDR_P0, (char *) rf_addr_bind, 5);
    nrfWriteReg( REG_TX_ADDR, (char *) rf_addr_bind, 5);

    //dump_regs();
}

#endif // defined(RF_XN297)

/*--------------------------------------------------------------------------*/
/*                                                                          */
/*--------------------------------------------------------------------------*/
void show_packet(char * packet, int len) {
    for (int i=0; i<len; i++){
         LOG(0, "%02x ", packet[i]);
    }
    LOG(0, "(%d)\n", len);
}

/*--------------------------------------------------------------------------*/
/* Configure the RF chip and bind                                           */
/*--------------------------------------------------------------------------*/
void init_RFRX(void)
{
    // Initialise SPI, clocks, etc.
    nrfInit();

    // Initialize RF-chip specific details.
    rfchip_init();

#if !defined(TEST_TRANSMIT) && !defined(TEST_SENSORS) // TEST_TRANSMIT and this won't fit in memory.

    static char rf_addr_cmnd[5];

    (void) rf_addr_cmnd;

    // Power up in RX mode
    nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_PWR_UP | NRF24_PRIM_RX));
    nrfSetEnable(ENABLE_RECEIVE);

#if 0

    LOG(0, "Waiting BIND\n");
    while (!(nrfGetStatus() & 0x40)) {
        bindflasher(500);
    }
    LOG(0, "received\n");

    // Bind packet is nine bytes on pipe zero
    uint8_t len = nrfRxLength(0);

    nrfReadRX(rxbuffer, PAYLOADSIZE);
    show_packet(rxbuffer, len);

    // Configure the command address
    rf_addr_cmnd[0] = rxbuffer[0];
    rf_addr_cmnd[1] = rxbuffer[1];
    rf_addr_cmnd[2] = rxbuffer[2];
    rf_addr_cmnd[3] = rxbuffer[3];
    rf_addr_cmnd[4] = 0xC1;

    // Flush buffer and clear status
    nrfFlushRx();
    nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);

    // Send reply
    nrfSetEnable(ENABLE_SEND);
    nrfWrite1Reg(REG_CONFIG, (NRF24_EN_CRC | NRF24_PWR_UP));
    nrfWrite1Reg(REG_RF_CH, RF_CHANNEL);
    nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);
    nrfFlushTx();

    rxbuffer[5] = 0xDE;
    rxbuffer[6] = 0xAD;
    rxbuffer[7] = 0xBE;
    rxbuffer[8] = 0xEF;
    rxbuffer[9] = 1;
    
    rxbuffer[10] = 0;
    rxbuffer[11] = 0;
    rxbuffer[12] = 0;
    rxbuffer[13] = 0;
    rxbuffer[14] = 0;
    rxbuffer[15] = 0;
    rxbuffer[16] = 0;
    rxbuffer[17] = 0;
    rxbuffer[18] = 0;

    show_packet(rxbuffer, len);

    nrfWriteTX((char*)rxbuffer, sizeof(rxbuffer));

    // Turn off blinking LEDs
    GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
    GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);

    nrfSetEnable(ENABLE_RECEIVE);

#else

    while (!bind) {

        // Wait until we receive a data packet, flashing alternately
        flashtime = micros() / 1000;

        //LOG(0, "Waiting BIND\n");
        while (!(nrfGetStatus() & 0x40)) {
            bindflasher(500);
        }
        //LOG(0, "received\n");

        // TX sends mutliple packets, so keep reading the FIFO
        // until there is no more data.
        while (!(nrfRead1Reg(REG_FIFO_STATUS) & 0x01)) {

            // Bind packet is nine bytes on pipe zero
            if (nrfRxLength(0) == PAYLOADSIZE) {

                // Get the packet
                nrfReadRX(rxbuffer, PAYLOADSIZE);
                break;
            }
        }
        //LOG(0, "L%d\n", __LINE__);

        // Flush buffer and clear status
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);

        // Configure the command address
        rf_addr_cmnd[0] = rxbuffer[0];
        rf_addr_cmnd[1] = rxbuffer[1];
        rf_addr_cmnd[2] = rxbuffer[2];
        rf_addr_cmnd[3] = rxbuffer[3];
        rf_addr_cmnd[4] = 0xC1;

        //LOG(0, "Addr %02x:%02x:%02X:%02x:%02x\n", rf_addr_cmnd[0], rf_addr_cmnd[1], rf_addr_cmnd[2], rf_addr_cmnd[3], rf_addr_cmnd[4]);

        show_packet(rxbuffer, PAYLOADSIZE);

        // Set to TX command address
        nrfWriteReg(REG_RX_ADDR_P0,  rf_addr_cmnd, sizeof(rf_addr_cmnd));
        nrfWriteReg(REG_TX_ADDR, rf_addr_cmnd, sizeof(rf_addr_cmnd));

        // Flush buffer and clear status
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);

        // Wait until we receive data on the command address
        flashtime = micros() / 1000;

        while (!(nrfGetStatus() & 0x40)) {
            bindflasher(250);
        }

        //LOG(0, "BIND!\n");
        bind = true;

        // Turn of LEDs
        GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
        GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
    }
#endif // !defined(TEST_TRANSMIT)

#endif
}

/*--------------------------------------------------------------------------*/
/* Place RF command data in RXcommand variable, process AUX commands        */
/*--------------------------------------------------------------------------*/
void get_RFRXDatas()
{
    // If a new packet exists in the buffer
    if (nrfGetStatus() & 0x40) {

        // Read the latest command to the buffer
        nrfReadRX(rxbuffer, PAYLOADSIZE);

        // Flush the buffer and clear interrupt
        nrfFlushRx();
        nrfWrite1Reg(REG_STATUS, NRF_STATUS_CLEAR);

        // FC order: T   A   E   R   A1   A2
        // RF order: T   R   ?   E   A    Et    At   F   ?

        // PPM firmware expects a range of 1000, TX range is 0xFF (max rate), mid-stick is 0x40,
        // BS twice leads to a range of 1020, which is close enough for now.
        RXcommands[0] = constrain((((int16_t)rxbuffer[0]) << 2), 0, 1000);
        RXcommands[1] = constrain((((int16_t)rxbuffer[4]) << 2) - 512, -500, 500);
        RXcommands[2] = constrain((((int16_t)rxbuffer[3]) << 2) - 512, -500, 500);
        RXcommands[3] = constrain((((int16_t)rxbuffer[1]) << 2) - 512, -500, 500);

        // Forward flip sets AUX1 high, backwards flip sets AUX1 low
        if (rxbuffer[7] & 0x0F) {
            if ((uint8_t) rxbuffer[3] > 0xF0) {
                RXcommands[4] = 500;
            }

            if ((uint8_t) rxbuffer[3] < 0x0F) {
                RXcommands[4] = -500;
            }
        }

        // Since data has been received, reset failsafe counter
        failsave = 0;
    }
}

/*--------------------------------------------------------------------------*/
/*                                                                          */
/*--------------------------------------------------------------------------*/
void bindflasher(uint32_t rate)
{
    uint32_t millitime = micros() / 1000;

    if (millitime - flashtime > rate) {
        flashtime = millitime;

        switch (flashstate) {

        case true:
            GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDon);
            GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDoff);
            flashstate = false;
            break;

        case false:
            GPIO_WriteBit(LED1_PORT, LED1_BIT, LEDoff);
            GPIO_WriteBit(LED2_PORT, LED2_BIT, LEDon);
            flashstate = true;
            break;

        }
    }
}
