

 
/*
*****************************************************************************************
* INCLUDE FILES
*****************************************************************************************
*/
#define USE_CAYENNELPP
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <Wire.h> 
#include  "adcvcc.h" 
#include <EEPROM.h>
#include "myconfig.h" 

/*
 * Defines
 */ 

#define debugSerial Serial 
#define SHOW_DEBUGINFO  
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); } 
#define debugFlush() { if (debugSerial) debugSerial.flush(); } 

// Pin mapping CH2I (check out : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059 ) 
#define LMIC_NSS    10
#define LMIC_RXTX   LMIC_UNUSED_PIN
#define LMIC_RST    LMIC_UNUSED_PIN
#define LMIC_DIO0   2
#define LMIC_DIO1   7
#define LMIC_DIO2   8

const lmic_pinmap lmic_pins = {
    .nss = LMIC_NSS,
    .rxtx = LMIC_RXTX,   
    .rst = LMIC_RST,
    .dio = {LMIC_DIO0, LMIC_DIO1, LMIC_DIO2},  
}; 

#define WAKE_PIN 3    //Pin used for wake interrupt
#define INTERRUPT 1    //Interrupt 1 = Pin 3

struct gas_counter_t
{
  long data;
};

gas_counter_t gas_counter;

//OTAA Functions...
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
 
static osjob_t sendjob;    
  

byte LMIC_transmitted = 0;
int LMIC_event_Timeout = 0;

/* ======================================================================
Function: ADC_vect
Purpose : IRQ Handler for ADC 
Input   : - 
Output  : - 
Comments: used for measuring 8 samples low power mode, ADC is then in 
          free running mode for 8 samples
====================================================================== */
ISR(ADC_vect)  
{
  // Increment ADC counter
  _adc_irq_cnt++;
}

void wakeUp()
{
   gas_counter.data += 1;
}

void onEvent (ev_t ev) 
{
    debugPrint(os_getTime());
    debugPrint(": ");
    debugPrintLn(ev);
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            //debugPrintLn(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //debugPrintLn(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //debugPrintLn(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //debugPrintLn(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            debugPrintLn(F("EV_JOINING"));
            break;
        case EV_JOINED:
            debugPrintLn(F("EV_JOINED"));
            break;
        case EV_RFU1:
            //debugPrintLn(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            debugPrintLn(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            debugPrintLn(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            debugPrintLn(F("EV_TXC"));
            if (LMIC.txrxFlags & TXRX_ACK)
              debugPrintLn(F("R ACK")); // Received ack
            if (LMIC.dataLen) 
            {
              debugPrintLn(F("R "));
              debugPrintLn(LMIC.dataLen);
              debugPrintLn(F(" bytes")); // of payload
              gas_counter.data = LMIC.frame[0] | LMIC.frame[1]<<8 | LMIC.frame[2]<<16; 
              debugPrintLn(F("gas counter"));
              debugPrintLn(gas_counter.data);
              debugPrintLn(F(" long"));
              saveGasCounter();
            }           
            LMIC_transmitted = 1; 
            break;
        case EV_LOST_TSYNC:
            //debugPrintLn(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //debugPrintLn(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //debugPrintLn(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //debugPrintLn(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //debugPrintLn(F("EV_LINK_ALIVE"));
            break;
         default:
            //debugPrintLn(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        debugPrintLn(F("OP_TXRXPEND")); //P_TXRXPEND, not sending
    } 
    else 
    {
        // Prepare upstream data transmission at the next possible time.
        int batt = (int)(readVcc() / 100);  // readVCC returns  mVolt need just 100mVolt steps
        byte batvalue = (byte)batt; // no problem putting it into a int. 
        
#ifdef SHOW_DEBUGINFO      
        debugPrint(F("B="));
        debugPrintLn(batt);
        debugPrint(F("BV="));
        debugPrintLn(batvalue); 
        debugPrint(F("gascounter="));
        debugPrintLn(gas_counter.data); 
#endif
        uint8_t mydata[4];
        mydata[0] =  batvalue;
        mydata[1] = gas_counter.data;
        mydata[2] = gas_counter.data >> 8;
        mydata[3] = gas_counter.data >> 16;
        
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        debugPrintLn(F("PQ")); //Packet queued
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
 

bool bootFromBrownOut = false;

// shows the bootstatus on serial output and set bootFromBrownout flag.
void showBootStatus(uint8_t _mcusr)
{
  debugPrint(F("mcusr = "));
  debugPrint(_mcusr, HEX);
  debugPrint(F(" > "));
  if (_mcusr & (1<<WDRF))
  {
    debugPrint(F(" WDR"));
    _mcusr &= ~(1<<WDRF);
  }
  if (_mcusr & (1<<BORF))
  {
    debugPrint(F(" BOR"));
    _mcusr &= ~(1<<BORF);
    bootFromBrownOut = true;
  }
  if (_mcusr & (1<<EXTRF))
  {
    debugPrint(F(" EXTF"));
    _mcusr &= ~(1<<EXTRF);
  }
  if (_mcusr & (1<<PORF))
  {
    debugPrint(F(" POR"));
    _mcusr &= ~(1<<PORF);
  }
  if (_mcusr != 0x00)
  {
    // It should never enter here
    debugPrint(F(" ??"));
  }
  debugPrintLn("");
}

void readGasCounter()
{
  EEPROM.get( 0, gas_counter );
  if(gas_counter.data < 0) gas_counter.data = 1;
}

void saveGasCounter()
{
  EEPROM.put( 0, gas_counter );
}

void setup() 
{
    uint8_t mcusr = MCUSR;
    MCUSR = 0;
  
    Serial.begin(115200);
    debugPrintLn(F("Boot")); 

    showBootStatus(mcusr);
    pinMode(WAKE_PIN, INPUT);
    digitalWrite(WAKE_PIN, HIGH);
   
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

    LMIC_setLinkCheckMode(1);
    LMIC_setAdrMode(1);
    readGasCounter();
    LMIC_startJoining();
    debugPrintLn(F("S")); // Setup complete!"
    debugFlush();
       
}

void loop() 
{    
    // Start job
    do_send(&sendjob);
    // Wait for response of the queued message (check if message is send correctly)
    os_runloop_once();
    // Continue until message is transmitted correctly 
    debugPrintLn(F("W")); // aiting for transmittion
    LMIC_event_Timeout = 60*100;  // 60 * 100 times 10mSec = 60 seconds
    while(LMIC_transmitted != 1) 
    {
            os_runloop_once();
            // Add timeout counter when nothing happens: 
            delay(10);
            if (LMIC_event_Timeout-- == 0) 
            {
                // Timeout when there's no "EV_TXCOMPLETE" event after 60 seconds
                debugPrintLn(F("ETO, msg not tx"));
                break;
            } 
    }
    LMIC_transmitted = 0;
    LMIC_event_Timeout = 0;
    debugPrintLn(F("G"));
    debugFlush();
    debugPrintLn("-");
        // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(INTERRUPT, wakeUp, CHANGE);

    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(INTERRUPT);   
    saveGasCounter(); 
}
