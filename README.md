# LoRaWAN_TTN_Gas_Counter

Description : 
 - A low power gas datalogger node for the ThingsNetwork with deepsleep support using a LMIC OTAA.  
 
 
Hardware used : 
 - Arduino Pro-Mini 3.3V 
 - Reed switch MK04

 
Software and libraries used : 
 - LMIC https://github.com/matthijskooijman/arduino-lmic 
 - LowPower library https://github.com/rocketscream/Low-Power
 - MiniCore loader  https://forum.arduino.cc/index.php?topic=412070 https://github.com/MCUdude/MiniCore 
   To use a brownout detection of 1.8V.
 - special adcvcc library from Charles (see : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059/32?u=lex_ph2lb )
 
For licenses of the used libraries, check the links above.
 
 
Aditional note : 

TTN decoder function :

    function Decoder(bytes, port) 
    {
      var retValue =   { 
        bytes: bytes
      };
      
      retValue.batt = bytes[0] / 10.0;
      if (retValue.batt === 0)
         delete retValue.batt; 
     
      if (bytes.length >= 2)
      {
        retValue.gascounter = (bytes[1] | bytes[2]<<8 | bytes[3] <<16) / 100.0;
        if (retValue.gascounter === 0)
          delete retValue.gascounter; 
      } 
       
      return retValue; 
    }