//
// INTRO
//
// The following code is a variant of the software for a Homematic HM-ES-TX-WM_CCU variant 
// based on AskSinPP [1]. The device supports GAS ONLY, e.g. energy related code has been removed.

//
// DISPLAY
// 
// It supports an OLED SSD1306 display.
// The display informs about:
// 
// E: GAS ENERGY COUNTER
// P: GAS POWER

//
// MODIFICATIONS
// 
// GAS ENERGY COUNTER is a cumulative counter of gas in m3.
// GAS POWER has been modified. It measures the amount of
// gas in 3 minutes and converts it to m3/h.

//
// YOUR MODIFICATIONS
// 
// Set your own device serial number, search for "papa555555" below.


// 
// LIBS
// 
// The software makes use of the following libraries:
// - greygnome/EnableInterrupt [2]
// - pa-pa/AskSinPP [3]
// - greiman/SSD1306Ascii [4]
//
// The library SSD1306Ascii has been chosen because the 
// "Adafruit SSD1306" library is too big to be included.

// 
// Author: J. Opschroef, November 2022
//
// License: Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/

// 
// REFERENCES
// 
// [1] HM-ES-TX-WM_CCU  https://github.com/jp112sdl/Beispiel_AskSinPP/tree/master/examples/HM-ES-TX-WM_CCU
// [2] EnableInterrupt https://github.com/GreyGnome/EnableInterrupt
// [3] AskSinPP https://github.com/pa-pa/AskSinPP and https://asksinpp.de/
// [4] SSD1306Ascii https://github.com/greiman/SSD1306Ascii

//
// ASSUMPTIONS
//
// (a) average gas consumption is 2000 m3 per year
// (b) the gas meter counts 0.01 m3 / pulse
//
// The following calculations are averages
// They are used to get an idea about the ranges
// I use gas to heat water and for the radiators
// Warm water is necessary the whole year,
// while heating is only necessary in winter.
// So the actual gas consumption is not constant
// but lower at summer and higher at winter.
//
//  1 year     2000 m3   200 kilopulses
//  5 years   10000 m3  1000 kilopulses
// 10 years   20000 m3  2000 kilopulses
//
//  1 day      5.5 m3    550 pulses
//  1 week    38.4 m3   3840 pulses
//  1 month  166.7 m3  16670 pulses
//
//  1 hour  0.228 m3    23 pulses
//
// So roughly a pulse every 2 to 3 minutes




//- -----------------------------------------------------------------------------------------------------------------------
// SSD1306 Ascii library
//- -----------------------------------------------------------------------------------------------------------------------

#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
#define RST_PIN -1
uint32_t oled_counter = 0;
uint32_t oled_counterSum = 0;
uint32_t oled_consumptionSum = 0;
float oled_consumptionSumFloat = 0;
uint32_t oled_actualConsumption = 0;
float oled_actualConsumptionFloat = 0;
uint32_t oled_c = 0;
uint16_t oled_sigs = 0;
float oled_sigsFloat = 0;

float oled_gasEnergyCounter = 0;
float oled_gasPower = 0;
uint8_t oled_meterType = 0;
uint8_t oled_contrast = 255;



SSD1306AsciiAvrI2c oled;


// define this to disable debug 
#define NDEBUG 

//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <MultiChannelDevice.h>

// we use a Pro Mini
// Arduino pin for the LED
// D4 == PIN 4 on Pro Mini
#define LED_PIN 4
// Arduino pin for the config button
// B0 == PIN 8 on Pro Mini
#define CONFIG_BUTTON_PIN 8
// Arduino pin for the counter impulse
// A0 == PIN 14 on Pro Mini
#define COUNTER1_PIN 14
// we send the counter every 3 minutes
#define MSG_CYCLE seconds2ticks(60 * 3)

// number of available peers per channel
#define PEERS_PER_CHANNEL 2

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
    {0x90,0x12,0x34},       // Device ID
    "papa555555",           // Device Serial
    {0x00,0xde},            // Device Model
    0x10,                   // Firmware Version
    as::DeviceType::PowerMeter, // Device Type
    {0x01,0x00}             // Info Bytes
};

/**
 * Configure the used hardware
 */
typedef AvrSPI<10,11,12,13> SPIType;
typedef Radio<SPIType,2> RadioType;
typedef StatusLed<LED_PIN> LedType;
typedef AskSin<LedType,BatterySensor,RadioType> HalType;

class MeterList0Data : public List0Data {
  uint8_t LocalResetDisbale : 1;   // 0x18 - 24
  uint8_t Baudrate          : 8;   // 0x23 - 35
  uint8_t SerialFormat      : 8;   // 0x24 - 36
  uint8_t MeterPowerMode    : 8;   // 0x25 - 37
  uint8_t MeterProtocolMode : 8;   // 0x26 - 38
  uint8_t SamplesPerCycle   : 8;   // 0x27 - 39

public:
  static uint8_t getOffset(uint8_t reg) {
    switch (reg) {
      case 0x18: return sizeof(List0Data) + 0;
      case 0x23: return sizeof(List0Data) + 1;
      case 0x24: return sizeof(List0Data) + 2;
      case 0x25: return sizeof(List0Data) + 3;
      case 0x26: return sizeof(List0Data) + 4;
      case 0x27: return sizeof(List0Data) + 5;
      default:   break;
    }
    return List0Data::getOffset(reg);
  }

  static uint8_t getRegister(uint8_t offset) {
    switch (offset) {
      case sizeof(List0Data) + 0:  return 0x18;
      case sizeof(List0Data) + 1:  return 0x23;
      case sizeof(List0Data) + 2:  return 0x24;
      case sizeof(List0Data) + 3:  return 0x25;
      case sizeof(List0Data) + 4:  return 0x26;
      case sizeof(List0Data) + 5:  return 0x27;
      default: break;
    }
    return List0Data::getRegister(offset);
  }
};

class MeterList0 : public ChannelList<MeterList0Data> {
public:
  MeterList0(uint16_t a) : ChannelList(a) {}

  operator List0& () const { return *(List0*)this; }

  // from List0
  HMID masterid () { return ((List0*)this)->masterid(); }
  void masterid (const HMID& mid) { ((List0*)this)->masterid(mid); }
  bool aesActive() const { return ((List0*)this)->aesActive(); }

  bool localResetDisable () const { return isBitSet(sizeof(List0Data) + 0,0x01); }
  bool localResetDisable (bool value) const { return setBit(sizeof(List0Data) + 0,0x01,value); }
  uint8_t baudrate () const { return getByte(sizeof(List0Data) + 1); }
  bool baudrate (uint8_t value) const { return setByte(sizeof(List0Data) + 1,value); }
  uint8_t serialFormat () const { return getByte(sizeof(List0Data) + 2); }
  bool serialFormat (uint8_t value) const { return setByte(sizeof(List0Data) + 2,value); }
  uint8_t powerMode () const { return getByte(sizeof(List0Data) + 3); }
  bool powerMode (uint8_t value) const { return setByte(sizeof(List0Data) + 3,value); }
  uint8_t protocolMode () const { return getByte(sizeof(List0Data) + 4); }
  bool protocolMode (uint8_t value) const { return setByte(sizeof(List0Data) + 4,value); }
  uint8_t samplesPerCycle () const { return getByte(sizeof(List0Data) + 5); }
  bool samplesPerCycle (uint8_t value) const { return setByte(sizeof(List0Data) + 5,value); }

  uint8_t transmitDevTryMax () const { return 6; }
  uint8_t ledMode () const { return 1; }

  void defaults () {
    ((List0*)this)->defaults();
  }
};

class MeterList1Data {
public:
  uint8_t  AesActive          :1;   // 0x08, s:0, e:1
  uint8_t  MeterType          :8;   // 0x95
  uint8_t  MeterSensibilityIR :8;   // 0x9c
  uint32_t TxThresholdPower   :24;  // 0x7C - 0x7E
  uint8_t  PowerString[16];         // 0x36 - 0x46 : 06 - 21
  uint8_t  EnergyCounterString[16]; // 0x47 - 0x57 : 22 - 37
  uint16_t MeterConstantIR    :16;  // 0x96 - 0x97 : 38 - 39
  uint16_t MeterConstantGas   :16;  // 0x98 - 0x99 : 40 - 41
  uint16_t MeterConstantLed   :16;  // 0x9a - 0x9b : 42 - 43

  static uint8_t getOffset(uint8_t reg) {
    switch (reg) {
      case 0x08: return 0;
      case 0x95: return 1;
      case 0x9c: return 2;
      case 0x7c: return 3;
      case 0x7d: return 4;
      case 0x7e: return 5;
      default: break;
    }
    if( reg >= 0x36 && reg <= 0x57 ) {
      return reg - 0x36 + 6;
    }
    if( reg >= 0x96 && reg <= 0x9b ) {
      return reg - 0x96 + 38;
    }
    return 0xff;
  }

  static uint8_t getRegister(uint8_t offset) {
    switch (offset) {
      case 0:  return 0x08;
      case 1:  return 0x95;
      case 2:  return 0x9c;
      case 3:  return 0x7c;
      case 4:  return 0x7d;
      case 5:  return 0x7e;
      default: break;
    }
    if( offset >= 6 && offset <= 37 ) {
      return offset - 6 + 0x36;
    }
    if( offset >= 38 && offset <= 43 ) {
      return offset - 38 + 0x96;
    }
    return 0xff;
  }
};

class MeterList1 : public ChannelList<MeterList1Data> {
public:
  MeterList1(uint16_t a) : ChannelList(a) {}

  bool aesActive () const { return isBitSet(0,0x01); }
  bool aesActive (bool s) const { return setBit(0,0x01,s); }
  uint8_t meterType () const { return getByte(1); }
  bool meterType (uint8_t value) const { return setByte(1,value); }
  uint8_t meterSensibilty () const { return getByte(2); }
  bool meterSensibilty (uint8_t value) const { return setByte(2,value); }
  uint32_t thresholdPower () const { return ((uint32_t)getByte(3)<<16) + ((uint16_t)getByte(4)<<8) + getByte(5); }
  bool thresholdPower (uint32_t value) const { return setByte(3,(value>>16)&0xff) && setByte(4,(value>>8)&0xff) && setByte(5,value&0xff); }

  uint16_t constantIR () const { return ((uint16_t)getByte(38)<<8) + getByte(39); }
  bool constantIR (uint16_t value) const { return setByte(38,(value>>8)&0xff) && setByte(39,value&0xff); }
  uint16_t constantGas () const { return ((uint16_t)getByte(40)<<8) + getByte(41); }
  bool constantGas (uint16_t value) const { return setByte(40,(value>>8)&0xff) && setByte(41,value&0xff); }
  uint16_t constantLed () const { return ((uint16_t)getByte(42)<<8) + getByte(43); }
  bool constantLed (uint16_t value) const { return setByte(42,(value>>8)&0xff) && setByte(43,value&0xff); }

  void defaults () {
    aesActive(false);
    meterType(0xff);
    meterSensibilty(0);
    thresholdPower(100*100);
    constantIR(100);
    constantGas(10);
    constantLed(10000);
  }
};

class GasPowerEventMsg : public Message {
public:
  void init(uint8_t msgcnt,bool boot,const uint64_t& counter,const uint32_t& power) {
    uint8_t cnt1 = (counter >> 24) & 0x7f;
    if( boot == true ) {
      cnt1 |= 0x80;
    }
    Message::init(0x10,msgcnt,0x54,BIDI|WKMEUP,cnt1,(counter >> 16) & 0xff);
    pload[0] = (counter >> 8) & 0xff;
    pload[1] = counter & 0xff;
    pload[2] = (power >> 16) & 0xff;
    pload[3] = (power >> 8) & 0xff;
    pload[4] = power & 0xff;
  }
};

class GasPowerEventCycleMsg : public GasPowerEventMsg {
public:
  void init(uint8_t msgcnt,bool boot,const uint64_t& counter,const uint32_t& power) {
    GasPowerEventMsg::init(msgcnt,boot,counter,power);
    typ = 0x53;
  }
};

class MeterChannel : public Channel<HalType,MeterList1,EmptyList,List4,PEERS_PER_CHANNEL,MeterList0>, public Alarm {

  const uint32_t    maxVal = 838860700;
  uint64_t          counterSum;
  volatile uint32_t counter; // declare as volatile because of usage withing interrupt
  Message           msg;
  uint8_t           msgcnt;
  bool              boot;
  
private:

public:
  MeterChannel () : Channel(), Alarm(MSG_CYCLE), counterSum(0), counter(0), msgcnt(0), boot(true) {}
  virtual ~MeterChannel () {}

  void firstinit () {
    Channel<HalType,MeterList1,EmptyList,List4,PEERS_PER_CHANNEL,MeterList0>::firstinit();
    getList1().meterType(number()==1 ? 1 : 8);  // Channel 1 default Gas / Channel 2 default IEC
  }

  uint8_t status () const {
    return 0;
  }

  uint8_t flags () const {
    return device().battery().low() ? 0x80 : 0x00;
  }

  void next () {
    // only count rotations/flashes and calculate real value when sending, to prevent inaccuracy
    counter++;

    device().led().ledOn(millis2ticks(300));
    
    #ifndef NDEBUG
      DHEXLN(counter);
    #endif
  }

  virtual void trigger (AlarmClock& clock) {
    tick = MSG_CYCLE;
    clock.add(*this);

    
    uint32_t consumptionSum;
    uint32_t actualConsumption=0;

    MeterList1 l1 = getList1();
    uint8_t metertype = l1.meterType(); // cache metertype to reduce eeprom access
    oled_meterType = metertype;
    if( metertype == 0 ) {
      return;
    }

    // copy value, to be consistent during calculation (counter may change when an interrupt is triggered)
    uint32_t c;
    ATOMIC_BLOCK( ATOMIC_RESTORESTATE )
    {
      c = counter;
      counter = 0;
    }
    counterSum += c;
    
    // Copy values for SSD1306 display
    
    DPRINTLN("");
    oled_c = c;
    DPRINT(F("c= "));
    DPRINTLN(c);
    
    oled_counterSum = counterSum;
    DPRINT(F("counterSum= "));
    DPRINTLN((uint32_t)counterSum);
    DPRINTLN("");

    // Suport Gas only
    uint16_t sigs = l1.constantGas();

    switch( metertype ) {
    case 1:
      consumptionSum = counterSum * sigs;
      // actualConsumption = (c * sigs * 10) / (MSG_CYCLE / seconds2ticks(60)); // This is the original calculation
      actualConsumption = (c * sigs * 60) / (MSG_CYCLE / seconds2ticks(60));   // Changed to "20" to convert into m3/h

      // Analysis of actualConsumption
      // Assuming every count event represents 0.01 m3. (sigs = 10)
      // Assuming a count event happens every 3 minutes. (c=1)
      // That means that the 20 * 0.01 m3 = 0.02 m3/hhas been consumed.
      
      // actualConsumption is calculated every 3 minutes
      // actualConsumption = (c * sigs * 60) / (MSG_CYCLE / seconds2ticks(60));
      // actualConsumption = (c * 10 * 60) / (18000 / 6000); 
      // actualConsumption = (1 * 600) / 3;
      // actualConsumption = 200;

      // What will Homematic Display ?
      // Homematic divides the value by 1000 to display "Verbrauch"
      // Verbrauch = 200 / 1000
      // Verbrauch = 0.2
      // I don't know why m3 is displayed instead of m3 / time interval

            
      // TODO handle overflow
      
      // Copy values for SSD1306 display
      DPRINTLN("");
      DPRINT(F("metertype= "));
      DPRINTLN(metertype);
 
      oled_consumptionSum = consumptionSum;
      DPRINT(F("consumptionSum= "));
      DPRINTLN(consumptionSum);
 
      oled_counterSum = counterSum;
      DPRINT(F("counterSum= "));
      DPRINTLN((uint32_t)counterSum);
           
      oled_sigs = sigs;
      DPRINT(F("sigs= "));
      DPRINTLN(sigs);

      oled_actualConsumption = actualConsumption;
      DPRINT(F("ActualConsumption= "));
      DPRINTLN(actualConsumption);
      DPRINTLN("");     
      
      
      
      
      ((GasPowerEventCycleMsg&)msg).init(msgcnt++,boot,consumptionSum,actualConsumption);
      break;

    /*
    case 8:
    
      ((IECEventCycleMsg&)msg).init(msgcnt++,number(),counterSum,actualConsumption,device().battery().low());

      break;
      */
    default:
      DPRINTLN(F("Unknown meter type"));
      return;
      break;
    }

    device().sendPeerEvent(msg,*this);
    boot = false;
  }
};

typedef MultiChannelDevice<HalType,MeterChannel,1,MeterList0> MeterType;

HalType hal;
MeterType sdev(devinfo,0x20);

template <uint8_t pin, void (*isr)(), uint16_t millis>
class ISRWrapper : public Alarm {
  uint8_t curstate;
public:
  ISRWrapper () : Alarm(0), curstate(HIGH) {
    pinMode(pin,INPUT_PULLUP);
  }
  virtual ~ISRWrapper () {}

  bool checkstate () {
    uint8_t oldstate = curstate;
    curstate = digitalRead(pin);
    return curstate != oldstate;
  }

  uint8_t state () const {
    return curstate;
  }

  void attach() {
    if( digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT )
      enableInterrupt(pin,isr,CHANGE);
    else
      attachInterrupt(digitalPinToInterrupt(pin),isr,CHANGE);
  }

  void detach () {
    if( digitalPinToInterrupt(pin) == NOT_AN_INTERRUPT )
      disableInterrupt(pin);
    else
      detachInterrupt(digitalPinToInterrupt(pin));
  }

  void debounce () {
    detach();
    tick = millis2ticks(millis);
    sysclock.add(*this);
  }

  virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
    checkstate();
    attach();
  }
};
void counter1ISR();
ISRWrapper<COUNTER1_PIN,counter1ISR,200> c1ISR;
void counter1ISR () {
  c1ISR.debounce();
  if( c1ISR.checkstate() ) {
    if( c1ISR.state() == LOW ) {
      sdev.channel(1).next();
    }
  }
}

ConfigButton<MeterType> cfgBtn(sdev);

class CycleInfoAlarm : public Alarm {
public:
CycleInfoAlarm () : Alarm (0) {}
virtual ~CycleInfoAlarm () {}
  void trigger (AlarmClock& clock)  {
   set(seconds2ticks(60UL*60));
   clock.add(*this);
   sdev.channel(1).changed(true);
  }
} cycle;


// 
// Update the SSD1306 Oled display
// 
void display_update(){
  
  String display_data_str = "";
  oled.setCursor(0,0);
  

  // E: GAS ENERGY COUNTER
  // derived from consumptionSum divided by 1000
  //
  // Template "E xxxxxx.xx m3  "
  // We can print 6 number plus "." plus two positions after decimal point - total 9
  

  oled.print(F("E "));
  
  oled_consumptionSumFloat = oled_consumptionSum;
  oled_gasEnergyCounter = oled_consumptionSumFloat/1000.0;

  display_data_str = String(oled_gasEnergyCounter);
  
  if (display_data_str.length() <= 9) { 
    // Number can be displayed
    
    for (int i=0; i<9-display_data_str.length(); i++ )
      oled.print(F(" "));  
    
    oled.print(display_data_str);
    oled.println(F(" m3"));
  } else {
    oled.println (F("Ueberlauf m3"));
  }
  

  
  // P:GAS POWER
  // derived from "actualConsumption" [m3/ 3 min]
  // "actualConsumption" is transferred to Homematic
  // The value on the OLED display is converted to 
  // gasPower in [m3/h]
  // 
  // Template "P xxxxxx.xx m3/h"

  oled.print(F("P "));

  oled_actualConsumptionFloat = oled_actualConsumption;
  oled_gasPower = oled_actualConsumptionFloat/1000.0; 

  display_data_str = String(oled_gasPower);
  if (display_data_str.length() <= 9) { 
    // Number can be displayed
    for (int i=0; i<9-display_data_str.length(); i++ )
      oled.print(F(" "));  
    oled.print(display_data_str);
    oled.println(F(" m3/h"));

  } else {
     oled.println (F("Ueberlauf m3/h"));
  }

  
  // I: number of count events / message cycle ("Impulse)
  // derived from oled_c
  //
  // Template  "I xxxxxx    Imp "

  oled.print("I ");  
  display_data_str = String(oled_c); // max 6 digits

  if (display_data_str.length() <= 6) { 
    // Number can be displayed
    for (int i=0; i<6-display_data_str.length(); i++ )
      oled.print(F(" "));  
    oled.print(display_data_str);
    oled.println(F("    Imp "));

  } else {
     oled.println (F("Ueberlauf Imp "));
  }



// C: Configuration of Gas consumption per count event
// Configuration is done in Homematic WebUI
// 
// First check, if correct meterType is selected.
// We allow "Gas" and "Unknown", only.
// oled_metertype =   0 -> default
// oled_metertype =   1 -> Gas
// oled_metertype =   2 -> IR
// oled_metertype =   4 -> LED
// oled_metertype = 255 -> Unbekannt
// 
// Template "C xxxxxx.xx m3/I"

oled_sigsFloat = oled_sigs/1000.0;
  
oled.print(F("C "));

if( ((1==oled_meterType) ||  (0==oled_meterType)) ){  // Gas or "Unknown"
  // Print the configuration
  display_data_str = String(oled_sigsFloat);
  
  if (display_data_str.length() <= 9) { 
    // Number can be displayed
    
    for (int i=0; i<9-display_data_str.length(); i++ )
      oled.print(F(" "));  
    
    oled.print(display_data_str);
    oled.println(F(" m3/I"));
  } else {
    oled.println (F("Ueberlauf m3/I"));
  }
  

} else {
  // Config error
  
  oled.println("Konfig Fehler ");
}

}


void display_init(){
  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
    // Call oled.setI2cClock(frequency) to change from the default frequency.

  oled.setFont(ZevvPeep8x16);
  /* Not necessary 
  oled.println(F("E xxxxxx.xx m3  "));
  oled.println(F("P xxxxxx.xx m3/h"));
  oled.println(F("I xxxxxx    Imp "));
  oled.println(F("C xxxxxx.xx m3/I"));
  */

}

  

void setup () {

  // 
  // AskSinPP 
  //
  DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);




  buttonISR(cfgBtn,CONFIG_BUTTON_PIN);

  // measure battery every 1h
  hal.battery.init(seconds2ticks(60UL*60),sysclock);
  // set low voltage to 2.2V
  hal.battery.low(22);
  hal.battery.critical(19);

  c1ISR.attach();
  // add channel 1 to timer to send event
  sysclock.add(sdev.channel(1));
  sdev.initDone();
  sysclock.add(cycle);

  //
  // SSD1306 Ascii library
  //
  display_init();
  
  // 
  // Print internal data to understand the code  :-)
  //
  DPRINT(F("seconds2ticks(60)= "));
  DPRINTLN(seconds2ticks(60));
  
  // MSG_CYCLE. The value is 3min * 100 = 180 * 100 = 18000
  DPRINT(F("MSG_CYCLE= "));
  DPRINTLN(MSG_CYCLE);

  // sigs: Gas constant, configured via Homematic Web GUI, "Gas-Zählerkonstante" [m3/ Impuls]
  DPRINT(F("Gas-Zählerkonstante (sigs)= "));
  DPRINTLN(oled_sigs);
 
  DPRINTLN("");
  }


void loop() {
  // AsksinPP 
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();

  if( worked == false && poll == false ) {
    hal.activity.savePower<Sleep<> >(hal);
  }

  // SSD1306 Ascii library
  display_update();

}