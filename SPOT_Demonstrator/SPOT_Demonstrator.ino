/*
 * Spot Demonstrator Kit
 *
 * Copyright (c) 2018-2021 INFICON Ltd.
 *
 */

// spot demonstrator firmware version
#define FWVERSION "01.07.00"

#include <Adafruit_RGBLCDShield.h>

#include <SPI.h>

#include "crc.h"

// INFICON Spot sensor
#include <inficon_spot.h>

// Adafruit LCD shield
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// bitmap of a user-defined degree symbol
uint8_t DegreeBitmap[]= {0x6, 0x9, 0x9, 0x6, 0x0, 0, 0, 0};
uint8_t Inficon1Bitmap[] = {0x0, 0x1, 0x6, 0xf, 0x1f, 0x1f, 0x1f, 0xe};
uint8_t Inficon2Bitmap[] = {0xc, 0x1e, 0xe, 0xc, 0xc, 0x8, 0x0, 0x0};

// pins used for connection with sensor
const int SSPin = 10; // slave(chip) select, input (low true)
const int RDYPin = 3; //Ready signal, out (low true)

InficonSpot spot = InficonSpot(SSPin, RDYPin, 4000000UL);

enum CrcState {
  CRC_OK,
  CRC_BAD,
  CRC_NOTCHECKED,
};

// crc32 over the spot program memory
struct CrcInfo {
  crc_t otpCrc;
  crc_t sramCrc;
  CrcState otpGood, sramGood;
};

CrcInfo crcInfo;

// base class for screens on the LCD
class Screen {
  public:
    virtual void enter() {} // called when entering this screen
    virtual void buttonUp() {}
    virtual void buttonDown() {}
    virtual void buttonSelect() {}
    virtual void display() = 0;
    virtual void tick() {}
};

// screen displaying combined pressure and temperature
class PresTempScreen : public Screen {
  public:
    virtual void display();
};

void PresTempScreen::display()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("p");
  lcd.setCursor(0,1);
  lcd.print("T");
  lcd.setCursor(8, 1);
  lcd.print("\001C");
}

// screen displaying combined pressure and temperature
class P1P2Screen : public Screen {
  public:
    virtual void display();
};

void P1P2Screen::display()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("p1");
  lcd.setCursor(0,1);
  lcd.print("p2");
}

class StatusScreen : public Screen {
  public:
    virtual void display();
};

void StatusScreen::display()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor status");
  lcd.setCursor(0, 1);
  lcd.print("0x");
}

// This screen provides the Zero function
class ZeroScreen : public Screen {
  public:
    ZeroScreen(void (*callback)());
    virtual void display();
    virtual void buttonSelect();
    void setZero(int32_t offset);
    int32_t getOffset();
  private:
    void (*_callback)();
    int32_t _offset;
};

ZeroScreen::ZeroScreen(void (*callback)())
{
  _callback = callback;
  _offset = 0;
}

void ZeroScreen::display()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("ZeroAdj [Select]");
}

void ZeroScreen::buttonSelect()
{
  lcd.setCursor(0,1);
  lcd.print("                ");
  _callback();
}

void ZeroScreen::setZero(int32_t offset)
{
  _offset = offset;
}

int32_t ZeroScreen::getOffset()
{
  return _offset;
}

// This screen provides the Zero function
class StartStreamScreen : public Screen {
  public:
    StartStreamScreen(void (*callback)());
    virtual void display();
    virtual void buttonSelect();
  private:
    void (*_callback)();
};

StartStreamScreen::StartStreamScreen(void (*callback)())
{
  _callback = callback;
}

void StartStreamScreen::display()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Press [Select]");
  lcd.setCursor(0,1);
  lcd.print("to start stream");
}

void StartStreamScreen::buttonSelect()
{
  lcd.setBacklight(YELLOW);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Streaming...");
  lcd.setCursor(0,1);
  lcd.print("([Reset] to end)");
  _callback();
}

class LabelDataScreen : public Screen {
  public:
    LabelDataScreen(const char *msg, InficonSpot::ReadLabelFunction readLabelFunction);
    virtual void display();
  private:
    const char *_msg;
    InficonSpot::ReadLabelFunction _readLabelFunction;
};

LabelDataScreen::LabelDataScreen(const char *msg, InficonSpot::ReadLabelFunction readLabelFunction)
{
  _msg = msg;
  _readLabelFunction = readLabelFunction;
}

void LabelDataScreen::display()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(_msg);
  lcd.setCursor(0, 1);
  lcd.print((spot.*_readLabelFunction)().substring(0, 16));
}

class CrcScreen : public Screen {
  public:
    virtual void display();
};

void CrcScreen::display()
{
  char buf[17];

  lcd.clear();
  lcd.setCursor(0, 0);
  sprintf(buf, "OTP:%08lX", crcInfo.otpCrc);
  lcd.print(buf);
  lcd.setCursor(13, 0);
  switch(crcInfo.otpGood) {
    case CRC_OK:
      lcd.print("ok");
      break;
    case CRC_BAD:
      lcd.print("bad");
      break;
    default:
      lcd.print("n/a");
      break;
  }
  lcd.setCursor(0, 1);
  sprintf(buf, "RAM:%08lX", crcInfo.sramCrc);
  lcd.print(buf);
  lcd.setCursor(13, 1);
  switch(crcInfo.sramGood) {
    case CRC_OK:
      lcd.print("ok");
      break;
    case CRC_BAD:
      lcd.print("bad");
      break;
    default:
      lcd.print("n/a");
      break;
  }
}

// screen displaying combined pressure and temperature
class SwVersionScreen : public Screen {
  public:
    virtual void display();
};

void SwVersionScreen::display()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Demo FW Version");
  lcd.setCursor(0,1);
  lcd.print(FWVERSION);
}

PresTempScreen presTempScreen;
P1P2Screen p1p2Screen;
StatusScreen statusScreen;
ZeroScreen zeroScreen(zeroGauge);
StartStreamScreen startStreamScreen(startStream);
LabelDataScreen prodNumScreen("Label: Prod No.", &InficonSpot::readProductNo);
LabelDataScreen serialNoScreen("Label: Serial No", &InficonSpot::readSerialNo);
LabelDataScreen readFs1Screen("Label: FS p1", &InficonSpot::readFullscale1);
LabelDataScreen readFs2Screen("Label: FS p2", &InficonSpot::readFullscale2);
LabelDataScreen readTypeScreen("Label: Type", &InficonSpot::readType);
LabelDataScreen readSpeedScreen("Label: Speed", &InficonSpot::readSpeed);
SwVersionScreen swVersionString;
CrcScreen crcScreen;

const int NUMSCREENS = 13;
Screen *screens[NUMSCREENS] = {
    &presTempScreen, &p1p2Screen,
    &statusScreen,
    &zeroScreen,
    &startStreamScreen,
    &prodNumScreen,
    &serialNoScreen,
    &readFs1Screen,
    &readFs2Screen,
    &readTypeScreen,
    &readSpeedScreen,
    &swVersionString,
    &crcScreen,
};
int displayscreen = 0;

struct results {
  bool locked;
  bool updated;
  int32_t pressure;
  int32_t p1;
  int32_t p2;
  int32_t temperature;
  uint32_t status;
  uint32_t count;
  int32_t mean_pressure;
  int32_t mean_p1;
  int32_t mean_p2;
  int32_t mean_temperature;
  uint32_t total_errors;
  int n; // number of results in mean value fields
};

volatile struct results results;

String p1unit;
String p2unit;
int p1fullscale;
int p2fullscale;
int sensorspeed;      // Sensor speed in milliseconds
int average;          // Moving average window size

void setup() {
  // setup serial port
  Serial.begin(2000000); // 2 MBaud

  // Adafruit LCD shield, 16 columns, 2 lines
  lcd.begin(16, 2);
  lcd.createChar(1, DegreeBitmap);
  lcd.createChar(2, Inficon1Bitmap);
  lcd.createChar(3, Inficon2Bitmap);

  lcd.clear();
  lcd.print("\002\003 INFICON Spot ");
  lcd.setBacklight(WHITE);
  delay(1000);

  // initialize results struct
  results.mean_pressure = 0;
  results.mean_temperature = 0;
  results.mean_p1 = 0;
  results.mean_p2 = 0;
  results.total_errors = 0;
  results.n = 0;
  results.updated = false;
  results.locked = false;

  screens[displayscreen]->enter();
  screens[displayscreen]->display();

  // Initialize Spot sensor
  spot.begin();
  Serial.println ("#working");

  // stop the sensor before reading
  spot.writeRegister(20, 0);

  // calculate OTP crc32
  crc_t otp_crc32, storedCrc;
  otp_crc32 = crc_init();
  otp_crc32 = crc_loop(otp_crc32, 0, 3819, true);
  otp_crc32 = crc_loop(otp_crc32, 3824, 4094, true);
  otp_crc32 = crc_finalize(otp_crc32);
  storedCrc = spot.readOtpCrc();
  Serial.println("#OTP: calculated CRC is " + String(otp_crc32, HEX) + ", stored in mem " +
    String(storedCrc, HEX));
  crcInfo.otpCrc = otp_crc32;
  if (storedCrc == 0xffffffff) {
    crcInfo.otpGood = CRC_NOTCHECKED;
  } else {
    if (otp_crc32 == storedCrc) {
      crcInfo.otpGood = CRC_OK;
    } else {
      crcInfo.otpGood = CRC_BAD;
    }
  }

  // calculate SRAM CRC
  crc_t sram_crc32;
  sram_crc32 = crc_init();
  sram_crc32 = crc_loop(sram_crc32, 0, 3815, false);
  sram_crc32 = crc_loop(sram_crc32, 3824, 4031, false);
  sram_crc32 = crc_finalize(sram_crc32);
  storedCrc = spot.readSramCrc();
  Serial.println("#SRAM: calculated CRC is " + String(sram_crc32, HEX) + ", stored in mem " +
    String(storedCrc, HEX));
  crcInfo.sramCrc = sram_crc32;
  if (storedCrc == 0xffffffff) {
    crcInfo.sramGood = CRC_NOTCHECKED;
  } else {
    if (sram_crc32 == storedCrc) {
      crcInfo.sramGood = CRC_OK;
    } else {
      crcInfo.sramGood = CRC_BAD;
    }
  }

  // restart sensor
  spot.resetSensor();
  delay(10);
  spot.sendCommand(0x8a);
  delay(10);
  spot.writeRegister(20, 0);

  // install spot interrupt
  attachInterrupt(digitalPinToInterrupt(RDYPin), spotInterrupt, FALLING);

  // reset the Spot sensor
  spot.resetSensor();
  delay(10);
  spot.sendCommand(0x8a);
  delay(100);

  // read fullscales and units from the sensor
  String label = spot.readFullscale1();
  bool label1good = false, label2good = false;
  if (label.substring(0,4) == "FS1=") {
    // separate value and unit
    unsigned int i = 4;
    while ((i < label.length()) && (isDigit(label.charAt(i)) || (label.charAt(i) == '.'))) {
      i++;
    }
    p1fullscale = label.substring(4, i).toInt();
    if (i < label.length()) {
      p1unit = label.substring(i);
      label1good = true;
    }
  }

  label = spot.readFullscale2();
  if (label.substring(0,4) == "FS2=") {
    // separate value and unit
    unsigned int i = 4;
    while ((i < label.length()) && (isDigit(label.charAt(i)) || (label.charAt(i) == '.'))) {
      i++;
    }
    p2fullscale = label.substring(4, i).toInt();
    if (i < label.length()) {
      p2unit = label.substring(i);
      label2good = true;
    }
  }

  // sensor speed
  label = spot.readSpeed();
  bool labelspeedgood = false;
  if (label.substring(0, 6) == "Speed=") {
    // separate value and unit
    unsigned int i = 6;
    while ((i < label.length()) && (isDigit(label.charAt(i)) || (label.charAt(i) == '.'))) {
      i++;
    }
    sensorspeed = label.substring(6, i).toInt();
    if (i < label.length()) {
      String speedunit = label.substring(i);
      if (speedunit == "ms") {
        labelspeedgood = true;
      }
    }
  }

  if (!labelspeedgood) {
    Serial.println("#Error reading speed label from sensor");
    sensorspeed = 0;
    average = 128;
  } else {
    Serial.println("#Sensor speed is " + String(sensorspeed) + " ms.");
    // aim at 4 display updates per second
    average = 1000 / (4 * sensorspeed);
    if (average < 1) {
      average = 1;
    }
    Serial.println("#Moving average window size = " + String(average));
  }

  if ((!label1good) || (!label2good)) {
    // reading the labels failed
    p1fullscale = p2fullscale = 0;
    Serial.println("#Error reading fullscale labels from sensor");
  } else {
    Serial.println(String("#Pressure 1 fullscale: ") + String(p1fullscale, DEC) + String(" ") + String(p1unit));
    Serial.println(String("#Pressure 2 fullscale: ") + String(p2fullscale, DEC) + String(" ") + String(p2unit));
  }
}

crc_t crc_loop(crc_t crc, uint16_t start, uint16_t end, bool otp)
{
  uint8_t crcbuf[16];
  uint16_t length = end - start + 1;
  uint16_t address = start;
  while (length) {
    uint16_t readlen = (length < 16) ? length : 16;
    if (otp) {
      spot.readOTP(address, crcbuf, readlen);
    } else {
      spot.readMemory(address, crcbuf, readlen);
    }
    crc = crc_update(crc, crcbuf, readlen);
    address += readlen;
    length -= readlen;
  }
  return crc;
}

unsigned long lastresult = 0, lastkeyboardtime = 0;
uint8_t oldbuttons;

// display blinking, call the tick function of the current screen every second
unsigned long blinktick = 0;

// request zero gauge
bool requestZero = false;
unsigned long requestZeroTime = 0;

enum DemoMode {
  MODE_NORMAL,
  MODE_STREAMING,
};

volatile DemoMode mode = MODE_NORMAL;

void loop() {
  if (millis() - lastresult > 1000) {
    // no measurement received, in the last 1000 ms, try to reset Spot
    Serial.println("#Timeout: Resetting Spot");
    results.locked = true;
    spot.resetSensor();
    delay(10);
    spot.sendCommand(0x8a);
    delay(10);

    lastresult = millis(); // store the time of the last valid measurement
    results.mean_pressure = 0;
    results.mean_temperature = 0;
    results.n = 0;
    results.updated = false;
    results.locked = false;
    lastresult = millis();
  }

  if (mode == MODE_STREAMING) {
    // wait for new data and process it
    if (results.updated) {
      lastresult = millis(); // store the time of the last valid measurement

      // results have been updated, get the results
      struct results tmp;
      results.locked = true; // protect the results from beeing updated by the interrupt
      tmp.pressure = results.pressure;
      tmp.temperature = results.temperature;
      tmp.status = results.status;
      tmp.count = results.count;
      results.updated = false;
      results.locked = false;
      Serial.println(String(tmp.count) + "," + String(spot.convertPressure(tmp.pressure, p1fullscale)) + "," +
          String(spot.convertTemperature(tmp.temperature)) + "," +
          String(tmp.status & (InficonSpot::STATUS_SPI_INTERFERENCE | InficonSpot::STATUS_PRESSURE_ERROR |
          InficonSpot::STATUS_PORT3_ERROR | InficonSpot::STATUS_PORT2_ERROR | InficonSpot::STATUS_PORT1_ERROR |
          InficonSpot::STATUS_PORT0_ERROR | InficonSpot::STATUS_TEMPERATURE_ERROR)));
    }
    return;
  }

  if (millis() - blinktick > 1000) {
    screens[displayscreen]->tick();
    blinktick = millis();
  }

  if (requestZero && (millis() - requestZeroTime > 5000)) {
    // timeout while waiting for zero value
    if (screens[displayscreen] == &zeroScreen) {
      lcd.setCursor(0, 1);
      lcd.print("timeout");
    }
    requestZero = false;
  }

  // wait for new data and process it
  if (results.updated) {
    lastresult = millis(); // store the time of the last valid measurement

    // results have been updated, get the results
    struct results tmp;
    bool haveNewMeanValue = false;
    results.locked = true; // protect the results from beeing updated by the interrupt
    tmp.pressure = results.pressure;
    tmp.p1 = results.p1;
    tmp.p2 = results.p2;
    tmp.temperature = results.temperature;
    tmp.status = results.status;
    if (results.n >= average) {
      // we have a new mean value, update display and restart averaging
      tmp.mean_pressure = results.mean_pressure;
      tmp.mean_p1 = results.mean_p1;
      tmp.mean_p2 = results.mean_p2;
      tmp.mean_temperature = results.mean_temperature;
      tmp.total_errors = results.total_errors;
      tmp.n = results.n;

      results.mean_pressure = 0;
      results.mean_p1 = 0;
      results.mean_p2 = 0;
      results.mean_temperature = 0;
      results.total_errors = 0;
      results.n = 0;
      haveNewMeanValue = true;

      // calculate mean value
      tmp.mean_pressure /= tmp.n;
      tmp.mean_p1 /= tmp.n;
      tmp.mean_p2 /= tmp.n;
      tmp.mean_temperature /= tmp.n;

      // convert to q17
      tmp.mean_pressure >>= 4;
      tmp.mean_p1 >>= 4;
      tmp.mean_p2 >>= 4;
      tmp.mean_temperature >>= 4;
    }
    results.updated = false;
    results.locked = false;

    if (haveNewMeanValue && (screens[displayscreen] == &presTempScreen)) {
      char buf[6];
      buf[5] = '\0';
      lcd.setCursor(2, 0);
      if (p1fullscale != 0) {
        q17ToString(buf, 5, (tmp.mean_pressure - zeroScreen.getOffset()) * p1fullscale);
        lcd.print(buf);
        lcd.print(" ");
        lcd.print(p1unit);
      } else {
        lcd.print("            ");
      }
      lcd.setCursor(2, 1);
      q17ToString(buf, 4, tmp.mean_temperature * 25);
      buf[4] = '\0';
      lcd.print(buf);
    }
    if (haveNewMeanValue && (screens[displayscreen] == &p1p2Screen)) {
      char buf[6];
      buf[5] = '\0';
      lcd.setCursor(3, 0);
      if (p1fullscale != 0) {
        q17ToString(buf, 5, tmp.mean_p1 * p1fullscale);
        lcd.print(buf);
        lcd.print(" ");
        lcd.print(p1unit);
      } else {
        lcd.print("            ");
      }
      lcd.setCursor(3, 1);
      if (p2fullscale != 0) {
        q17ToString(buf, 5, tmp.mean_p2 * p2fullscale);
        lcd.print(buf);
        lcd.print(" ");
        lcd.print(p2unit);
      } else {
        lcd.print("            ");
      }
    }
    if (haveNewMeanValue && (screens[displayscreen] == &statusScreen)) {
      lcd.setCursor(2, 1);
      char hexstr[10];
      uint32_t err = tmp.total_errors & (InficonSpot::STATUS_SPI_INTERFERENCE | InficonSpot::STATUS_PRESSURE_ERROR |
        InficonSpot::STATUS_PORT3_ERROR | InficonSpot::STATUS_PORT2_ERROR | InficonSpot::STATUS_PORT1_ERROR |
        InficonSpot::STATUS_PORT0_ERROR | InficonSpot::STATUS_TEMPERATURE_ERROR);
      sprintf(hexstr, "%06lx", err);
      lcd.print(hexstr);
      lcd.setCursor(9, 1);
      if (err) {
        lcd.print("(err)");
      } else {
        lcd.print("(ok) ");
      }
    }
    if (haveNewMeanValue && requestZero) {
      if ((tmp.total_errors & (InficonSpot::STATUS_SPI_INTERFERENCE | InficonSpot::STATUS_PRESSURE_ERROR |
        InficonSpot::STATUS_PORT3_ERROR | InficonSpot::STATUS_PORT2_ERROR | InficonSpot::STATUS_PORT1_ERROR |
        InficonSpot::STATUS_PORT0_ERROR | InficonSpot::STATUS_TEMPERATURE_ERROR)) == 0) {
          // no error, we have zeroed the gauge!
          zeroScreen.setZero(tmp.mean_pressure);
          if (screens[displayscreen] == &zeroScreen) {
            lcd.setCursor(0, 1);
            lcd.print("done");
        }
        requestZero = false;
      }
    }
  }

  // handle keyboard, detect buttondown events
  if (millis() - lastkeyboardtime > 30) {
    lastkeyboardtime = millis();
    // 30 ms debounce time over, check keyboard again
    uint8_t buttons = lcd.readButtons();
    uint8_t buttondown = (buttons ^ oldbuttons) & buttons;
    oldbuttons = buttons;
    if (buttondown == BUTTON_LEFT) {
      displayscreen--;
      if (displayscreen < 0) {
        displayscreen = NUMSCREENS - 1;
      }
      screens[displayscreen]->enter();
      screens[displayscreen]->display();
    }
    if (buttondown == BUTTON_RIGHT) {
      displayscreen++;
      if (displayscreen >= NUMSCREENS) {
        displayscreen = 0;
      }
      screens[displayscreen]->enter();
      screens[displayscreen]->display();
    }
    if (buttondown == BUTTON_SELECT) {
      screens[displayscreen]->buttonSelect();
    }
    if (buttondown == BUTTON_UP) {
      screens[displayscreen]->buttonUp();
    }
    if (buttondown == BUTTON_DOWN) {
      screens[displayscreen]->buttonDown();
    }
  }
}

uint32_t count = 0;

void spotInterrupt() {
  // Read the pressure data, temperature and status
  uint32_t pressure = spot.readRegister(InficonSpot::REG_PRESSURE);
  uint32_t p1 = spot.readRegister(InficonSpot::REG_PRESSURE1);
  uint32_t p2 = spot.readRegister(InficonSpot::REG_PRESSURE2);
  uint32_t temperature = spot.readRegister(InficonSpot::REG_TEMPERATURE);
  uint32_t status = spot.readRegister(InficonSpot::REG_STATUS);

  if (!results.locked) {
    // convert 24 bit signed integers to 32 bit signed integers
    *(uint32_t *) &results.pressure = (pressure & 0x800000) ? (0xff000000 | pressure) : pressure;
    *(uint32_t *) &results.p1 = (p1 & 0x800000) ? (0xff000000 | p1) : p1;
    *(uint32_t *) &results.p2 = (p2 & 0x800000) ? (0xff000000 | p2) : p2;
    *(uint32_t *) &results.temperature = (temperature & 0x800000) ? (0xff000000 | temperature) : temperature;
    results.status = status;
    results.count = count;
    results.mean_pressure += results.pressure;
    results.mean_temperature += results.temperature;
    results.mean_p1 += results.p1;
    results.mean_p2 += results.p2;
    results.total_errors |= results.status;
    results.n++;
    results.updated = true;
  }
  count++;
}

// zero gauge screen callback function
void zeroGauge()
{
  requestZero = true;
  requestZeroTime = millis();
}

// switch to streaming mode
void startStream()
{
  mode = MODE_STREAMING;
}

void q17ToString(char *str, int len, int32_t inum)
{
  char *endp = &str[len];
  uint32_t num;

  if (inum < 0) {
    num = -inum;
  } else {
    num = inum;
  }

  uint32_t intpart = num >> 17;

  if (len <= 0)
    return;

  // first convert the integer part of num to a right-aligned string
  do {
    *--endp = '0' + intpart % 10;
    intpart /= 10;
  } while ((intpart > 0) && (endp > str));

  // add minus to negative numbers
  if (inum < 0) {
    if (endp > str) {
      *--endp = '-';
    } else {
      // no space left for minus, trigger error
      intpart = 1;
    }
  }

  // if intpart is not yet zero, the target string is too short. Replace everything with #
  if (intpart > 0) {
    for (int i = 0; i < len; i++) {
      str[i] = '#';
    }
    return;
  }

  if (endp == str) {
    // string is already full, we are done
    return;
  }

  if (endp == str + 1) {
    // only one character left, clear it and exit
    *--endp = ' ';
  }

  // make the result that we have now left aligned
  char *startp = str;
  while (endp < &str[len]) {
    *startp++ = *endp++;
  }

  if (startp == &str[len]) {
    // string is full, we are done
    return;
  }

  if (startp == &str[len - 1]) {
    // one character left, make it blank
    *startp = ' ';
    return;
  }

  // add a dot and decimal places
  *startp++ = '.';
  while (startp < &str[len]) {
    num &= (1UL << 17) - 1;
    num *= 10;
    *startp++ = '0' + (num >> 17);
  }
}
