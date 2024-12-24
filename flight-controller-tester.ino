// For Wemos Lolin32, OLED with 18650


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MSP.h>

#include "bee.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define UART_RX 16
#define UART_TX 16
#define IO_PROBE 18
// #define SAMPLE_COUNT (64 * 24) + 4 // 1 LED requires 24 bits (48 transitions) on each side of the idle
#define SAMPLE_COUNT (2  * 16 * 2) + 4 // Dshot requires 16 bits (32 transitions) on either side of the idle, which may be in the center

int32_t debug_code = 0;
char msg[32];
uint32_t timings[SAMPLE_COUNT - 1];
int timing_bit = 0;

typedef struct { int idle; int min_bits; int max_high; int min_high; int min_low; int max_low; } timing_conf;


// define PIN_READ(x) ( REG_READ(GPIO_IN_REG) >> (x) & 1);
// ULL IO_PROBE_MASK = (1ULL << IO_PROBE));

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
MSP msp;


void uart_ok(void) {
  display.clearDisplay();
  display.setCursor(0, 20);     // Start at top-left corner
  display.write("UART OK");
  display.display();
}


void test_sbus() {
  msp_api_version_t apiv;
  // Serial.println("running uart test");

  Serial2.begin(115200, SERIAL_8N1, 16, 17, true, 200);
  delay(50);

  Serial2.println("#");
  delay(100);
  Serial2.println("R");

  msp.begin(Serial2);
  msp.command(68, NULL, 0);
  Serial2.end(); 
}


void test_uart() {
  msp_api_version_t apiv;
  // Serial.println("running uart test");

  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  msp.begin(Serial2);
  

  if (msp.request(MSP_API_VERSION, &apiv, sizeof(apiv))) {
    Serial.printf("prot version: %u, API major: %u\n", (unsigned int) apiv.protocolVersion, (unsigned int) apiv.APIMajor);
    if ( (apiv.protocolVersion == 0) && apiv.APIMajor) {
      uart_ok();
      delay(1500);
    }
  } else {
    display.clearDisplay();
    display.display();
  }

  msp.reset();
  Serial2.end(); 
}


static inline int32_t cpu_ticks() {
  int32_t r;
  asm volatile ("rsr %0, ccount" : "=r"(r));
  return r;
}



uint32_t timing_to_bits(timing_conf conf) {
  uint32_t iobits = 0;
  int32_t cpu_mhz = (float) ESP.getCpuFreqMHz();
  int frame_start = 0;
  int i;


  for (int i=1; i < SAMPLE_COUNT / 2; i++) {
    uint32_t duration = ( (uint32_t)(timings[i]- timings[i - 1]) ) / (cpu_mhz / 10);
    // TODO comment this out
    
    if ( (duration != 0) ) {
      Serial.println(duration);
    }
    
    if (duration > conf.idle)  {
       frame_start = i + 1;
       Serial.printf("found frame start at %d with duration %d\n", frame_start, duration);
       break;
    }
  }

  if (frame_start == 0) {
    Serial.println("frame start not found in first half of bits");
    return(0);
  }

  for (i=frame_start; i < frame_start + (conf.min_bits * 2); i = i + 2) {
    uint32_t duration = ( (uint32_t)(timings[i]- timings[i - 1]) ) / (cpu_mhz / 10);
    // Serial.println(duration);
    if ( (duration <= conf.max_high) && (duration >= conf.min_high) ) {
      iobits =  ( iobits << 1 ) | 1;
    } else if ( (duration <= conf.max_low) && (duration >= conf.min_low) ) {
      iobits =  ( iobits << 1 );
    } else {
      Serial.print("invalid bit length ");
      Serial.println(duration);
      break;
    }
  }

  if ( (i - frame_start) > conf.min_bits) {
    return iobits;
  } else {
    Serial.printf("Not enough bits found. Found %d bits after frame start %d\n", i - frame_start, frame_start);
    return 0;
  }
}

void IRAM_ATTR isr_null() { }

void IRAM_ATTR probe_change() {
  if (timing_bit < SAMPLE_COUNT) {
    // timings[timing_bit++] = ESP.getCycleCount();
    timings[timing_bit++] = cpu_ticks();
  } else {
    attachInterrupt(digitalPinToInterrupt(IO_PROBE), isr_null, CHANGE);
    detachInterrupt(IO_PROBE);
  }
}

void IRAM_ATTR probe_high() {
  attachInterrupt(digitalPinToInterrupt(IO_PROBE), probe_change, CHANGE);
}


void get_timings(int milliseconds) {
  timing_bit = 0;

  for (int i=0; i < SAMPLE_COUNT; i++) {
    timings[i] = 0;
  }
  // TODO put this back so I know the first timing is a low
  // attachInterrupt(digitalPinToInterrupt(IO_PROBE), probe_high, HIGH);
  attachInterrupt(digitalPinToInterrupt(IO_PROBE), probe_change, CHANGE);
  delay(milliseconds);
  // TODO put this back
  // attachInterrupt(digitalPinToInterrupt(IO_PROBE), isr_null, HIGH);
  detachInterrupt(IO_PROBE);
}



int dshot_crc(uint16_t iobits) {
  uint8_t crc = (iobits ^ (iobits >> 4) ^ (iobits >> 8)) & 0x0F;
  return( ( (iobits >> 12) & 0x0F) == crc);
}

int dshot_crc_inav(uint16_t iobits) {
    int csum = 0;
    int csum_data = iobits;
    for (int i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
    Serial.print("(iobits & 0x0F) == csum?   :  ");
    Serial.print(iobits & 0x0F);
    Serial.print( " == ");
    Serial.println(csum);
    return( (iobits & 0x0f) == csum);
}


void print_dshot_bits(uint16_t iobits) {
  Serial.print("crc match: ");
  Serial.println( dshot_crc(iobits) );
  for (int i=0; i < 16; i++) {
    if ((i % 4) == 0) {
      Serial.print(" ");
    }
    if ( bitRead(iobits,i) ) {
      Serial.print ("1");
    } else {
      Serial.print ("0");
    }
  }
  Serial.println("");
}



void test_dshot() {
  uint16_t iobits = 0;

  timing_conf bit_timing;
  bit_timing.idle = 70;  // TODO was 50
  bit_timing.min_bits = 16;
  bit_timing.max_high = 60;
  bit_timing.min_high = 40;
  bit_timing.max_low = 35;
  bit_timing.min_low = 10; // TODO should this be about 25?

  display.clearDisplay();

  iobits = timing_to_bits(bit_timing);
  // iobits = iobits >> 16;
  if (iobits > 0) {
    print_dshot_bits(iobits);
    if ( ! dshot_crc(iobits) ) {
      return;
    }

    
    int throttle = (iobits >> 5) & 0x07ff;
    // throttle = (throttle>>8) | (throttle<<8);
    Serial.print("Dshot throttle: ");
    Serial.println(throttle);
      
    display.clearDisplay();
    display.setCursor(0, 20);
    display.write("Dshot OK");
    display.setCursor(0, 36);
    sprintf(msg, "%d", throttle);
    display.write(msg);
    display.display();
    delay(1500);
    display.clearDisplay();
    display.display();
  }

}



void test_led() {
  bool frame_started = 0;
  uint32_t ledbytes;
  timing_conf bit_timing;

  bit_timing.idle = 60;
  bit_timing.min_bits = 24;
  bit_timing.max_high = 90;
  bit_timing.min_high = 70;
  bit_timing.max_low = 50;
  bit_timing.min_low = 30;

  bit_timing.idle = 60;
  bit_timing.max_high = 100;
  bit_timing.min_high = 1;
  bit_timing.max_low = 12;
  bit_timing.min_low = 8;
  bit_timing.min_bits = 16;

  display.clearDisplay();

  ledbytes = timing_to_bits(bit_timing);

  if (ledbytes) {   
      
    display.clearDisplay();
    display.setCursor(0, 20);
    display.write("LED");
    display.setCursor(0, 36);
    sprintf(msg, "%#08x", ledbytes);
    display.write(msg);
    display.display();
    delay(1500);
    display.clearDisplay();
    display.display();
  }


  if (debug_code) {
    Serial.print("debug_code: ");
    Serial.println(debug_code);
  }

}

uint32_t timing_to_pwm(timing_conf conf) {
  uint32_t iobits = 0;
  int32_t cpu_mhz = (float) ESP.getCpuFreqMHz();
  int frame_start = 0;
  uint32_t duration_idle;
  uint32_t duration_high;
  uint32_t freq;

  for (int i=1; i < SAMPLE_COUNT / 2; i++) {
    uint32_t duration = ( (uint32_t)(timings[i]- timings[i - 1]) ) / (cpu_mhz / 10);

    // TODO comment this out
    /*
    if ( (duration != 0)  ) {
      Serial.println(duration);
    }
    */
    
    if (duration > conf.idle)  {
      duration_idle = duration;
      frame_start = i + 1;
      Serial.printf("found frame start at %d with duration %d\n", frame_start, duration);
      duration_high = ( (uint32_t)(timings[i + 1]- timings[i]) ) / (cpu_mhz / 10);
      break;
    }
  }

  if (frame_start == 0) {
    Serial.println("frame start not found in first half of bits");
    return(0);
  }

  if ( (duration_high >= conf.min_high) && (duration_high <= conf.max_high) ) {
    freq = 10e6 / (20000 + duration_idle);
    Serial.printf("value: %d, freq: %d\n", duration_high, freq);
  } else {
    Serial.printf("invalid pulse length %d\n", duration_high);  
  }

  duration_high = duration_high / 10;
  iobits = freq | ( duration_high << 16);
  return iobits;

}

void test_servo() {
  uint32_t iobits = 0;
  uint16_t freq;
  uint16_t value;

  timing_conf bit_timing;
  bit_timing.idle = 50000;
  bit_timing.min_bits = 16;
  bit_timing.max_high = 25000;
  bit_timing.min_high = 7500;
  bit_timing.max_high = 25000;
  bit_timing.min_high = 7500;

  display.clearDisplay();

  iobits = timing_to_pwm(bit_timing);
  freq = iobits & 0xffff;
  value = iobits >> 16;  

  if (value > 0) {
    
    Serial.printf("pwm: %d @ %d Hz\n", value, freq);
      
    display.clearDisplay();
    display.setCursor(0, 20);
    sprintf(msg, "PWM %d", value);
    display.write(msg);
    display.setCursor(0, 36);
    sprintf(msg, "%d Hz", freq);
    display.write(msg);

    display.display();
    delay(1500);
    display.clearDisplay();
    display.display();
  }
}


void bzzz() {
#define BMP_HEIGHT 64
#define BMP_WIDTH  128 
  display.setCursor(0, 0);

  display.drawBitmap(0, 0,  bee_bmp, BMP_WIDTH, BMP_HEIGHT, 1);
  display.display();
}

void test_buzz() {

  pinMode(IO_PROBE, INPUT_PULLUP);

  display.clearDisplay();


  int oldtime = millis();
  int oldval = digitalRead(IO_PROBE);
  int count_100ms = 0;

  for (int i = 0; i < 50; i++) {
    int newval = digitalRead(IO_PROBE);
    if (newval != oldval) {
      int newtime = millis();
      int duration = newtime - oldtime;
      if ( (duration > 50) && (duration < 150) ) {
        /*
        bzzz();
        display.invertDisplay(newval);
        display.display();
        */
        count_100ms++;
      } else {
        count_100ms = 0;
      }
      oldval = newval;
      oldtime = newtime;
    }
    delay(10);
  }


  if (count_100ms > 3) {

    Serial.printf("Found %d buzzer pulses in 500 ms\n", count_100ms);
    bzzz();
    delay(1500);
    display.invertDisplay(false);
    display.clearDisplay();
    display.display();
  }
  pinMode(IO_PROBE, INPUT_PULLDOWN);

}


void setup() {
  Serial.begin(115200);

  /*
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  msp.begin(Serial2);
  Serial2.end();
  */


  // Start I2C Communication SDA = 5 and SCL = 4 on Wemos Lolin32 ESP32 with built-in SSD1306 OLED
  Wire.begin(5, 4);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false, false)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();

  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  pinMode(IO_PROBE, INPUT_PULLDOWN);
  // pinMode(19, OUTPUT);

 display.setRotation(2);
/*
uint8_t[2] uc;
{
uc[0] = 0; // command
uc[1] = 0xa0;
i2cWrite(oled_addr, uc, 2);
uc[1] = 0xc0;
i2cWrite(oled_addr, uc, 2);
}
*/

}


void loop() {
  display.clearDisplay();
  
  get_timings(200);

  /*
  for (int i = 0; i < SAMPLE_COUNT - 1; i++) {
    Serial.println(timings[i]);
  */

  // test_led();

  test_dshot();
  test_servo();
  
  test_uart();
  test_sbus();
  

  test_buzz();
  delay(50);
  
}

