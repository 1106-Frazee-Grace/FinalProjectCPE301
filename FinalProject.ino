// Swamp Cooler - CPE301 Final Project
// Jakob Kelley & Grace Frazee

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <RTClib.h>

// ====== HARDWARE CONFIGURATION ======
// Pin assignments should be verified with the physical wiring configuration.

// LED indicators (controlled via PORTA bits: digital pins 22–25 correspond to PORTA0–3)
const uint8_t LED_GREEN_PIN  = 22; // IDLE state
const uint8_t LED_RED_PIN    = 23; // ERROR state
const uint8_t LED_YELLOW_PIN = 24; // DISABLED state
const uint8_t LED_BLUE_PIN   = 25; // RUNNING state

// Button inputs using interrupt-capable pins
// Pin 2 -> INT4 (used for START button ISR)
// Pin 3 -> INT5 (used for STOP button ISR)
// Pin 21 -> INT2 (used for RESET button ISR)
const uint8_t START_BUTTON_PIN = 2;
const uint8_t STOP_BUTTON_PIN  = 3;
const uint8_t RESET_BUTTON_PIN = 21;

// Fan motor PWM control pin
// analogWrite() is permitted for PWM fan control per project rules.
const uint8_t FAN_PWM_PIN = 11;

// Stepper motor configuration (Stepper library usage permitted)
const uint8_t STEPPER_PIN_1 = 4;
const uint8_t STEPPER_PIN_2 = 5;
const uint8_t STEPPER_PIN_3 = 6;
const uint8_t STEPPER_PIN_4 = 7;
const int STEPPER_STEPS_PER_REV = 200;
Stepper ventStepper(STEPPER_STEPS_PER_REV, STEPPER_PIN_1, STEPPER_PIN_2, STEPPER_PIN_3, STEPPER_PIN_4);

// DHT11 temperature and humidity sensor
const uint8_t DHT_PIN = 8;
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

// Real-time clock module
RTC_DS3231 rtc;

// LCD display (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 13, A0, A1, A2, A3);

// Water-level sensor input through ADC (manual ADC register read)
// ADC7 (Analog pin A7) selected to avoid LCD pin conflicts.
const uint8_t WATER_LEVEL_ADC_CHANNEL = 7;

// ====== CONTROL CONSTANTS ======
const uint16_t WATER_LOW_THRESHOLD_ADC = 300; // Adjust according to calibration (0–1023)
const float TEMP_HIGH_THRESHOLD_C = 28.0;     // Temperature to start fan
const float TEMP_LOW_THRESHOLD_C  = 24.0;     // Temperature to stop fan
const unsigned long LCD_UPDATE_INTERVAL_MS = 60000UL; // 1 minute in milliseconds

// ====== SYSTEM STATES ======
enum CoolerState { DISABLED, IDLE, RUNNING, ERROR_STATE };
volatile CoolerState currentState = DISABLED;

// ====== INTERRUPT FLAGS ======
volatile bool start_pressed = false;
volatile bool stop_pressed  = false;
volatile bool reset_pressed = false;

// Vent position tracking for stepper motor
volatile long ventPositionSteps = 0;
long lastReportedVentPositionSteps = 0;

// LCD update timing control
unsigned long lastLcdUpdateMillis = 0;

// ====== UART (USART0) CONFIGURATION ======
// Provides serial communication through direct register access.
void uart_init(unsigned long baud){
  uint16_t ubrr = (uint16_t)(F_CPU/16UL/baud - 1);
  UBRR0H = (ubrr >> 8);
  UBRR0L = ubrr & 0xFF;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_send_char(char c){
  while(!(UCSR0A & (1 << UDRE0)));
  UDR0 = c;
}

void uart_send_string(const char* s) {
  while(*s) uart_send_char(*s++);
}

void uart_send_decimal(unsigned long val){
  char buf[12];
  int n = 0;
  if(val == 0){ uart_send_char('0'); return;}
  while(val > 0 && n < (int)sizeof(buf)-1){
    buf[n++] = '0' + (val % 10);
    val /= 10;
  }
  for(int i = n-1; i >= 0; --i) uart_send_char(buf[i]);
}

void uart_newline() {
  uart_send_char('\r');
  uart_send_char('\n');
}

// Sends timestamped messages using the RTC
void log_event(const char* msg){
  DateTime now = rtc.now();
  char buf[64];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u: %s",
           now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second(), msg);
  uart_send_string(buf);
  uart_newline();
}

// ====== ADC (Manual Register-Based) ======
// Reads analog input channel (0–15) using direct register access.
uint16_t adc_read_channel(uint8_t channel){
  ADMUX = (1 << REFS0) | (channel & 0x0F);
  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

// ====== LED PORT INITIALIZATION ======
// LED pins are configured as outputs via direct DDR register manipulation.
void init_led_pins(){
  DDRA |= (1 << DDA0) | (1 << DDA1) | (1 << DDA2) | (1 << DDA3);
  PORTA &= ~((1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3));
}

// LED control macros for individual state indicators
inline void led_set_green(bool on){ if (on) PORTA |= (1 << PA0); else PORTA &= ~(1 << PA0); }
inline void led_set_red(bool on){ if (on) PORTA |= (1 << PA1); else PORTA &= ~(1 << PA1); }
inline void led_set_yellow(bool on){ if (on) PORTA |= (1 << PA2); else PORTA &= ~(1 << PA2); }
inline void led_set_blue(bool on){ if (on) PORTA |= (1 << PA3); else PORTA &= ~(1 << PA3); }

// ====== FAN CONFIGURATION ======
// Fan pin configured via DDRB; PWM control handled using analogWrite().
void init_fan_pin(){
  DDRB |= (1 << DDB5);
  analogWrite(FAN_PWM_PIN, 0);
}

// ====== INTERRUPT SERVICE ROUTINES ======
void IRAM_ATTR start_isr(){ start_pressed = true;}
void IRAM_ATTR stop_isr(){ stop_pressed = true;}
void IRAM_ATTR reset_isr(){ reset_pressed = true;}

// ====== LED UPDATE ROUTINE ======
void update_leds_for_state(CoolerState s){
  switch (s) {
    case DISABLED:
      led_set_yellow(true); led_set_green(false); led_set_red(false); led_set_blue(false);
      break;
    case IDLE:
      led_set_yellow(false); led_set_green(true); led_set_red(false); led_set_blue(false);
      break;
    case RUNNING:
      led_set_yellow(false); led_set_green(false); led_set_red(false); led_set_blue(true);
      break;
    case ERROR_STATE:
      led_set_yellow(false); led_set_green(false); led_set_red(true); led_set_blue(false);
      break;
  }
}

// Handles state transitions and logs the event.
void transition_to_state(CoolerState newState){
  if(newState == currentState) return;
  currentState = newState;
  switch(newState){
    case DISABLED:     log_event("STATE -> DISABLED"); break;
    case IDLE:         log_event("STATE -> IDLE"); break;
    case RUNNING:      log_event("STATE -> RUNNING"); break;
    case ERROR_STATE:  log_event("STATE -> ERROR"); break;
  }
  update_leds_for_state(newState);
}

// ====== SYSTEM INITIALIZATION ======
void setup(){
  uart_init(9600);

  if (!rtc.begin()){
    uart_send_string("RTC: NOT FOUND");
    uart_newline();
  }

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Swamp Cooler Init");

  dht.begin();
  ventStepper.setSpeed(30);

  init_led_pins();
  init_fan_pin();

  // Configure button inputs using direct port access and enable pull-ups.
  DDRE &= ~((1 << DDE4) | (1 << DDE5));
  PORTE |= (1 << PE4) | (1 << PE5);
  DDRB &= ~(1 << DDB5);
  PORTB |= (1 << PB5);

  // Interrupt configuration for buttons
  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), start_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stop_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), reset_isr, FALLING);

  transition_to_state(DISABLED);
  log_event("SYSTEM BOOT");
  lastLcdUpdateMillis = millis();
}

// ====== EVENT LOGGING ROUTINES ======
void log_motor_event(const char* action){
  char buf[64];
  DateTime now = rtc.now();
  snprintf(buf, sizeof(buf), "MOTOR %s at %04u-%02u-%02u %02u:%02u:%02u",
           action, now.year(), now.month(), now.day(),
           now.hour(), now.minute(), now.second());
  uart_send_string(buf);
  uart_newline();
}

// ====== FAN CONTROL ROUTINES ======
void fan_set_on(uint8_t pwm = 200){ analogWrite(FAN_PWM_PIN, pwm); }
void fan_set_off(){ analogWrite(FAN_PWM_PIN, 0); }

// ====== LCD DISPLAY ROUTINE ======
void update_lcd(float tempC, float hum, uint16_t waterADC) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.print(tempC,1);
  lcd.print("C H:");
  lcd.print(hum,0);
  lcd.setCursor(0,1);
  lcd.print("Water:");
  lcd.print(waterADC);
  lcd.print(" S:");
  lcd.print(currentState==RUNNING ? "RUN" : currentState==IDLE ? "IDLE" : currentState==ERROR_STATE ? "ERR" : "OFF");
}

// ====== MAIN CONTROL LOOP ======
void loop(){
  // Button handling logic
  if(start_pressed) {
    start_pressed = false;
    if(currentState == DISABLED) {
      transition_to_state(IDLE);
    }else if (currentState == IDLE) {
      transition_to_state(RUNNING);
      log_event("User START pressed -> forced RUNNING");
    }else {
      log_event("User START pressed (no-op)");
    }
  }

  if(stop_pressed){
    stop_pressed = false;
    fan_set_off();
    transition_to_state(DISABLED);
    log_event("User STOP pressed -> DISABLED");
    log_motor_event("OFF (stop btn)");
  }

  if(reset_pressed){
    reset_pressed = false;
    uint16_t waterADC = adc_read_channel(WATER_LEVEL_ADC_CHANNEL);
    if(waterADC >= WATER_LOW_THRESHOLD_ADC) {
      transition_to_state(IDLE);
      log_event("User RESET pressed -> IDLE");
    }else {
      log_event("User RESET pressed -> water still low, remain ERROR");
    }
  }

  // Sensor monitoring and LCD update
  if(currentState != DISABLED){
    unsigned long nowms = millis();
    if(nowms - lastLcdUpdateMillis >= LCD_UPDATE_INTERVAL_MS) {
      lastLcdUpdateMillis = nowms;

      float hum = dht.readHumidity();
      float tempC = dht.readTemperature();
      if(isnan(hum) || isnan(tempC)){
        uart_send_string("DHT read failed");
        uart_newline();
        hum = 0;
        tempC = 0;
      }

      uint16_t waterADC = adc_read_channel(WATER_LEVEL_ADC_CHANNEL);
      update_lcd(tempC, hum, waterADC);

      char buf[128];
      DateTime dt = rtc.now();
      snprintf(buf, sizeof(buf),
               "%04u-%02u-%02u %02u:%02u:%02u SENSORS T=%.1fC H=%.1f%% WaterADC=%u",
               dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second(),
               tempC, hum, waterADC);
      uart_send_string(buf);
      uart_newline();

      if(waterADC < WATER_LOW_THRESHOLD_ADC) {
        fan_set_off();
        transition_to_state(ERROR_STATE);
        log_event("Transition -> ERROR due to low water");
        log_motor_event("OFF (low water)");
      }else{
        if(currentState == IDLE && tempC >= TEMP_HIGH_THRESHOLD_C) {
          transition_to_state(RUNNING);
          fan_set_on(200);
          log_motor_event("ON (temp high)");
        }else if(currentState == RUNNING && tempC <= TEMP_LOW_THRESHOLD_C) {
          fan_set_off();
          transition_to_state(IDLE);
          log_motor_event("OFF (temp low)");
        }
      }
    }

    // Vent control via potentiometer input on ADC6
    uint16_t potVal = adc_read_channel(6);
    long targetSteps = map(potVal, 0, 1023, -1000, 1000);
    if(targetSteps != ventPositionSteps) {
      long delta = targetSteps - ventPositionSteps;
      if(delta > 0){
        long stepAmount = (delta > 10) ? 10 : delta;
        ventStepper.step((int)stepAmount);
        ventPositionSteps += stepAmount;
      }else{
        long stepAmount = (-delta > 10) ? -10 : delta;
        ventStepper.step((int)stepAmount);
        ventPositionSteps += stepAmount;
      }

      char vbuf[64];
      snprintf(vbuf, sizeof(vbuf), "VENT POS %ld", ventPositionSteps);
      uart_send_string(vbuf);
      uart_newline();

      DateTime now = rtc.now();
      char logbuf[80];
      snprintf(logbuf, sizeof(logbuf),
               "%04u-%02u-%02u %02u:%02u:%02u VENT %ld",
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second(),
               ventPositionSteps);
      uart_send_string(logbuf);
      uart_newline();
    }
  }

  if(currentState == DISABLED){
  fan_set_off();
  }
}