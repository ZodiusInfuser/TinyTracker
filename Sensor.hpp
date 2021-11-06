#pragma once

#include "pico/stdlib.h"
#include "hardware/pio.h"

class Sensor {
  //--------------------------------------------------
  // Constants
  //--------------------------------------------------
public:
  static const uint16_t DEFAULT_FREQ_DIVIDER      = 1;    
  static const uint8_t PIN_UNUSED                 = UINT8_MAX;
private:
  static const uint32_t BUFFER = 5000;

  //--------------------------------------------------
  // Variables
  //--------------------------------------------------
private:
  const PIO sens_pio            = pio0;
  const uint8_t pin             = PIN_UNUSED;
  const uint8_t sideset_pin     = PIN_UNUSED;
  const uint16_t freq_divider   = DEFAULT_FREQ_DIVIDER;

  //--------------------------------------------------

  uint sens_sm         = 0;
  static uint sens_offset;

  uint32_t last_on = 0;

  bool initialised = false;

  //--------------------------------------------------
  // Statics
  //--------------------------------------------------
public:
  static Sensor* pio_sensors[NUM_PIOS][NUM_PIO_STATE_MACHINES];
  static uint8_t pio_claimed_sms[NUM_PIOS];
  static void pio0_interrupt_callback();
  static void pio1_interrupt_callback();


  //--------------------------------------------------
  // Constructors/Destructor
  //--------------------------------------------------
public:
  Sensor() {}
  Sensor(PIO pio, uint8_t pin, uint8_t sideset_pin,
         uint16_t freq_divider = DEFAULT_FREQ_DIVIDER);
  ~Sensor();


  //--------------------------------------------------
  // Methods
  //--------------------------------------------------
public:    
  bool init();
  void start();
  uint32_t last_on_time() { return last_on; }
  uint32_t get_received(uint32_t& bufferCount);

  static uint32_t millis();
private:
  void check_for_transition();
  volatile uint32_t receivedBuffer[BUFFER];
  volatile uint32_t readIndex = 0;
  volatile uint32_t writeIndex = 0;
};