#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "lighthouse.pio.h"
#include "simulated_lh.pio.h"
#include "Sensor.hpp"
#include "hardware/pwm.h"
#include "math.h"

const uint SENSOR1_PIN = 0;
const uint SENSOR2_PIN = 2;
const uint SENSOR3_PIN = 5;
const uint SENSOR4_PIN = 7;

const uint SENSOR1_DBG = 1;
const uint SENSOR2_DBG = 3;
const uint SENSOR3_DBG = 4;
const uint SENSOR4_DBG = 6;

static const uint16_t FREQ_DIVIDER            = 1; 

Sensor sensor1(pio0, SENSOR1_PIN, SENSOR1_DBG, FREQ_DIVIDER);
Sensor sensor2(pio0, SENSOR2_PIN, SENSOR2_DBG, FREQ_DIVIDER);
Sensor sensor3(pio0, SENSOR3_PIN, SENSOR3_DBG, FREQ_DIVIDER);
Sensor sensor4(pio0, SENSOR4_PIN, SENSOR4_DBG, FREQ_DIVIDER);

static const bool SIMULATED_OUT_ENABLED      = false;
const uint SIMULATED_OUT_PIN = 6;

static constexpr float US_TO_LH_TICK = 48.0f;

void set_led(uint8_t r, uint8_t g, uint8_t b) {
  // gamma correct the provided 0-255 brightness value onto a
  // 0-65535 range for the pwm counter
  static const float gamma = 2.8;

  uint16_t value;

  // red
  value = (uint16_t)(pow((float)(r) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(TINY2040_LED_R_PIN, value);

  // green
  value = (uint16_t)(pow((float)(g) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(TINY2040_LED_G_PIN, value);

  // blue
  value = (uint16_t)(pow((float)(b) / 255.0f, gamma) * 65535.0f + 0.5f);
  pwm_set_gpio_level(TINY2040_LED_B_PIN, value);
}

int main() {

  stdio_init_all();

  //sleep_ms(10000);

  // gpio_init(TINY2040_LED_R_PIN);
  // gpio_set_dir(TINY2040_LED_R_PIN, GPIO_OUT);
  // gpio_put(TINY2040_LED_R_PIN, PICO_DEFAULT_LED_PIN_INVERTED);

  // gpio_init(TINY2040_LED_G_PIN);
  // gpio_set_dir(TINY2040_LED_G_PIN, GPIO_OUT);
  // gpio_put(TINY2040_LED_G_PIN, PICO_DEFAULT_LED_PIN_INVERTED);

  // gpio_init(TINY2040_LED_B_PIN);
  // gpio_set_dir(TINY2040_LED_B_PIN, GPIO_OUT);
  // gpio_put(TINY2040_LED_B_PIN, PICO_DEFAULT_LED_PIN_INVERTED);

  // setup the rgb led for pwm control
  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_output_polarity(&cfg, true, true);

  // red
  pwm_set_wrap(pwm_gpio_to_slice_num(TINY2040_LED_R_PIN), 65535);
  pwm_init(pwm_gpio_to_slice_num(TINY2040_LED_R_PIN), &cfg, true);
  gpio_set_function(TINY2040_LED_R_PIN, GPIO_FUNC_PWM);

  // green
  pwm_set_wrap(pwm_gpio_to_slice_num(TINY2040_LED_G_PIN), 65535);
  pwm_init(pwm_gpio_to_slice_num(TINY2040_LED_G_PIN), &cfg, true);
  gpio_set_function(TINY2040_LED_G_PIN, GPIO_FUNC_PWM);

  // blue
  pwm_set_wrap(pwm_gpio_to_slice_num(TINY2040_LED_B_PIN), 65535);
  pwm_init(pwm_gpio_to_slice_num(TINY2040_LED_B_PIN), &cfg, true);
  gpio_set_function(TINY2040_LED_B_PIN, GPIO_FUNC_PWM);

  set_led(127, 127, 255);

  sensor1.init();
  sensor2.init();
  sensor3.init();
  sensor4.init();

  if(SIMULATED_OUT_ENABLED) {
      //Set up the quadrature encoder output
      PIO pio = pio1;
      uint offset = pio_add_program(pio, &simulated_lh_out_program);
      uint sm = pio_claim_unused_sm(pio, true);
      simulated_lh_out_program_init(pio, sm, offset, SIMULATED_OUT_PIN);
  }

  
  int lastAxis[4] = { 0, 0, 0, 0 };
  int lastData[4] = { 0, 0, 0, 0 };
  int lastSkip[4] = { 0, 0, 0, 0 };

  float lastX[4] = { 0, 0, 0, 0 };
  float lastY[4] = { 0, 0, 0, 0 };

  bool newData[4] = { false, false, false, false };

  while (1) {
    //gpio_put(LED_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);
    //sleep_ms(250);
    //gpio_put(LED_PIN, PICO_DEFAULT_LED_PIN_INVERTED);
    //puts("Hello World\n");
    //sleep_ms(1000);

    uint32_t bufferCount;

    uint32_t received = sensor1.get_received(bufferCount);
    if(received > 0) {
      uint32_t istart = (received & 0xffff0000) >> 16;
      uint32_t iend = (received & 0xffff);
      float start = (float)((received & 0xffff0000) >> 16) * 15.0f * 0.008f;
      float end = (float)(received & 0xffff) * 15.0f * 0.008f;
      uint32_t ilength = iend - istart;
      float length = end - start;
      float tick_length = length * US_TO_LH_TICK;

      if(istart == 0) {
        int sync_data = (int)((tick_length - 2751) / 500.0f);
        lastAxis[0] = sync_data & 0b001;
        lastData[0] = (sync_data & 0b010) >> 1;
        lastSkip[0] = (sync_data & 0b100) >> 2;
        //printf("\n[%d] B-Sync = %.1fuS (%d)", bufferCount, end, iend);
        //printf("\nB-Sync = %.1fuS (%d) [%d %d %d]", end, iend, lastAxis, lastData, lastSkip);        
      }
      else {        
        if(length > 50.0f) {
          int sync_data = (int)((tick_length - 2751) / 500.0f);
          lastAxis[0] = sync_data & 0b001;
          //printf("\t[%d] C-Sync @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          //printf("\tC-Sync @ %.1fuS (%d) = %.1fuS (%d) [%d]", start, istart, length, ilength, sync_data);
        }
        else {
          //printf("\t[%d] Sweep @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          if(lastAxis[0]) {
            //printf("\tSweep Y @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastY[0] = (start + end) / 2.0f;
            newData[0] = true;
          }
          else {
            //printf("\tSweep X @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastX[0] = (start + end) / 2.0f;
          }
        }
      }
    }

    received = sensor2.get_received(bufferCount);
    if(received > 0) {
      uint32_t istart = (received & 0xffff0000) >> 16;
      uint32_t iend = (received & 0xffff);
      float start = (float)((received & 0xffff0000) >> 16) * 15.0f * 0.008f;
      float end = (float)(received & 0xffff) * 15.0f * 0.008f;
      uint32_t ilength = iend - istart;
      float length = end - start;
      float tick_length = length * US_TO_LH_TICK;

      if(istart == 0) {
        int sync_data = (int)((tick_length - 2751) / 500.0f);
        lastAxis[1] = sync_data & 0b001;
        lastData[1] = (sync_data & 0b010) >> 1;
        lastSkip[1] = (sync_data & 0b100) >> 2;
        //printf("\n[%d] B-Sync = %.1fuS (%d)", bufferCount, end, iend);
        //printf("\nB-Sync = %.1fuS (%d) [%d %d %d]", end, iend, lastAxis, lastData, lastSkip);        
      }
      else {        
        if(length > 50.0f) {
          int sync_data = (int)((tick_length - 2751) / 500.0f);
          lastAxis[1] = sync_data & 0b001;
          //printf("\t[%d] C-Sync @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          //printf("\tC-Sync @ %.1fuS (%d) = %.1fuS (%d) [%d]", start, istart, length, ilength, sync_data);
        }
        else {
          //printf("\t[%d] Sweep @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          if(lastAxis[1]) {
            //printf("\tSweep Y @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastY[1] = (start + end) / 2.0f;
            newData[1] = true;
          }
          else {
            //printf("\tSweep X @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastX[1] = (start + end) / 2.0f;
          }
        }
      }
    }

    received = sensor3.get_received(bufferCount);
    if(received > 0) {
      uint32_t istart = (received & 0xffff0000) >> 16;
      uint32_t iend = (received & 0xffff);
      float start = (float)((received & 0xffff0000) >> 16) * 15.0f * 0.008f;
      float end = (float)(received & 0xffff) * 15.0f * 0.008f;
      uint32_t ilength = iend - istart;
      float length = end - start;
      float tick_length = length * US_TO_LH_TICK;

      if(istart == 0) {
        int sync_data = (int)((tick_length - 2751) / 500.0f);
        lastAxis[2] = sync_data & 0b001;
        lastData[2] = (sync_data & 0b010) >> 1;
        lastSkip[2] = (sync_data & 0b100) >> 2;
        //printf("\n[%d] B-Sync = %.1fuS (%d)", bufferCount, end, iend);
        //printf("\nB-Sync = %.1fuS (%d) [%d %d %d]", end, iend, lastAxis, lastData, lastSkip);        
      }
      else {        
        if(length > 50.0f) {
          int sync_data = (int)((tick_length - 2751) / 500.0f);
          lastAxis[2] = sync_data & 0b001;
          //printf("\t[%d] C-Sync @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          //printf("\tC-Sync @ %.1fuS (%d) = %.1fuS (%d) [%d]", start, istart, length, ilength, sync_data);
        }
        else {
          //printf("\t[%d] Sweep @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          if(lastAxis[2]) {
            //printf("\tSweep Y @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastY[2] = (start + end) / 2.0f;
            newData[2] = true;
          }
          else {
            //printf("\tSweep X @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastX[2] = (start + end) / 2.0f;
          }
        }
      }
    }

    received = sensor4.get_received(bufferCount);
    if(received > 0) {
      uint32_t istart = (received & 0xffff0000) >> 16;
      uint32_t iend = (received & 0xffff);
      float start = (float)((received & 0xffff0000) >> 16) * 15.0f * 0.008f;
      float end = (float)(received & 0xffff) * 15.0f * 0.008f;
      uint32_t ilength = iend - istart;
      float length = end - start;
      float tick_length = length * US_TO_LH_TICK;

      if(istart == 0) {
        int sync_data = (int)((tick_length - 2751) / 500.0f);
        lastAxis[3] = sync_data & 0b001;
        lastData[3] = (sync_data & 0b010) >> 1;
        lastSkip[3] = (sync_data & 0b100) >> 2;
        //printf("\n[%d] B-Sync = %.1fuS (%d)", bufferCount, end, iend);
        //printf("\nB-Sync = %.1fuS (%d) [%d %d %d]", end, iend, lastAxis, lastData, lastSkip);        
      }
      else {        
        if(length > 50.0f) {
          int sync_data = (int)((tick_length - 2751) / 500.0f);
          lastAxis[3] = sync_data & 0b001;
          //printf("\t[%d] C-Sync @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          //printf("\tC-Sync @ %.1fuS (%d) = %.1fuS (%d) [%d]", start, istart, length, ilength, sync_data);
        }
        else {
          //printf("\t[%d] Sweep @ %.1fuS (%d) = %.1fuS (%d)", bufferCount, start, istart, length, ilength);
          if(lastAxis[3]) {
            //printf("\tSweep Y @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastY[3] = (start + end) / 2.0f;
            newData[3] = true;
          }
          else {
            //printf("\tSweep X @ %.1fuS (%d) = %.1fuS (%d)", start, istart, length, ilength);
            lastX[3] = (start + end) / 2.0f;
          }
        }
      }
    }

    // if(Sensor::millis() - sensor1.last_on_time() < 20) {
    //   gpio_put(TINY2040_LED_R_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);
    // }
    // else {
    //   gpio_put(TINY2040_LED_R_PIN, PICO_DEFAULT_LED_PIN_INVERTED);
    // }

    // if(Sensor::millis() - sensor2.last_on_time() < 20) {
    //   gpio_put(TINY2040_LED_G_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);
    // }
    // else {
    //   gpio_put(TINY2040_LED_G_PIN, PICO_DEFAULT_LED_PIN_INVERTED);
    // }l

    // if(Sensor::millis() - sensor3.last_on_time() < 20) {
    //   gpio_put(TINY2040_LED_B_PIN, !PICO_DEFAULT_LED_PIN_INVERTED);
    // }
    // else {
    //   gpio_put(TINY2040_LED_B_PIN, PICO_DEFAULT_LED_PIN_INVERTED);
    // }
    if(newData[0] || newData[1] || newData[2] || newData[3]) {
      float xAngle1 = ((lastX[0] - 4000.0f) / 4000.0f) * 90.0f;
      float yAngle1 = ((lastY[0] - 4000.0f) / 4000.0f) * 90.0f;
      float xAngle2 = ((lastX[1] - 4000.0f) / 4000.0f) * 90.0f;
      float yAngle2 = ((lastY[1] - 4000.0f) / 4000.0f) * 90.0f;
      float xAngle3 = ((lastX[2] - 4000.0f) / 4000.0f) * 90.0f;
      float yAngle3 = ((lastY[2] - 4000.0f) / 4000.0f) * 90.0f;
      float xAngle4 = ((lastX[3] - 4000.0f) / 4000.0f) * 90.0f;
      float yAngle4 = ((lastY[3] - 4000.0f) / 4000.0f) * 90.0f;
      printf("%f, %f, %f, %f, %f, %f, %f, %f\n", xAngle1, yAngle1, xAngle2, yAngle2, xAngle3, yAngle3, xAngle4, yAngle4);
      
      set_led(((lastX[0] - 4000.0f) / 22) + 127.0f, ((lastY[0] - 4000.0f) / 22) + 127.0f, 255.0f);
      newData[0] = false;
      newData[1] = false;
      newData[2] = false;
      newData[3] = false;
    }
  }
}