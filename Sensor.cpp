#include <math.h>
#include <climits>
#include "hardware/irq.h"
#include "Sensor.hpp"
#include "lighthouse.pio.h"
#include <cstdio>

////////////////////////////////////////////////////////////////////////////////////////////////////
// STATICS
////////////////////////////////////////////////////////////////////////////////////////////////////
Sensor* Sensor::pio_sensors[][NUM_PIO_STATE_MACHINES] = { { nullptr, nullptr, nullptr, nullptr }, { nullptr, nullptr, nullptr, nullptr } };
uint8_t Sensor::pio_claimed_sms[] = { 0x0, 0x0 };
uint Sensor::sens_offset = 0;;

////////////////////////////////////////////////////////////////////////////////////////////////////
void Sensor::pio0_interrupt_callback() {
  //Go through each of encoders on this PIO to see which triggered this interrupt
  for(uint8_t sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
    if(pio_sensors[0][sm] != nullptr) { 
      pio_sensors[0][sm]->check_for_transition(); 
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Sensor::pio1_interrupt_callback() {
  //Go through each of encoders on this PIO to see which triggered this interrupt
  for(uint8_t sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
    if(pio_sensors[1][sm] != nullptr) { 
      pio_sensors[1][sm]->check_for_transition(); 
    }
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTRUCTORS / DESTRUCTOR
////////////////////////////////////////////////////////////////////////////////////////////////////
Sensor::Sensor(PIO pio, uint8_t pin, uint8_t sideset_pin,
               uint16_t freq_divider) :
  sens_pio(pio), pin(pin), sideset_pin(sideset_pin),
  freq_divider(freq_divider) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Sensor::~Sensor() {
  //Clean up our use of the SM associated with this encoder
  lighthouse_program_release(sens_pio, sens_sm);
  uint index = pio_get_index(sens_pio);
  pio_sensors[index][sens_sm] = nullptr;
  pio_claimed_sms[index] &= ~(1u << sens_sm);

  //If there are no more SMs using the encoder program, then we can remove it from the PIO
  if(pio_claimed_sms[index] == 0) {
    pio_remove_program(sens_pio, &lighthouse_program, sens_offset);
  }
}



////////////////////////////////////////////////////////////////////////////////////////////////////
// METHODS
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Sensor::init() {
  initialised = false;

  //Is the pin we want to use actually valid?
  if(pin < NUM_BANK0_GPIOS) {

    sens_sm = pio_claim_unused_sm(sens_pio, true);
    uint pio_idx = pio_get_index(sens_pio);

    //Is this the first time using a sensor on this PIO?      
    if(pio_claimed_sms[pio_idx] == 0) {
      //Add the program to the PIO memory
      sens_offset = pio_add_program(sens_pio, &lighthouse_program);
    }

    //Init the program on this sm and enable the appropriate interrupt
    lighthouse_program_init(sens_pio, sens_sm, sens_offset, pin, sideset_pin, freq_divider);
    //hw_set_bits(&sens_pio->inte0, PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << sens_sm);
    pio_set_irq0_source_enabled(sens_pio, (pio_interrupt_source)(PIO_INTR_SM0_RXNEMPTY_LSB + sens_sm), true);

    if(pio_idx == 0) {
      irq_add_shared_handler(PIO0_IRQ_0, pio0_interrupt_callback, sens_sm);
      irq_set_enabled(PIO0_IRQ_0, true);
    }
    else {
      irq_add_shared_handler(PIO1_IRQ_0, pio1_interrupt_callback, sens_sm);
      irq_set_enabled(PIO1_IRQ_0, true);
    }

    //Keep a record of this sensor for the interrupt callback
    pio_sensors[pio_idx][sens_sm] = this;
    pio_claimed_sms[pio_idx] |= 1u << sens_sm;

    //Start the PIO program on the SM
    lighthouse_program_start(sens_pio, sens_sm);

    initialised = true;
  }
  return initialised;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Sensor::start() {
  if(initialised) {

    // printf("%d\n", sens_pio->inte0);

    // uint pio_idx = pio_get_index(sens_pio);
    // pio_set_irq0_source_enabled(sens_pio, (pio_interrupt_source)(PIO_INTR_SM0_RXNEMPTY_LSB + sens_sm), true);
    //   if(pio_idx == 0) {
    //     irq_add_shared_handler(PIO0_IRQ_0, pio0_interrupt_callback, sens_sm);
    //     irq_set_enabled(PIO0_IRQ_0, true);
    //     printf("Irq en, %d, Sm = %d, PIO = %d\n", PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << sens_sm, sens_sm, pio_idx);
    //   }
    //   else {
    //     irq_add_shared_handler(PIO1_IRQ_0, pio1_interrupt_callback, sens_sm);
    //     irq_set_enabled(PIO1_IRQ_0, true);
    //   }

    // printf("%d\n", sens_pio->inte0);

    // // 
    // // if(pio_idx == 0) {
    // //   irq_set_exclusive_handler(PIO0_IRQ_0, pio0_interrupt_callback);
    // //   irq_set_enabled(PIO0_IRQ_0, true);
    // // }
    // // else {
    // //   irq_set_exclusive_handler(PIO1_IRQ_0, pio1_interrupt_callback);
    // //   irq_set_enabled(PIO1_IRQ_0, true);
    // // }

    // //Start the PIO program on the SM
    // lighthouse_program_start(sens_pio, sens_sm);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t Sensor::millis() {
    return to_us_since_boot(get_absolute_time()) / 1000;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t Sensor::get_received(uint32_t& bufferCount) {
  uint32_t received = 0;

  if(readIndex != writeIndex) {
    received = receivedBuffer[readIndex];
    readIndex++;
    if(readIndex >= BUFFER)
      readIndex = 0;
    bufferCount = writeIndex - readIndex;
  }
  
  return received;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void Sensor::check_for_transition() {
  while(sens_pio->ints0 & (PIO_IRQ0_INTS_SM0_RXNEMPTY_BITS << sens_sm)) {    
    uint32_t received = pio_sm_get(sens_pio, sens_sm);
    if(received > 0) {
      receivedBuffer[writeIndex] = received;
      writeIndex++;
      if(writeIndex >= BUFFER)
        writeIndex = 0;
      
      last_on = millis();
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////