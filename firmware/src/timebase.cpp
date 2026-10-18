#include "timebase.h"

#include <Arduino.h>
#include <nrf_soc.h>
#include <nrf_sdm.h>
#include "config.h"

static tick_cb_t s_tick_cb = nullptr;

void timebase_begin(tick_cb_t tick_cb) {
  s_tick_cb = tick_cb;
  NRF_TIMER4->TASKS_STOP = 1;
  NRF_TIMER4->TASKS_CLEAR = 1;
  NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER4->PRESCALER = 4;  // 16 MHz / 2^4 = 1 MHz
  NRF_TIMER4->SHORTS = 0;
  NRF_TIMER4->CC[TB_CC_TICK] = SAMPLE_PERIOD_US;
  NRF_TIMER4->EVENTS_COMPARE[TB_CC_TICK] = 0;
  NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE4_Msk;
  // Ten sam priorytet co GPIOTE (3) – ISR halla i taktu nie wywłaszczają się.
  NVIC_SetPriority(TIMER4_IRQn, 3);
  NVIC_ClearPendingIRQ(TIMER4_IRQn);
  NVIC_EnableIRQ(TIMER4_IRQn);
  NRF_TIMER4->TASKS_START = 1;
}

static bool sd_enabled(void) {
  uint8_t en = 0;
  sd_softdevice_is_enabled(&en);
  return en != 0;
}

bool timebase_request_hfxo(void) {
  if (sd_enabled()) {
    if (sd_clock_hfclk_request() != NRF_SUCCESS) return false;
    uint32_t running = 0;
    for (int i = 0; i < 1000 && !running; i++) {
      sd_clock_hfclk_is_running(&running);
      delayMicroseconds(10);
    }
    return running != 0;
  }
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  for (int i = 0; i < 1000 && !NRF_CLOCK->EVENTS_HFCLKSTARTED; i++) delayMicroseconds(10);
  return NRF_CLOCK->EVENTS_HFCLKSTARTED != 0;
}

uint32_t timebase_now(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  NRF_TIMER4->TASKS_CAPTURE[TB_CC_NOW] = 1;
  uint32_t t = NRF_TIMER4->CC[TB_CC_NOW];
  __set_PRIMASK(primask);
  return t;
}

bool timebase_connect_capture(uint32_t arduino_pin, uint8_t cc, uint8_t ppi_ch) {
  uint32_t const pin = g_ADigitalPinMap[arduino_pin];
  int ch = -1;
  for (int i = 0; i < 8; i++) {
    uint32_t cfg = NRF_GPIOTE->CONFIG[i];
    if ((cfg & GPIOTE_CONFIG_MODE_Msk) == (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) &&
        ((cfg & (GPIOTE_CONFIG_PSEL_Msk | GPIOTE_CONFIG_PORT_Msk)) >> GPIOTE_CONFIG_PSEL_Pos) == pin) {
      ch = i;
      break;
    }
  }
  if (ch < 0) return false;
  uint32_t eep = (uint32_t)&NRF_GPIOTE->EVENTS_IN[ch];
  uint32_t tep = (uint32_t)&NRF_TIMER4->TASKS_CAPTURE[cc];
  if (sd_enabled()) {
    if (sd_ppi_channel_assign(ppi_ch, (const volatile void*)eep, (const volatile void*)tep) != NRF_SUCCESS)
      return false;
    return sd_ppi_channel_enable_set(1u << ppi_ch) == NRF_SUCCESS;
  }
  NRF_PPI->CH[ppi_ch].EEP = eep;
  NRF_PPI->CH[ppi_ch].TEP = tep;
  NRF_PPI->CHENSET = 1u << ppi_ch;
  return true;
}

extern "C" void TIMER4_IRQHandler(void) {
  if (NRF_TIMER4->EVENTS_COMPARE[TB_CC_TICK]) {
    NRF_TIMER4->EVENTS_COMPARE[TB_CC_TICK] = 0;
    uint32_t t = NRF_TIMER4->CC[TB_CC_TICK];
    uint32_t next = t + SAMPLE_PERIOD_US;
    NRF_TIMER4->CC[TB_CC_TICK] = next;
    // Gdyby ISR spóźnił się o cały okres (np. długa sekcja SoftDevice),
    // compare zostałby przegapiony i następny takt byłby za 71 min.
    NRF_TIMER4->TASKS_CAPTURE[TB_CC_NOW] = 1;
    uint32_t now = NRF_TIMER4->CC[TB_CC_NOW];
    if ((int32_t)(now - next) >= -2) {
      next = now + SAMPLE_PERIOD_US - (now - t) % SAMPLE_PERIOD_US;
      NRF_TIMER4->CC[TB_CC_TICK] = next;
    }
    if (s_tick_cb) s_tick_cb(t);
  }
  __DSB();
}
