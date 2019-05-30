
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

// Matlab includes
#include "BLDC_controller.h"           /* Model's header file */
#include "bldc.h"
#include "hallinterrupts.h"

volatile ELECTRICAL_PARAMS electrical_measurements;

#define DO_MEASUREMENTS

volatile int pwml = 0;
volatile int pwmr = 0;


extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;
extern uint8_t disablepoweroff;

uint32_t buzzerFreq     = 0;
uint32_t buzzerPattern  = 0;
uint32_t buzzerTimer    = 0;

uint8_t enable          = 0;
volatile long long bldc_counter = 0;
volatile unsigned  bldc_count = 0;

extern volatile HALL_PARAMS local_hall_params[2];


const int pwm_res       = 64000000 / 2 / PWM_FREQ; // = 2000

int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

int curl = 0;
//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  __disable_irq(); // but we want both values at the same time, without interferance
#ifdef HALL_INTERRUPTS
  unsigned long time = h_timer_hall.Instance->CNT;
  long long timerwraps_copy = timerwraps;
  now_us = ((timerwraps_copy<<16) + time) * 10;
#endif

  __enable_irq();


  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
    offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
    offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
    offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
    batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
//#ifdef DO_MEASUREMENTS
    electrical_measurements.batteryVoltage = batteryVoltage;
//#endif
  }


  // reduce to 8khz by running every other interrupt.
  // used to control freq.
  bldc_count++;
  if (!(bldc_count & 1)) {
    //return;
  }

  // used to measure freq
  bldc_counter++;

  float dclAmps = ((float)ABS(adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP);
  float dcrAmps = ((float)ABS(adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP);

  electrical_measurements.motors[0].dcAmps = dclAmps;
  electrical_measurements.motors[1].dcAmps = dcrAmps;

#ifdef DO_MEASUREMENTS
  electrical_measurements.motors[0].dcAmpsAvgAcc += ABS(adc_buffer.dcl - offsetdcl);
  electrical_measurements.motors[1].dcAmpsAvgAcc += ABS(adc_buffer.dcl - offsetdcl);

  if (buzzerTimer % 1000 == 500) { // to save CPU time
    electrical_measurements.motors[0].dcAmpsAvg = electrical_measurements.motors[0].dcAmpsAvgAcc*MOTOR_AMP_CONV_DC_AMP/1000;
    electrical_measurements.motors[1].dcAmpsAvg = electrical_measurements.motors[1].dcAmpsAvgAcc*MOTOR_AMP_CONV_DC_AMP/1000;
    electrical_measurements.motors[0].dcAmpsAvgAcc = 0;
    electrical_measurements.motors[1].dcAmpsAvgAcc = 0;
  }
#endif

  //disable PWM when current limit is reached (current chopping)
  if(dclAmps > electrical_measurements.dcCurLim || timeout > TIMEOUT || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
  }

  if(dcrAmps > electrical_measurements.dcCurLim || timeout > TIMEOUT || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

#ifdef DO_MEASUREMENTS
  electrical_measurements.motors[0].r1 = adc_buffer.rl1 - offsetrl1;
  electrical_measurements.motors[0].r2 = adc_buffer.rl2 - offsetrl2;
  electrical_measurements.motors[0].q  = curl;

  electrical_measurements.motors[1].r1 = adc_buffer.rr1 - offsetrr1;
  electrical_measurements.motors[1].r2 = adc_buffer.rr2 - offsetrr2;
  electrical_measurements.motors[1].q  = 0;//curl;
#endif

  //create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerTimer % buzzerFreq == 0) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
  }

  // ############################### MOTOR CONTROL ###############################
  int ul, vl, wl;
  int ur, vr, wr;



  static boolean_T OverrunFlag = false;
  /* Check for overrun */
  if (OverrunFlag) {
    return;
  }
  OverrunFlag = true;
 
    /* Set motor inputs here */
    // hall_ul etc. now read in hall interrupt.

  __disable_irq(); // but we want all values at the same time, without interferance
    rtU.b_hallALeft   = hall_ul;
    rtU.b_hallBLeft   = hall_vl;
    rtU.b_hallCLeft   = hall_wl;
    rtU.r_DCLeft      = -pwml;

    rtU.b_hallARight  = hall_ur;
    rtU.b_hallBRight  = hall_vr;
    rtU.b_hallCRight  = hall_wr;
    rtU.r_DCRight     = -pwmr;
  __enable_irq();

    /* Step the controller */
    BLDC_controller_step();

    if ((rtU.b_hallALeft != hall_ul) || (rtU.b_hallBLeft != hall_vl) || (rtU.b_hallCLeft != hall_wl)){
      local_hall_params[0].hall_change_in_bldc_count++;
    }
    if ((rtU.b_hallARight != hall_ur) || (rtU.b_hallBRight != hall_vr) || (rtU.b_hallCRight != hall_wr)){
      local_hall_params[1].hall_change_in_bldc_count++;
    }

    /* Get motor outputs here */
    ul            = rtY.DC_phaALeft;
    vl            = rtY.DC_phaBLeft;
    wl            = rtY.DC_phaCLeft;
  // motSpeedLeft = rtY.n_motLeft;
  // motAngleLeft = rtY.a_elecAngleLeft;

    ur            = rtY.DC_phaARight;
    vr            = rtY.DC_phaBRight;
    wr            = rtY.DC_phaCRight;
 // motSpeedRight = rtY.n_motRight;
 // motAngleRight = rtY.a_elecAngleRight;

  /* Indicate task complete */
  OverrunFlag = false;
  // ###############################################################################

  LEFT_TIM->LEFT_TIM_U    = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_V    = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  LEFT_TIM->LEFT_TIM_W    = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  RIGHT_TIM->RIGHT_TIM_U  = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_V  = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
  RIGHT_TIM->RIGHT_TIM_W  = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);

  __disable_irq(); // but we want both values at the same time, without interferance
#ifdef HALL_INTERRUPTS
  time = h_timer_hall.Instance->CNT;
  timerwraps_copy = timerwraps;
  long long end_us = ((timerwraps_copy<<16) + time) * 10;
#endif
  __enable_irq();

  timeStats.bldc_us = (end_us - now_us);

}
