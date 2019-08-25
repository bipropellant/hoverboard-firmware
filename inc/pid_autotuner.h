/*
Adapted from https://github.com/br3ttb/Arduino-PID-AutoTune-Library/ by Emerick Herve
 */

#ifndef PID_AutoTune_v0
#define PID_AutoTune_v0
/*-------------------------------------------------------------*/
/*		Includes and dependencies			*/
/*-------------------------------------------------------------*/
//#include "tick/tick.h"
//#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

//LIBRARY_VERSION	0.0.1

typedef struct tag_pid_autotune {
	bool isMax;
	bool isMin;
	float * input;
	float * output;
	float setpoint;
	float noiseBand;
	int controlType;
	bool running;
	unsigned long peak1;
	unsigned long peak2;
	unsigned long lastTime;
	int sampleTime;
	int nLookBack;
	int peakType;
	float lastInputs[101];
    float peaks[10];
	int peakCount;
	bool justchanged;
	bool justevaled;
	float absMax;
	float absMin;
	float oStep;
	float outputStart;
	float Ku;
	float Pu;
} pid_autotune;

typedef pid_autotune * p_pid_autotune;

//bool tuningEnabled = false;

#ifdef	__cplusplus
extern "C" {
#endif
  //commonly used functions **************************************************************************
    p_pid_autotune at_create(p_pid_autotune at, float* in, float* out);

    int Runtime(p_pid_autotune at);						// * Similar to the PID Compute function, returns non 0 when done
	void Cancel(p_pid_autotune at);						// * Stops the AutoTune	
	
	void SetOutputStep(p_pid_autotune at, float Step);	// * how far above and below the starting value will the output step?	
	float GetOutputStep(p_pid_autotune at);				// 

	void SetOutput(p_pid_autotune at, float output);
	
	void SetControlType(p_pid_autotune at, int Type); 	// * Determies if the tuning parameters returned will be PI (D=0)
	int GetControlType(p_pid_autotune at);				//   or PID.  (0=PI, 1=PID)			
	
	void SetLookbackSec(p_pid_autotune at, int value);	// * how far back are we looking to identify peaks
	int GetLookbackSec(p_pid_autotune at);				//
	
	void SetNoiseBand(p_pid_autotune at, float Band);	// * the autotune will ignore signal chatter smaller than this value
	float GetNoiseBand(p_pid_autotune at);				//   this should be acurately set
	
	float GetKp(p_pid_autotune at);						// * once autotune is complete, these functions contain the
	float GetKi(p_pid_autotune at);						//   computed tuning parameters.  
	float GetKd(p_pid_autotune at);						//
	
    void FinishUp(p_pid_autotune at);
#ifdef	__cplusplus
}
#endif

#endif
