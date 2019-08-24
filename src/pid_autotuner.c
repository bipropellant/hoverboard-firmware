/*
Adapted from https://github.com/br3ttb/Arduino-PID-AutoTune-Library/ by Emerick Herve
 */

#include "pid_autotuner.h"

p_pid_autotune at_create(p_pid_autotune at, float* in, float* out)
{
	at->input = in;
	at->output = out;
	at->controlType = 1 ; //default to PID
	at->noiseBand = 5;
	at->running = false;
	at->oStep = 30;
	SetLookbackSec(at, 10);
	at->lastTime = HAL_GetTick();

	return at;
}

void Cancel(p_pid_autotune at)
{
	at->running = false;
} 
 
int Runtime(p_pid_autotune at)
{
	at->justevaled=false;
	if(at->peakCount>9 && at->running)
	{
		at->running = false;
		FinishUp(at);
		return 1;
	}
	unsigned int now = HAL_GetTick();
	
	if((now-at->lastTime)<at->sampleTime) return false;
	at->lastTime = now;
	float refVal = *(at->input);
	at->justevaled=true;
	if(!at->running)
	{ //initialize working variables the first time around
		char tmp[50];
      	sprintf(tmp, "Starting calibration at %.2f\r\n", (*at->output));
        consoleLog(tmp);

		at->peakType = 0;
		at->peakCount=0;
		at->justchanged=false;
		at->absMax=refVal;
		at->absMin=refVal;
		at->setpoint = refVal;
		at->running = true;
		at->outputStart = (*at->output);
		(*at->output) = at->outputStart+at->oStep;
	}
	else
	{
		if(refVal>at->absMax)at->absMax=refVal;
		if(refVal<at->absMin)at->absMin=refVal;
	}
	
	//oscillate the output base on the input's relation to the setpoint
	
	if(refVal>at->setpoint+at->noiseBand) (*at->output) = at->outputStart-at->oStep;
	else if (refVal<at->setpoint-at->noiseBand) (*at->output) = at->outputStart+at->oStep;
	
	
  //bool isMax=true, isMin=true;
  at->isMax=true;at->isMin=true;
  //id peaks
  for(int i=at->nLookBack-1;i>=0;i--)
  {
    float val = at->lastInputs[i];
    if(at->isMax) at->isMax = refVal>val;
    if(at->isMin) at->isMin = refVal<val;
    at->lastInputs[i+1] = at->lastInputs[i];
  }
  at->lastInputs[0] = refVal;  
  if(at->nLookBack<9)
  {  //we don't want to trust the maxes or mins until the inputs array has been filled
	return 0;
	}
  
  if(at->isMax)
  {
    if(at->peakType==0)at->peakType=1;
    if(at->peakType==-1)
    {
      at->peakType = 1;
      at->justchanged=true;
      at->peak2 = at->peak1;
    }
    at->peak1 = now;
    at->peaks[at->peakCount] = refVal;
   
  }
  else if(at->isMin)
  {
    if(at->peakType==0)at->peakType=-1;
    if(at->peakType==1)
    {
      at->peakType=-1;
      at->peakCount++;
      at->justchanged=true;
    }
    
    if(at->peakCount<10)at->peaks[at->peakCount] = refVal;
  }
  
  if(at->justchanged && at->peakCount>2)
  { //we've transitioned.  check if we can autotune based on the last peaks
    float avgSeparation = (abs(at->peaks[at->peakCount-1]-at->peaks[at->peakCount-2])+abs(at->peaks[at->peakCount-2]-at->peaks[at->peakCount-3]))/2;
    if( avgSeparation < 0.05*(at->absMax-at->absMin))
    {
		FinishUp(at);
      at->running = false;
	  return 1;
	 
    }
  }
   at->justchanged=false;
	return 0;
}
void FinishUp(p_pid_autotune at)
{
	  (*at->output) = at->outputStart;
      //we can generate tuning parameters!
      at->Ku = 4*(2*at->oStep)/((at->absMax-at->absMin)*3.14159);
      at->Pu = (float)(at->peak1-at->peak2) / 1000;
}

float GetKp(p_pid_autotune at)
{
	return at->controlType==1 ? 0.6 * at->Ku : 0.4 * at->Ku;
}

float GetKi(p_pid_autotune at)
{
	return at->controlType==1? 1.2*at->Ku / at->Pu : 0.48 * at->Ku / at->Pu;  // Ki = Kc/Ti
}

float GetKd(p_pid_autotune at)
{
	return at->controlType==1? 0.075 * at->Ku * at->Pu : 0;  //Kd = Kc * Td
}

void SetOutputStep(p_pid_autotune at, float Step)
{
	at->oStep = Step;
}

void SetOutput(p_pid_autotune at, float output)
{
	(*at->output) = output;
}

float GetOutputStep(p_pid_autotune at)
{
	return at->oStep;
}

void SetControlType(p_pid_autotune at, int Type) //0=PI, 1=PID
{
	at->controlType = Type;
}
int GetControlType(p_pid_autotune at)
{
	return at->controlType;
}
	
void SetNoiseBand(p_pid_autotune at, float Band)
{
	at->noiseBand = Band;
}

float GetNoiseBand(p_pid_autotune at)
{
	return at->noiseBand;
}

void SetLookbackSec(p_pid_autotune at, int value)
{
    if (value<1) value = 1;
	
	if(value<25)
	{
		at->nLookBack = value * 4;
		at->sampleTime = 250;
	}
	else
	{
		at->nLookBack = 100;
		at->sampleTime = value*10;
	}
}

int GetLookbackSec(p_pid_autotune at)
{
	return at->nLookBack * at->sampleTime / 1000;
}