#include "Muscle.h"



Muscle::Muscle(String Name, uint8_t valve_pin, int pulse_freq, bool enable_pulsing){
  this->Name = Name;
  this->valve_pin = valve_pin;
  this->PULSE_FREQUENCY = pulse_freq;
  this->IS_PULSING_ENABLED = enable_pulsing;

  //set the pinMode 
  pinMode(valve_pin, OUTPUT); 
  
  // --- pulse parameters
  DT_ON = 30;
  IS_A_PULSE_ACTIVE = false;
  
  WHEN_THIS_PULSE_STARTED = millis();
  WHEN_LAST_PULSE_ENDED = millis();

  set_pulse_frequency(PULSE_FREQUENCY);
  
}

// ---- Setters -----
void Muscle::set_pulse_frequency(float freq){
  //assign incoming value
  PULSE_FREQUENCY = freq;

  //calculate DT_Off from new frequency
  DT_OFF = 1000/PULSE_FREQUENCY - DT_ON;

  return;
}

void Muscle::set_pulse_enable(bool enable){
  // assign incoming value
  IS_PULSING_ENABLED = enable;

  return;
}

bool Muscle::should_pulse_end(){
  //yes if dt_target<dt_actual
  if((millis()-WHEN_THIS_PULSE_STARTED)>DT_ON){
    if(IS_A_PULSE_ACTIVE){
      this_pulse_end();
      return true;
    }
    return false;
  }else{
    return false;
  }
}

bool Muscle::should_pulse_start(){
  //yes, if dt_since_last_pulse_ended >= dt_off
  if((millis()-WHEN_LAST_PULSE_ENDED) >= DT_OFF){
    if(!IS_A_PULSE_ACTIVE){
      this_pulse_start();
      return true;
    }
    return false;
  }else{
    return false;
  }
}

void Muscle::pulse_nanny(){
  should_pulse_start();
  should_pulse_end();
  return;
}

// <---- Muscle commands ----->
void Muscle::this_pulse_start(){
  //open the valve
  open();

  //record the time
  WHEN_THIS_PULSE_STARTED = millis();
  
  //flip the pulse control boolean
  IS_A_PULSE_ACTIVE = true;
  
  return;
}

void Muscle::this_pulse_end(){
  //flip pulse control boolean
  IS_A_PULSE_ACTIVE = false;
  
  //close the muscle's valve
  close();
  
  //record the time
  WHEN_LAST_PULSE_ENDED = millis();
  
  return; 
}

// <---- Valve toggles ----->
void Muscle::open(){
  digitalWrite(valve_pin, HIGH);
  return;
}

void Muscle::close(){
  digitalWrite(valve_pin, LOW);
  return;
}
