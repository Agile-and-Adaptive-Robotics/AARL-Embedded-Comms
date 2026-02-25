#ifndef MY_MUSCLE_H
#define MY_MUSCLE_H

#include <Arduino.h>

class Muscle {
  private:
    uint8_t valve_pin;
    
  public:
    //initialize the class object      
    Muscle(String Name, uint8_t valve_pin, int pulse_freq = 100, bool enable_pulsing = false);

    //valve control functions
    void open();
    void close();

    //pulse control functions
    bool should_pulse_end();    
    bool should_pulse_start();
    void this_pulse_start();
    void this_pulse_end();
    void pulse_nanny();

    // pulse setters
    void set_pulse_enable(bool);
    void set_pulse_frequency(float);

    //=============================MUSCLE ID PARAMETERS==================================
    String Name;
    
    //==============================PULSE CONTROL PARAMETERS=============================

    // TOGGLES
    bool IS_PULSING_ENABLED;                // tracks whether auto pulsing is enabled. 
    bool IS_A_PULSE_ACTIVE;                 //(during pulsing) tracks whether the valve is open  

    // TIME TRACKING VARIABLES
    unsigned long WHEN_THIS_PULSE_STARTED;  //when the valve last opened 
    unsigned long WHEN_LAST_PULSE_ENDED;    //when the valve last closed 

    // 
    /* ================== PULSE CONTROL PARAMETERS ===========================================
     * DT_ON and DT_OFF are elements of the DUTY_CYCLE. The relationship to PULSE_FREQUENCY is 
     * 
     *          DUTY_CYCLE = DT_ON + DT_OFF = 1/PULSE_FREQUENCY
     *          
     * DT_ON is a fixed value and DT_OFF is derived from the user-set PULSE_FREQUENCY when the 
     * SetFrequency() function is called. DUTY_CYCLE is redundant and unused in the code. 
     */
    float PULSE_FREQUENCY;
    float DT_ON; 
    float DT_OFF;         
  
};

#endif
