
#ifndef _ISR_h
#define _ISR_h



extern bool debug_mode;
extern int encoder_counter;
extern int debounce_time;    // mS to debounce
extern unsigned long last_encoder_time;
extern unsigned long last_limit_time;
extern int current_location;


// Prototypes
void encoder_Interrupt(void);
void encoder_debounce(void);
void limit_debounce(void);
void limit_Interrupt(void);

#endif

