// 
// 
// 

#include "Salus_Common.h"

// TODO:  Make this non-blocking
void shortBeep(){
#ifdef BEEP
        digitalWriteFast(BUZZER_PIN, HIGH);
        delay(150);
        digitalWriteFast(BUZZER_PIN, LOW);
#endif
}


// TODO:  Make this non-blocking
void longBeep(int numBeeps){
#ifdef BEEP
        digitalWriteFast(BUZZER_PIN, HIGH);
        delay(350);
        digitalWriteFast(BUZZER_PIN, LOW);
#endif
}

