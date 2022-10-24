#ifndef _LegsStepper_H_
#define _LegsStepper_H_

#include "Arduino.h" 


class LegsStepper {
    public:
        LegsStepper();
        void initialize(int pinNumber, int topIsInPosition);
        void moveTo(int position);
        void moveToTop();
        void moveToBottom();
};

#endif
