/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PWM_SERVO_PLUGIN_H
#define PWM_SERVO_PLUGIN_H

#include <Arduino.h>
#include <Servo.h>
#include <macros/macros.h>

namespace DigitalServo
{
    class Plugin
    {
        Servo servo_;
        float current_angle_;
        float min_angle_;
        float max_angle_;
        float offset_;
        int inverter_;
        public:
            Plugin(int hardware_pin, float min_angle, float max_angle, float offset, bool inverted):
            current_angle_(0),    
            min_angle_(min_angle),
            max_angle_(max_angle),
            offset_(offset),
            inverter_(1)
            {            
                if(inverted)
                {
                    inverter_ = -1;
                }

                servo_.attach(hardware_pin);
            }

            void positionControl(float angle)
            {
                current_angle_ = angle;
                servo_.writeMicroseconds(radiansToPWM((angle + offset_) * inverter_));
            }

            float getJointPosition()
            {
                return current_angle_;
            }

            int radiansToPWM(float angle)
            {
                float pwm = 0;

                if(max_angle_ == 0)
                {   
                    pwm = mapFloat(angle, min_angle_, max_angle_, 2500, 500);
                }
                else
                {
                    pwm = mapFloat(angle, min_angle_, max_angle_, 500, 2500);
                }

                return round(pwm);
            }
    };
}

#endif