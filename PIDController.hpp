// Copyright (c) 2023, Tinker Twins
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <queue>

using namespace std;

// PID controller class
class PIDController {
    /*
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    */
    public:
        float kP; // Proportional gain
        float kI; // Integral gain
        float kD; // Derivative gain
        float kS; // Saturation constant (error history buffer size)
        float err_int; // Error integral
        float err_dif; // Error difference
        float err_prev; // Previous error
        queue<float> err_hist; // Limited buffer of error history
        double t_prev; // Previous time

        PIDController(float KP, float KI, float KD, float KS) {
            kP       = KP; // Proportional gain
            kI       = KI; // Integral gain
            kD       = KD; // Derivative gain
            kS       = KS; // Saturation constant (error history buffer size)
            err_int  = 0; // Error integral
            err_dif  = 0; // Error difference
            err_prev = 0; // Previous error
            t_prev   = 0; // Previous time
        }

        float control(float err, double t){
            /*
            Generate PID controller output.
            :param err: Instantaneous error in control variable w.r.t. setpoint
            :param t  : Current timestamp
            :return u : PID controller output
            */
            double dt = t - t_prev; // Timestep
            if (dt > 0.0) {
                err_hist.push(err); // Update error history
                err_int += err; // Integrate error
                if (err_hist.size() > kS) { // Jacketing logic to prevent integral windup
                    err_hist.pop(); // Rolling FIFO buffer
                }
                err_dif = (err - err_prev); // Error difference
                float u = (kP * err) + (kI * err_int * dt) + (kD * err_dif / dt); // PID control law
                err_prev = err; // Update previos error term
                t_prev = t; // Update timestamp
                return u; // Control signal
            }
        }
};