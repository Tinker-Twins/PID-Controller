// Copyright (c) 2026, Tinker Twins
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

using System.Collections.Generic;

// PID controller class
public class PIDController
{
    /*
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    */

    // PID controller properties
    public float kP { get; private set; } // Proportional gain
    public float kI { get; private set; } // Integral gain
    public float kD { get; private set; } // Derivative gain
    public int kS   { get; private set; } // Saturation constant (error history buffer size)

    private float errInt;           // Error integral
    private float errDif;           // Error difference
    private float errPrev;          // Previous error
    private Queue<float> errHist;   // Limited buffer of error history
    private float tPrev;            // Previous time

    public PIDController(float KP, float KI, float KD, int KS)
    {
        kP      = KP;   // Proportional gain
        kI      = KI;   // Integral gain
        kD      = KD;   // Derivative gain
        kS      = KS;   // Saturation constant (error history buffer size)
        errInt  = 0;    // Error integral
        errDif  = 0;    // Error difference
        errPrev = 0;    // Previous error
        tPrev   = 0;    // Previous time
        errHist = new Queue<float>(kS); // Limited error history buffer
    }

    public float Control(float err, float t)
    {
        /*
        Generate PID controller output.
        :param err: Instantaneous error in control variable w.r.t. setpoint
        :param t  : Current timestamp
        :return u : PID controller output
        */
        float dt = t - tPrev; // Timestep
        if (dt > 0.0f)
        {
            errHist.Enqueue(err); // Update error history
            errInt += err; // Integrate error
            if (errHist.Count > kS) // Jacketing logic to prevent integral windup
            {
                errInt -= errHist.Dequeue(); // Rolling FIFO buffer
            }
            errDif = (err - errPrev); // Error difference
            float u = (kP * err) + (kI * errInt * dt) + (kD * errDif / dt); // PID control law
            errPrev = err; // Update previous error term
            tPrev = t; // Update timestamp
            return u; // Control signal
        }
        else
        {
            Debug.Log("Non-positive timestep detected!");
            return 0.0f;
        }
    }

    public void Reset()
    {
        // Reset all PID controller variables to initial state
        errInt = 0;
        errDif = 0;
        errPrev = 0;
        tPrev = 0;
        errHist.Clear();
    }
}