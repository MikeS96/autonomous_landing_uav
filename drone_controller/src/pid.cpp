/**
 *  @file pid.cpp
 *  @author Miguel Saavedra (miguel.saaruiz@gmail@gmail.com)
 *  @brief PID controller implementation
 *  @version 0.1
 *  @date 05-01-2020
 * 
 *  Copyright (c) 2020 Miguel Saavedra
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include <iostream>
#include <cmath>
#include "drone_controller/pid.h"

using namespace std;

// Constructor
PID::PID( double cmax, double cmin, double ckp, double ckd, double cki ) :
    max(cmax), min(cmin), kp(ckp), kd(ckd), ki(cki), pre_error(0), integral(0), pre_integral(0) {}

// Destructor
PID::~PID() {}

//Calculation of PID values Setpoint is image_size/2
double PID::calculate( double setpoint, double pv, double cdt )
{
    
    // Calculate error
    double error = setpoint - pv;
    // Proportional term
    double Pout = kp * error;
    // Integral term
    integral = pre_integral + 0.5 * (pre_error + error) * 0.1; // cdt is equals to 0.1 to avoid overflow
    double Iout = ki * integral; // Integral output
    // Derivative term
    double derivative = (error - pre_error) / 0.1; // cdt is equals to 0.1 to avoid overflow
    double Dout = kp * kd * derivative; // Derivative output
    // Calculate total output
    double output = Pout + Iout + Dout;

    // Save error to previous error and previous integral value
    pre_error = error;
    pre_integral = integral;

    // Limit the max and min output
    if( output > max )
    {
        output = max; 
    }

    else if( output < min )
    {
        output = min;
    }

    return output;
}


