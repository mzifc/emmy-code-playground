package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// created by ME Zifchak - updated 7/7/23 15:59:56

public class PIDController {
    ElapsedTime timer = new ElapsedTime();
    
    double lastError = 0;
    double integralSum = 0;
    double Kp, Ki, Kd;
    //double reference = 1;

    
    // constructing the PID controllere
    // @param Kp Proportional coefficient
    // @param Ki Integral coefficient
    // @param Kd Derivative coefficient
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        
    }
    
    // update the PID controller output
    // @param target is where we want to be OR reference
    // @param state where we currently are OR motor position
    // @return the command to our motor OR motor power
    public double update(double target, double state) {
        double error = target - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        
        return Kp * error + Ki * integralSum + Kd * derivative;
    
    }
}
