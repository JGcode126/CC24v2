package org.firstinspires.ftc.teamcode.Autonomous;

public class badpid {
    private double proportional;
    //private double integral;
    //private double derivative;

    //private double integralSum;

    //private boolean isTuning = true;

    private double currentError;
    private double lastError;
    private double deltaError = currentError-lastError;

    public badpid(double proportional/*, double integral, double derivative*/){
        this.proportional = proportional;
        //this.integral = integral;
        //this.derivative = derivative;
    }


    }



