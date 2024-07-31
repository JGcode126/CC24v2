package org.firstinspires.ftc.teamcode.Utilities.DashConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ScoringDash {
    public static double servoROpen = .5;
    public static double servoLOpen = .5;
    public static double servoRClosed = .63;
    public static double  servoLClosed = .35;
    public static double blueThreshold = 500;
    public static double redThreshold = 500;
    public static double distThreshold = 40;

    public double getServoROpen(){
        return servoROpen;
    }
    public double getServoLOpen(){return servoLOpen;}
    public double getServoRClosed(){return servoRClosed;}
    public double getServoLClosed(){return servoLClosed;}
    public double getBlueThreshold(){return blueThreshold;}
    public double getRedThreshold(){return redThreshold;}
    public double getDistThreshold(){return distThreshold;}

}

