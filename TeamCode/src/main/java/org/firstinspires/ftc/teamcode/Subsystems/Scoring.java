package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLOpen;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoRClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoROpen;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    Servo servoR;
    Servo servoL;
    Servo servoArm;



    public Scoring(HardwareMap hardwareMap){
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
    }
    public void open(){
        servoR.setPosition(servoROpen);
        //opens forward
        servoL.setPosition(servoLOpen);
        // opens bachwards
    }
    public void closed(){
        servoR.setPosition(servoRClosed);
        servoL.setPosition(servoLClosed);
    }
    public void armUp(){
        servoArm.setPosition(1);
    }
    public void armDown(){
        servoArm.setPosition(0);
    }
}

