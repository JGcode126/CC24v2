package org.firstinspires.ftc.teamcode.Subsystems;

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
        servoR.setPosition(1);
        servoL.setPosition(0);
    }
    public void closed(){
        servoR.setPosition(.85);
        servoL.setPosition(.15);
    }
    public void armUp(){
        servoArm.setPosition(0);
    }
    public void armDown(){
        servoArm.setPosition(1);
    }
}

