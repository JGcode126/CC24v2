package org.firstinspires.ftc.teamcode.Subsystems;




import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    Servo servoR;
    Servo servoL;
    Servo servoArm;
    public Scoring(HardwareMap hardwareMap){
    servoR = hardwareMap.get(Servo.class, "servoR");
    servoL = hardwareMap.get(Servo.class, "servoL");
        servoArm = hardwareMap.get(Servo.class, "servoArm");}

    public void clawOpen(){
        servoR.setPosition(1);
        servoL.setPosition(0);
    }
    public void clawClose(){
        servoR.setPosition(0.8);
    }
    public void armDown(){
        servoArm.setPosition(0);
    }
    public void armUp(){
        servoArm.setPosition(1);
    }

}



