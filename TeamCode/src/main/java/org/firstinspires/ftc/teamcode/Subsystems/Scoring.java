package org.firstinspires.ftc.teamcode.Subsystems;




import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    Servo servoR;
    Servo servoL;
    Servo servoArm;
    // Servo servoWobbleGoalClaw;
    public Scoring(HardwareMap hardwareMap){
    servoR = hardwareMap.get(Servo.class, "servoR");
    servoL = hardwareMap.get(Servo.class, "servoL");
        servoArm = hardwareMap.get(Servo.class, "servoArm");}
    //servoWobbleGoalClaw = hardwareMap.get(Servo.class, "servoWobbleGoalClaw);}
// Potentially add new servo for servoWobbleGoalClaw, if added delete curly bracket from servoArm

    public void clawOpen(){
        servoR.setPosition(0.5);
        servoL.setPosition(0.5);
    }
    public void clawClose(){
        servoR.setPosition(0.63);
        servoL.setPosition(0.35);
    }
    public void armDown(){
        servoArm.setPosition(0);
    }
    public void armUp(){
        servoArm.setPosition(1);
    }
    //public void
}



