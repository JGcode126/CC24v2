package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Scoring {
    Servo clawR;
    Servo clawL;
    Servo arm;
    CRServo spinner;

    public enum DuckSpin {
        BLUE, RED, OFF
    }


    public Scoring(HardwareMap hardwareMap) {
        clawR = hardwareMap.get(Servo.class, "rightClaw");
        clawL = hardwareMap.get(Servo.class, "leftClaw");
        arm = hardwareMap.get(Servo.class, "arm");
        spinner = hardwareMap.get(CRServo.class, "spinner");
    }

    public void ringGrab() {
        clawR.setPosition(0.16);
        clawL.setPosition(0.8);
    }
    public void intake() {
        arm.setPosition(0);
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
//        superSlow = true;
    }
    public void pixelGrab() {
        clawR.setPosition(0.1);
        clawL.setPosition(0.9);
    }
    public void down() {
        arm.setPosition(0.395);
//        superSlow = false;
    }
    public void score() {
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
    }
    public void spin(boolean team) {
        if (team) {
            spinner.setPower(1);
        } else if (!team) {
            spinner.setPower(-1);
        }

    }

    public void stopSpin() {
        spinner.setPower(0);
    }
}
