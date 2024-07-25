package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Scoring {

    Servo rightClaw;
    Servo leftClaw;
    Servo arm;
    ElapsedTime timer=new ElapsedTime();;

    public Scoring(HardwareMap hardwareMap) {
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        arm = hardwareMap.get(Servo.class, "arm");
    }

    public void ringGrab() {
        rightClaw.setPosition(0.17);
        leftClaw.setPosition(0.8);
    }

    public void pixelGrab() {
        rightClaw.setPosition(0.1);
        leftClaw.setPosition(0.9);
    }

    public void armDown() {
        arm.setPosition(.3);
    }
    public void transferUp() {
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        while (timer.seconds() <= .15) {}
        arm.setPosition(0);

    }
}
