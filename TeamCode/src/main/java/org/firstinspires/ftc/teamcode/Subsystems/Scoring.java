package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    Servo claw;

    public Scoring(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void open() {
        claw.setPosition(0);
    }
    public void pixel() {
        claw.setPosition(0.5);
    }
    public void ring() {
        claw.setPosition(0.35);
    }



}

