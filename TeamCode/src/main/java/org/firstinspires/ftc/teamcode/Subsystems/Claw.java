package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo claw;
    public void Servo(HardwareMap hardwareMap) {
        claw =  hardwareMap.get(Servo.class, "claw");
    }
    public void open() {
        if (gamepad1.dpad_left) {
            claw.setPosition(0);
        }
    }
    public void ring() {
        if (gamepad1.dpad_right) {
            claw.setPosition(0.5);
        }
    }
    public void pixel() {
        if (gamepad1.dpad_right) {
            claw.setPosition(0.35);
        }
    }
}
