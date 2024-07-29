package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    DcMotor arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotor.class, "arm");
    }
    public void In() {
        if (gamepad1.cross) {
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setPower(0.3);
        }
    }
    public void Out() {
        if (gamepad1.triangle) {
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setPower(0.3);
        }
        else {
           arm.setPower(0);
        }
    }
}
