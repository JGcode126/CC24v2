package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;

public class DuckSpinner {
    CRServo duckSpinner;
    public DuckSpinner (org.firstinspires.ftc.teamcode.yHardware.Hardware hardware) {
        duckSpinner = hardwareMap.get(CRServo.class, "duckSpin");
    }
    public void ductSpinner(double speed) {
        if (gamepad1.square) {
            duckSpinner.setPower(speed);
            }
        else {
            duckSpinner.setPower(0);
        }
    }
}
