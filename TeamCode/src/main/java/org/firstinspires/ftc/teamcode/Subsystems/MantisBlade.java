package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MantisBlade {

    DcMotor bladeMotor;

    public MantisBlade(HardwareMap hardwareMap) {
        bladeMotor = hardwareMap.get(DcMotorEx.class, "bladeMotor");

    }

    public void extend(double power) {
        bladeMotor.setPower(power);
    }

    public void retract(double power) {
        bladeMotor.setPower(power);
    }

    public void still() {
        bladeMotor.setPower(0);
    }

}
