package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MantisBlade {

    DcMotor bladeMotor;

    public MantisBlade(HardwareMap hardwareMap) {
        bladeMotor = hardwareMap.get(DcMotorEx.class, "bladeMotor");
        bladeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void extend(double power) {

        bladeMotor.setPower(power);
    }
    public void retract(double power) {

        bladeMotor.setPower(power);
        //bladeMotor.setTargetPosition();
        //bladeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void still() {

        bladeMotor.setPower(0);
    }

    public int data1() {
        return bladeMotor.getCurrentPosition();
    }

    public int data2() {
        return bladeMotor.getTargetPosition();
    }

//    public void failSafe() {
//        if (bladeMotor.getCurrentPosition() >= 240) {
//            bladeMotor.setPower(0);
//        } else if (bladeMotor.getCurrentPosition() >= -945) {
//            bladeMotor.setPower(0);
//        }
//    }

}
