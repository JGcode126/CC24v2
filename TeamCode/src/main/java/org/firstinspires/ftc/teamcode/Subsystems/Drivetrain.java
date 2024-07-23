package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;
    DcMotor motorfl;
    DcMotor motorbr;
    DcMotor motorbl;

    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
    motorfr = hardwareMap.get(DcMotor.class, "motorfr");
    motorfl = hardwareMap.get(DcMotor.class, "motorfl");
    motorbr = hardwareMap.get(DcMotor.class, "motorbr");
    motorbl = hardwareMap.get(DcMotor.class, "motorbl");

    motorfl.setDirection(DcMotor.Direction.FORWARD);
    motorbl.setDirection(DcMotor.Direction.FORWARD);

    motorfr.setDirection(DcMotor.Direction.REVERSE);
    motorbr.setDirection(DcMotor.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drive){
        motorfr.setPower(-drive);
        motorfl.setPower(-drive);
        motorbr.setPower(-drive);
        motorbl.setPower(-drive);
    }

    public void strafe(double strafe) {
        /*
        motorbr.setDirection(DcMotor.Direction.FORWARD);
        motorbl.setDirection(DcMotor.Direction.FORWARD);
        motorfl.setDirection(DcMotor.Direction.FORWARD);
        motorfr.setDirection(DcMotor.Direction.FORWARD);

         */

        motorfr.setPower(-strafe);
        motorfl.setPower(strafe);
        motorbl.setPower(-strafe);
        motorbr.setPower(strafe);
    }
    public void turn(double turn){

        motorfr.setPower(turn);
        motorfl.setPower(-turn);
        motorbl.setPower(-turn);
        motorbr.setPower(turn);
    }
}
