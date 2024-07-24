package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

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
    public void driveDO(double drive, double strafe, double turn, double slow, double heading){
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;

        if(slow> 0.05){
            motorfl.setPower((drive + strafe + turn) * -0.25);
            motorfr.setPower((drive + strafe + turn) * -0.25);
            motorbr.setPower((drive + strafe + turn) * -0.25);
            motorbl.setPower((drive + strafe + turn) * -0.25);
        }else {
            motorbl.setPower((drive + strafe + turn) * -0.75);
            motorbr.setPower((drive + strafe + turn) * -0.75);
            motorfl.setPower((drive + strafe + turn) * -0.75);
            motorfr.setPower((drive + strafe + turn) * -0.75);
        }
    }


}
