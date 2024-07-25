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

        motorbr.setDirection(DcMotorSimple.Direction.REVERSE);
        motorfr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drive){
        motorfr.setPower(drive);
        motorfl.setPower(drive);
        motorbr.setPower(drive);
        motorbl.setPower(drive);
    }

    public void strafe(double drive) {
        motorfl.setPower(drive);
        motorbl.setPower(-drive);
        motorfr.setPower(-drive);
        motorbr.setPower(drive);
    }


    public void turn(double drive) {
        motorfl.setPower(drive);
        motorbl.setPower(drive);
        motorfr.setPower(-drive);
        motorbr.setPower(-drive);
    }

    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;

        if (slow > 0.05) {
            motorfl.setPower((drive + strafe + turn) * -0.5);
            motorfr.setPower((drive - strafe - turn) * -0.5);
            motorbl.setPower((drive - strafe + turn) * -0.5);
            motorbr.setPower((drive + strafe - turn) * -0.5);
        } else {
            motorfl.setPower((drive + strafe + turn) * -1);
            motorfr.setPower((drive - strafe - turn) * -1);
            motorbl.setPower((drive - strafe + turn) * -1);
            motorbr.setPower((drive + strafe - turn) * -1);
        }
    }

//    public void drivenonDO(double drive, double strafe, double turn, double slow){
//        if (slow > 0.05) {
//            motorfl.setPower((drive + strafe + turn) * -0.25);
//            motorfr.setPower((drive - strafe - turn) * -0.25);
//            motorbl.setPower((drive - strafe + turn) * -0.25);
//            motorbr.setPower((drive + strafe - turn) * -0.25);
//        } else {
//            motorfl.setPower((drive + strafe + turn) * -0.75);
//            motorfr.setPower((drive - strafe - turn) * -0.75);
//            motorbl.setPower((drive - strafe + turn) * -0.75);
//            motorbr.setPower((drive + strafe - turn) * -0.75);
//        }
//    }

}
