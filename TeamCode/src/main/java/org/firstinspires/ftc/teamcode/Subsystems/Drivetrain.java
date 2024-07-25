package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.Vector2d;

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
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        //settingreverse
        motorfl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorbr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drivePower){
    motorfr.setPower(drivePower);
    motorfl.setPower(drivePower);
    motorbr.setPower(drivePower);
    motorfl.setPower(drivePower);
    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading){
        Vector2d driveVector  = new Vector2d(strafe, drive);
        Vector2d rotatedvector = driveVector.rotate(Math.toRadians(heading));
        drive = rotatedvector.y;
        strafe  = rotatedvector.x;


        if(slow > 0.05){
            motorfl.setPower((drive + strafe + turn) * -0.25);
            motorfr.setPower((drive + strafe + turn) * -0.25);
            motorbl.setPower((drive + strafe + turn) * -0.25);
            motorbr.setPower((drive + strafe + turn) * -0.25);
        }else {
            motorfl.setPower((drive + strafe + turn) * -0.75);
            motorfr.setPower((drive + strafe + turn) * -0.75);
            motorbl.setPower((drive + strafe + turn) * -0.75);
            motorbr.setPower((drive + strafe + turn) * -0.75);
        }


    }
}
