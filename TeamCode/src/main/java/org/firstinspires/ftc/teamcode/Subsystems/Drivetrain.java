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
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");

        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double driveY, double driveX, double heading){
    motorfr.setPower(driveY - driveX - heading);
    motorfl.setPower(driveY + driveX + heading);
    motorbr.setPower(driveY + driveX - heading);
    motorbl.setPower(driveY - driveX + heading);
    }




    public void driveDO(double drive, double strafe, double turn, double slow, double heading){
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));
        drive = rotatedVector.y;
        strafe = rotatedVector.x;

        if(slow > 0.05){
            motorfr.setPower((drive + strafe + turn) * -.25);
            motorfl.setPower((drive - strafe - turn) * -.25);
            motorbr.setPower((drive - strafe + turn) * -.25);
            motorbl.setPower((drive + strafe - turn) * -.25);

        }else{
            motorfr.setPower((drive + strafe + turn) * -.75);
            motorfl.setPower((drive - strafe - turn) * -.75);
            motorbr.setPower((drive - strafe + turn) * -.75);
            motorbl.setPower((drive + strafe - turn) * -.75);
        }

    }
    }

