package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;
    DcMotor motorfl;
    DcMotor motorbr;
    DcMotor motorbl;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        motorfr = hardwareMap.get(DcMotor.class, "fr");
        motorfl = hardwareMap.get(DcMotor.class, "fl");
        motorbr = hardwareMap.get(DcMotor.class, "br");
        motorbl = hardwareMap.get(DcMotor.class, "bl");
        //fr br reverse
        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbr.setDirection(DcMotor.Direction.REVERSE);
    }




    //Callable drive functions
    public void drive(double drive, double strafe, double turn, double slow){

        if (slow > 0.05){
            motorfl.setPower((drive + strafe + turn) * -0.25);
            motorfr.setPower((drive - strafe - turn) * -0.25);
            motorbl.setPower((drive - strafe + turn) * -0.25);
            motorbr.setPower((drive + strafe - turn) * -0.25);
        } else {
            motorfl.setPower((drive + strafe + turn) * -0.75);
            motorfr.setPower((drive - strafe - turn) * -0.75);
            motorbl.setPower((drive - strafe + turn) * -0.75);
            motorbr.setPower((drive + strafe - turn) * -0.75);
        }
    }

    public void driveDO(double drive, double strafe, double turn, double slow, double heading){
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;



        if (slow > 0.05){
            motorfl.setPower((drive + strafe + turn) * -0.25);
            motorfr.setPower((drive - strafe - turn) * -0.25);
            motorbl.setPower((drive - strafe + turn) * -0.25);
            motorbr.setPower((drive + strafe - turn) * -0.25);
        } else {
            motorfl.setPower((drive + strafe + turn) * -0.75);
            motorfr.setPower((drive - strafe - turn) * -0.75);
            motorbl.setPower((drive - strafe + turn) * -0.75);
            motorbr.setPower((drive + strafe - turn) * -0.75);
        }

        }
    }


