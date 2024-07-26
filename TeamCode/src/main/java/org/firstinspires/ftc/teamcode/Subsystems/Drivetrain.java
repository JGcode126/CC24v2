package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;
    DcMotor motorfl;
    DcMotor motorbr;
    DcMotor motorbl;

    DcMotor armLift;

    CRServo duckSpinner;

    Servo claw;


    public double target;
    public double Kp;
    public double Ki;
    public double Kd;
    public double value;
    public double error;
    public double integral;
    public double dervative;

    public double inputTurn;

    public double targetAngle;
    public double releaseAngle;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");

        armLift = hardwareMap.get(DcMotor.class, "armlift");

        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");

        claw = hardwareMap.get(Servo.class, "claw");

        motorbr.setDirection(DcMotorSimple.Direction.FORWARD);
        motorfr.setDirection(DcMotorSimple.Direction.FORWARD);

        motorbl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorfl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double PIDCorrection(double Kp, double Ki, double Kd, double heading) {
        double lastError = error;
        double value = heading;
        double error = target - value;
        integral = integral + error;
        double derivative = error - lastError;
        return (error * Kp) + (integral * Ki) + (dervative * Kd);
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

    public void driveDO(double drive, double strafe, double turn, double slow, double heading, double superSlow, boolean PIDon) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;

        if (PIDon) {
            turn = PIDCorrection(-0.05, 0, 0, heading);
        }

        if (turn != 0) {
            inputTurn = turn;
            releaseAngle = heading;
        } else {
            targetAngle = releaseAngle + 0.5;
            inputTurn = PIDCorrection( 0.05, 0.0005, 0.01,targetAngle-heading);
        }




        if (slow > 0.25) {
            motorfl.setPower((drive + strafe + (inputTurn)) * -0.5);
            motorfr.setPower((drive - strafe - (inputTurn)) * -0.5);
            motorbl.setPower((drive - strafe + (inputTurn)) * -0.5);
            motorbr.setPower((drive + strafe - (inputTurn)) * -0.5);
        } else if (superSlow > 0.25) {
            motorfl.setPower((drive + strafe * 1.5 + inputTurn) * -0.25);
            motorfr.setPower((drive - strafe * 1.5 - inputTurn) * -0.25);
            motorbl.setPower((drive - strafe * 1.5 + inputTurn) * -0.25);
            motorbr.setPower((drive + strafe * 1.5 - inputTurn) * -0.25);
        } else {
            motorfl.setPower((drive + strafe + inputTurn * 0.9) * -1);
            motorfr.setPower((drive - strafe - inputTurn * 0.9) * -1);
            motorbl.setPower((drive - strafe + inputTurn * 0.9) * -1);
            motorbr.setPower((drive + strafe - inputTurn * 0.9) * -1);
        }
    }

    public void duckSpinner(double speed) {
        duckSpinner.setPower(speed);
    }

    public void liftArm(double speed) {
        armLift.setPower(speed);
    }

    public void claw(double position) {
        claw.setPosition(position);
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
