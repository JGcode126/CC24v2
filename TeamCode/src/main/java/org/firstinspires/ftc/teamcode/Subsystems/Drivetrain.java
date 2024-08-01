package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;
    DcMotor motorfl;
    DcMotor motorbr;
    DcMotor motorbl;

    double integralSum;
    double lastError;
    double PreTime;
    double error;
    double releaseAngle;
    double targetAngle;
    double inputTurn;

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






    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));
        drive = rotatedVector.y;
        strafe = rotatedVector.x;

         if (turn != 0) {
            inputTurn = turn;
            releaseAngle = (heading);
        } else {
            targetAngle = releaseAngle;
            inputTurn = PID(targetAngle-heading);
        }

        if (slow > 0.05) {
            motorfr.setPower((drive + strafe + inputTurn) * -.25);
            motorfl.setPower((drive - strafe - inputTurn) * -.25);
            motorbr.setPower((drive - strafe + inputTurn) * -.25);
            motorbl.setPower((drive + strafe - inputTurn) * -.25);

        } else {
            motorfr.setPower((drive + strafe + inputTurn) * -.78);
            motorfl.setPower((drive - strafe - inputTurn) * -.78);
            motorbr.setPower((drive - strafe + inputTurn) * -.78);
            motorbl.setPower((drive + strafe - inputTurn) * -.78);
        }
    }
    public double PID(double error){
            integralSum += error;

            double ChangeTime = (System.currentTimeMillis() - PreTime) / 1000.0;
            double ChangeError = error - lastError;
            if (ChangeError < 0){
                ChangeError += 360;
            } else if (ChangeError > 360){
                ChangeError -= 360;
            }
            double ChangeRate = ChangeError / ChangeTime;

            lastError = error;
            PreTime = System.currentTimeMillis();

            double pComponent = error * DrivetrainDash.kP;
            double iComponent = integralSum * DrivetrainDash.kI;
            double dComponent = ChangeRate * DrivetrainDash.kD;

            return pComponent +iComponent +dComponent;
    }


}




