package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;
    DcMotor motorfl;
    DcMotor motorbr;
    DcMotor motorbl;
    double Ki;
    double Kp;
    double Kd;

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
        Kp = -0.03;
        Ki = 0.0;
        Kd = 0.0;
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

        /* if (turn != 0) {
            inputTurn = turn;
            releaseAngle = (heading);
        } else {
            targetAngle = releaseAngle + 0.6;
            inputTurn = PID(targetAngle-heading);
        } */
        inputTurn = turn;
        if (slow > 0.05) {
            motorfr.setPower((drive + strafe + inputTurn) * -.25);
            motorfl.setPower((drive - strafe - inputTurn) * -.25);
            motorbr.setPower((drive - strafe + inputTurn) * -.25);
            motorbl.setPower((drive + strafe - inputTurn) * -.25);

        } else {
            motorfr.setPower((drive + strafe + inputTurn) * -.75);
            motorfl.setPower((drive - strafe - inputTurn) * -.75);
            motorbr.setPower((drive - strafe + inputTurn) * -.75);
            motorbl.setPower((drive + strafe - inputTurn) * -.75);
        }
    }
    public double PID(double error){
            integralSum += error;

            double ChangeTime = (System.currentTimeMillis() - PreTime) / 1000.0;
            double ChangeError = error - lastError;
            double ChangeRate = ChangeError / ChangeTime;

            lastError = error;
            PreTime = System.currentTimeMillis();

            double pComponent = error * Kp;
            double iComponent = integralSum * Ki;
            double dComponent = ChangeRate * Kd;

            return pComponent +iComponent +dComponent;
    }


}




