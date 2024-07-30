package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.DeadZone;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kds;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kf;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kfs;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kp;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kps;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    double integral;
    double lastErrorHeading;
    double inputTurn;
    double releaseAngle;
    double targetAngle;
    double lastErrorDrive;
    double lastErrorStrafe;

    SparkFunOTOS myOtos;

    double inputDrive;
    double targetY;

    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        fr = hardwareMap.get(DcMotor.class, "drivefr");
        fl = hardwareMap.get(DcMotor.class, "drivefl");
        br = hardwareMap.get(DcMotor.class, "drivebr");
        bl = hardwareMap.get(DcMotor.class, "drivebl");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        if (turn != 0) {
            inputTurn = turn;
            releaseAngle = heading;
        } else {
            targetAngle = releaseAngle + 0.5;
            inputTurn = pidHeading(targetAngle, 0.02, 0, 0, heading);
        }



        drive = rotatedVector.y;
        strafe = rotatedVector.x;
        if (slow > 0.05) {
            fr.setPower((drive - strafe - inputTurn) * 0.1);
            fl.setPower((drive + strafe + inputTurn) * 0.1);
            br.setPower((drive + strafe - inputTurn) * 0.1);
            bl.setPower((drive - strafe + inputTurn) * 0.1);
        }
        else {
            fr.setPower((drive - strafe - inputTurn) * 1);
            fl.setPower((drive + strafe + inputTurn) * 1);
            br.setPower((drive + strafe - inputTurn) * 1);
            bl.setPower((drive - strafe + inputTurn) * 1);
        }
    }
    public double pidHeading(double target, double kp, double ki, double kd, double error) {
        integral += error;
        double derivative = error - lastErrorHeading;
        error = target - error;
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
        return correction;
    }

    public double pfdDrive(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorDrive;
        double correction = (error * kp) + (derivative * kd);
        multTelemetry.addData("driveError", error);
        if (abs(error) > DeadZone) {
            correction += signum(error) * kf;
        }
        return correction;
    }
    public double pfdStrafe(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorStrafe;
        double correction = (error * kp) + (derivative * kd);
        multTelemetry.addData("strafeError", error);
        if (abs(error) > DeadZone) {
            correction += signum(error) * kf;
        }
        return correction;
    }

    public void goToPos(double targetX, double targetY, double currentX, double currentY) {
        multTelemetry.addData("TargetX", targetX);
        multTelemetry.addData("TargetY", targetY);
        multTelemetry.addData("CurrentX", currentX);
        multTelemetry.addData("CurrentY", currentY);

        double driveCorrection = pfdDrive(Kp, 0, Kf, targetY - currentY * 1.2);
        double strafeCorrection = pfdStrafe(Kps, 0, Kfs, targetX - currentX * 1.2);
        fr.setPower((driveCorrection - strafeCorrection - 0) * 1);
        fl.setPower((driveCorrection + strafeCorrection + 0) * 1);
        br.setPower((driveCorrection + strafeCorrection - 0) * 1);
        bl.setPower((driveCorrection - strafeCorrection + 0) * 1);
    }
}
