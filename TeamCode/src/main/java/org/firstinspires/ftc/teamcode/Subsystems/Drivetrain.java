package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveState.DRIVE;
import static org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.DriveState.HOLD;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.DeadZone;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kdd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kdh;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kds;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kfd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kfs;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kih;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kpd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kph;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kps;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    boolean reset = true;

    double holdX;
    double holdY;
    double holdH;
    DriveState driveState;

    int holdNum;

    ElapsedTime timer;

    SparkFunOTOS myOtos;

    double inputDrive;
    double targetY;

    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        driveState = DRIVE;
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

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setLinearUnit(DistanceUnit.INCH);
        myOtos.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 5, 0);
        myOtos.setOffset(offset);
        myOtos.setLinearScalar(1.01951);
        myOtos.setAngularScalar(1.0);
        myOtos.calibrateImu();
        myOtos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);
        myOtos.getPosition().y = 0;
        myOtos.getPosition().x = 0;

        timer = new ElapsedTime();

    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(-Math.toRadians(heading));

        if (turn != 0) {
            inputTurn = turn;
            releaseAngle = heading;
        } else {
            targetAngle = releaseAngle;// + 0.5;
            inputTurn = pidHeading(targetAngle, -0.02, 0, 0, heading);
        }
        multTelemetry.addData("gyro", heading);

        if (abs(drive) < .0001 && abs(strafe) < .0001 && abs(turn) < .0001) {
            holdNum = 0;
            if (reset) {
                reset = false;
                timer.reset();
            }
            if (timer.seconds() > .5) {
                setDriveState(HOLD);
            }

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
    public double pidHeading(double target, double kp, double ki, double kd, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - lastErrorHeading;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
//        multTelemetry.addData("target", target);
//        multTelemetry.addData("current", current);
//        multTelemetry.addData("error", error);
        return correction;
    }

    public double pfdDrive(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorDrive;
        double correction = (error * kp) + (derivative * kd);
        //multTelemetry.addData("driveError", error);
        if (abs(error) > DeadZone) {
            correction += signum(error) * kf;
        }
        return correction;
    }
    public double pfdStrafe(double kp, double kd, double kf, double error) {
        double derivative = error - lastErrorStrafe;
        double correction = (error * kp) + (derivative * kd);
        //multTelemetry.addData("strafeError", error);
        if (abs(error) > DeadZone) {
            correction += signum(error) * kf;
        }
        return correction;
    }

    public void goToPos(double targetX, double targetY, double targetHeading, double currentX, double currentY, double currentHeading) {


        Vector2d driveVector = new Vector2d(targetX - currentX, targetY - currentY);
        Vector2d rotatedVector = driveVector.rotate(-Math.toRadians(currentHeading));

        inputTurn = pidHeading(targetHeading, Kph, Kih, Kdh, currentHeading);

        double driveCorrection = pfdDrive(Kpd, Kdd, Kfd, rotatedVector.y);
        double strafeCorrection = pfdStrafe(Kps, Kds, Kfs, rotatedVector.x);
        fr.setPower((driveCorrection - strafeCorrection - inputTurn) * 1);
        fl.setPower((driveCorrection + strafeCorrection + inputTurn) * 1);
        br.setPower((driveCorrection + strafeCorrection - inputTurn) * 1);
        bl.setPower((driveCorrection - strafeCorrection + inputTurn) * 1);
    }
    public void hold(double xCoordinate, double yCoordinate, double heading, double x, double y, double h) {
        if (myOtos.getPosition().x != xCoordinate || myOtos.getPosition().y != yCoordinate || myOtos.getPosition().h != heading) {
            if (holdNum < 1) {
                holdX = myOtos.getPosition().x;
                holdY = myOtos.getPosition().y;
                holdH = myOtos.getPosition().h;
            }
            goToPos(xCoordinate, yCoordinate, heading, myOtos.getPosition().x, myOtos.getPosition().y, myOtos.getPosition().h);
            holdNum ++;
        }
        if (abs(x) > .0001 || abs(y) > .0001 || abs(h) > .0001) {
            setDriveState(DRIVE);
        }
    }

    public enum DriveState  {
        DRIVE, HOLD
    }
    public void driving (double x, double y, double h, double slow) {
        switch (driveState) {
            case DRIVE:
                driveDO(-y, -x, h, slow, myOtos.getPosition().h);
                break;
            case HOLD:
                hold(holdX, holdY, holdH, x, y ,h);
                break;
        }


    }
    public void setDriveState(DriveState state) {
        driveState = state;
        reset = true;
    }





}
