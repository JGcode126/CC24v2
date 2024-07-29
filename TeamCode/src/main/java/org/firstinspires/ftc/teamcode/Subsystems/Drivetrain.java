package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    IMU gyro;
    DcMotor bl;
    DcMotor br;
    DcMotor fl;
    DcMotor fr;
    public double releaseAngle;

    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    //Callable drive functions

    double error = 0;
    double integral;
    double derivative;
    double lastError;
    double target;
    double value;
    public double PID (double Kp, double Ki, double Kd, double heading, double target) {
        value = heading;
        lastError = error;
        error = target - value;
        if (error < -180){error+=360;}
        else if (error > 180){error-=360;}
        integral = integral + error;
        derivative = error - lastError;
        return ((error * Kp) + (integral * Ki) + (derivative * Kd));
    }

    public void drive(double drive, double turn, double strafe, double speed) {
        ;bl.setPower((drive - strafe + turn) * speed);
        br.setPower((drive + strafe - turn) * speed);
        fl.setPower((drive + strafe + turn) * speed);
        fr.setPower((drive - strafe - turn) * speed);
    }

    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;


        double inputTurn;
        if(turn != 0) {
            releaseAngle = heading;
        } else {
            target = releaseAngle + 0.5;
            turn = PID (0.03, 0.0001, 0.05, heading, target);
          }

        if (slow > 0.05) {
            fl.setPower((drive + strafe + turn) * 0.25);
            fr.setPower((drive - strafe - turn) * 0.25);
            bl.setPower((drive - strafe + turn) * 0.25);
            br.setPower((drive + strafe - turn) * 0.25);
        } else {
            bl.setPower((drive - strafe + turn) * 1);
            br.setPower((drive + strafe - turn) * 1);
            fl.setPower((drive + strafe + turn) * 1);
            fr.setPower((drive - strafe - turn) * 1);
        }
    }
}