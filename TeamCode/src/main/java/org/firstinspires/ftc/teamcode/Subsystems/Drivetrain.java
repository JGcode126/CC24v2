package org.firstinspires.ftc.teamcode.Subsystems;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Ki;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kp;
import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.VisionDash.target;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    double integral;
    double lastError;
    double inputTurn;
    double releaseAngle;
    double targetAngle;

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
    }

    //Callable drive functions

    public void drive(double drive, double strafe, double turn, double slow){
        if (slow > 0.05) {
            fr.setPower((drive - strafe - turn) * 0.25);
            fl.setPower((drive + strafe + turn) * 0.25);
            br.setPower((drive + strafe - turn) * 0.25);
            bl.setPower((drive - strafe + turn) * 0.25);
        }
        else {
            fr.setPower((drive - strafe - turn) * 0.75);
            fl.setPower((drive + strafe + turn) * 0.75);
            br.setPower((drive + strafe - turn) * 0.75);
            bl.setPower((drive - strafe + turn) * 0.75);
        }

    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        if (turn != 0) {
            inputTurn = turn;
            releaseAngle = Math.toDegrees(heading);
        } else {
            releaseAngle = 0;
            targetAngle = releaseAngle + 0.5;
            inputTurn = pid(targetAngle, 0.03, 0.00001, 0.001, heading);
        }

        drive = rotatedVector.y;
        strafe = rotatedVector.x;
        // turn += pid(0, 0.03, 0.00001, 0.001, heading);
        if (slow > 0.05) {
            fr.setPower((drive - strafe - inputTurn) * 0.2);
            fl.setPower((drive + strafe + inputTurn) * 0.2);
            br.setPower((drive + strafe - inputTurn) * 0.2);
            bl.setPower((drive - strafe + inputTurn) * 0.2);
        }
        else {
            fr.setPower((drive - strafe - inputTurn) * 1);
            fl.setPower((drive + strafe + inputTurn) * 1);
            br.setPower((drive + strafe - inputTurn) * 1);
            bl.setPower((drive - strafe + inputTurn) * 1);
        }
    }
    public double pid(double target, double kp, double ki, double kd, double error) {
        integral += error;
        double derivative = error - lastError;
        error = target - error;
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        return correction;
    }
}
