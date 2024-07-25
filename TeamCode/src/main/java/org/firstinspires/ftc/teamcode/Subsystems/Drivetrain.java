package org.firstinspires.ftc.teamcode.Subsystems;
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
    double integral = 0;
    double lastError = 0;
    boolean clawDown;

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
    public void driveDO(double drive, double strafe, double turn, double slow, double heading, double medium) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;

        if (slow > 0.05) {
            fr.setPower((drive - strafe - turn) * 0.35);
            fl.setPower((drive + strafe + turn) * 0.35);
            br.setPower((drive + strafe - turn) * 0.35);
            bl.setPower((drive - strafe + turn) * 0.35);
        }
        else if (medium > 0.05) {
            fr.setPower((drive - strafe - turn) * 0.6);
            fl.setPower((drive + strafe + turn) * 0.6);
            br.setPower((drive + strafe - turn) * 0.6);
            bl.setPower((drive - strafe + turn) * 0.6);
        }
        else if (clawDown) {
            fr.setPower((drive - strafe - turn) * 0.35);
            fl.setPower((drive + strafe + turn) * 0.35);
            br.setPower((drive + strafe - turn) * 0.35);
            bl.setPower((drive - strafe + turn) * 0.35);

        } else {
            fr.setPower((drive - strafe - turn) * 1);
            fl.setPower((drive + strafe + turn) * 1);
            br.setPower((drive + strafe - turn) * 1);
            bl.setPower((drive - strafe + turn) * 1);
        }
    }
    public double pid(double target, double kp, double ki, double kd, double heading) {
        double error = target - heading;
        integral += error;
        double derivative = error - lastError;
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        return correction;
    }
}
