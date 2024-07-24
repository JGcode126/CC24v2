package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void driverOriented(double drive, double strafe, double turn, double heading, double speed, double slow) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;
        if (slow >= 0.05) {
            fl.setPower((drive + strafe + turn) * (speed / 2));
            fr.setPower((drive - strafe - turn) * (speed / 2));
            bl.setPower((drive - strafe + turn) * (speed / 2));
            br.setPower((drive + strafe - turn) * (speed / 2));
        } else {
            fl.setPower((drive + strafe + turn) * speed);
            fr.setPower((drive - strafe - turn) * speed);
            bl.setPower((drive - strafe + turn) * speed);
            br.setPower((drive + strafe - turn) * speed);
        }
    }
}
