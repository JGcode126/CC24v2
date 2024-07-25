package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor bl;
    DcMotor br;
    DcMotor fl;
    DcMotor fr;
    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    //Callable drive functions

    public void drive(double drive, double turn, double strafe, double speed) {
        bl.setPower((drive - strafe + turn) * speed);
        br.setPower((drive + strafe - turn) * speed);
        fl.setPower((drive + strafe + turn) * speed);
        fr.setPower((drive - strafe - turn) * speed);
    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading) {
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;

        if (slow > 0.05) {
            fl.setPower(drive + strafe + turn);
            fr.setPower(drive - strafe - turn);
            bl.setPower(drive - strafe + turn);
            br.setPower(drive + strafe - turn);
        }
    }
}
