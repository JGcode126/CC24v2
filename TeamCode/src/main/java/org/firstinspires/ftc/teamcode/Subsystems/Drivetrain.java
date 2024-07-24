package org.firstinspires.ftc.teamcode.Subsystems;
import static java.lang.Math.toRadians;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor bl;
    DcMotor br;
    DcMotor fr;
    DcMotor fl;
    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double y, double x, double heading, float slowmode) {

    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading){
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(toRadians(heading));

        drive = rotatedVector.y;
        strafe = rotatedVector.x;
        bl.setPower(drive + heading - strafe);
        br.setPower(drive - heading + strafe);
        fr.setPower(drive - heading - strafe);
        fl.setPower(drive + heading + strafe);

        if (slow == 1) {
            bl.setPower((drive + heading - strafe) * 0.3);
            br.setPower((drive - heading + strafe) * 0.3);
            fr.setPower((drive - heading - strafe) * 0.3);
            fl.setPower((drive + heading + strafe) * 0.3);
        }
    }
}
