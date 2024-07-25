package org.firstinspires.ftc.teamcode.Subsystems;

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
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drive, double turn, double strafe){
        bl.setPower(drive + strafe - turn);
        br.setPower( drive - strafe + turn);
        fl.setPower(drive - strafe - turn);
        fr.setPower(drive + strafe + turn);

    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading, Boolean driverOrientied){
        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotate(Math.toRadians(heading));
if (driverOrientied) {
    drive = rotatedVector.y;
    strafe = rotatedVector.x;
}
if (slow == 1){
        bl.setPower((drive + strafe - turn) * .5);
        br.setPower(( drive - strafe + turn) * .5);
        fl.setPower((drive - strafe - turn) * .5);
        fr.setPower((drive + strafe + turn) * .5);


    }else{
        bl.setPower(drive + strafe - turn);
        br.setPower( drive - strafe + turn);
        fl.setPower(drive - strafe - turn);
        fr.setPower(drive + strafe + turn);
        }
    }
}
