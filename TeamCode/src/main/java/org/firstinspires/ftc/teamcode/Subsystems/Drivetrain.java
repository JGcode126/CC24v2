package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public void drive(double drive, double strafe, double turn, double speed){
        fl.setPower((drive + strafe + turn) * speed);
        fr.setPower((drive - strafe - turn) * speed);
        bl.setPower((drive - strafe + turn) * speed);
        br.setPower((drive + strafe - turn) * speed);
    }
}
