package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    //Declare Motors
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

    }

    //Callable drive functions
    public void driveForward(){
        fr.setPower(1);
        fl.setPower(1);
        br.setPower(1);
        bl.setPower(1);

    }
}
