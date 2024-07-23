package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    //Declare Motors
    DcMotor motor;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    //Callable drive functions
    public void drive(){
        motor.setPower(1);
    }
}
