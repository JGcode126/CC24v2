package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;

    DcMotor motorfl;

    DcMotor motorbr;

    DcMotor motorbl;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        //settingreverse
        motorfl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorbr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drivePower){
    motorfr.setPower(drivePower);
    motorfl.setPower(drivePower);
    motorbr.setPower(drivePower);
    motorfl.setPower(drivePower);
    }
}
