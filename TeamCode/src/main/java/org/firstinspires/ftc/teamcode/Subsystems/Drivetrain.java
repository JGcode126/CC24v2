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
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");

        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double driveY, double driveX, double heading){
    motorfr.setPower(driveY - driveX - heading);
    motorfl.setPower(driveY + driveX + heading);
    motorbr.setPower(driveY + driveX - heading);
    motorbl.setPower(driveY - driveX + heading);
    }

    }

