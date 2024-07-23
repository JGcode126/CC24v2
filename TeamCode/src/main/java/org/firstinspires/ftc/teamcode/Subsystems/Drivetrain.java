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
        motorfr = hardwareMap.get(DcMotor.class, "drivefr");
        motorfl = hardwareMap.get(DcMotor.class, "drivefl");
        motorbr = hardwareMap.get(DcMotor.class, "drivebr");
        motorbl = hardwareMap.get(DcMotor.class, "drivebl");
        //fr br reverse
        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbr.setDirection(DcMotor.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drive, double strafe, double turn, double slow){

        if (slow > 0.05){
            motorfl.setPower((drive + strafe + turn) * -0.25);
            motorfr.setPower((drive - strafe - turn) * -0.25);
            motorbl.setPower((drive - strafe + turn) * -0.25);
            motorbr.setPower((drive + strafe - turn) * -0.25);
        } else {
            motorfl.setPower((drive + strafe + turn) * -0.75);
            motorfr.setPower((drive - strafe - turn) * -0.75);
            motorbl.setPower((drive - strafe + turn) * -0.75);
            motorbr.setPower((drive + strafe - turn) * -0.75);
        }
    }
}
