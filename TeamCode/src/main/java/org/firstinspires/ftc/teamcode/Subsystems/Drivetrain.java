package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    //Declare Motors
    DcMotor bl;
    DcMotor br;
    DcMotor fl;
    DcMotor fr;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        bl = hardwareMap.get(DcMotor.class, "drivebl");
        br = hardwareMap.get(DcMotor.class, "drivebr");
        fl = hardwareMap.get(DcMotor.class, "drivefl");
        fr = hardwareMap.get(DcMotor.class, "drivefr");

        //fl bl reverse
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);



    }

    //Callable drive functions
    public void drive(double drive, double strafe, double turn, double speed){

        if (speed > .05) {
            bl.setPower((drive - strafe + turn) * .3);
            br.setPower((drive + strafe - turn) * .3);
            fl.setPower((drive + strafe + turn) * .3);
            fr.setPower((drive - strafe - turn) * .3);
        } else {
            bl.setPower((drive - strafe + turn) * 1);
            br.setPower((drive + strafe - turn) * 1);
            fl.setPower((drive + strafe + turn) * 1);
            fr.setPower((drive - strafe - turn) * 1);
        }


    }
}
