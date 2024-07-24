package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    //Declare Motors
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;

    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        fr = hardwareMap.get(DcMotor.class, "drivefr");
        fl = hardwareMap.get(DcMotor.class, "drivefl");
        br = hardwareMap.get(DcMotor.class, "drivebr");
        bl = hardwareMap.get(DcMotor.class, "drivebl");
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
    }

    //Callable drive functions

    public void drive(double drive, double strafe, double turn, double slow){
        if (slow > 0.05) {
            fr.setPower((drive - strafe - turn) * 0.25);
            fl.setPower((drive + strafe + turn) * 0.25);
            br.setPower((drive + strafe - turn) * 0.25);
            bl.setPower((drive - strafe + turn) * 0.25);
        }
        else {
            fr.setPower((drive - strafe - turn) * 0.75);
            fl.setPower((drive + strafe + turn) * 0.75);
            br.setPower((drive + strafe - turn) * 0.75);
            bl.setPower((drive - strafe + turn) * 0.75);
        }

    }
}
