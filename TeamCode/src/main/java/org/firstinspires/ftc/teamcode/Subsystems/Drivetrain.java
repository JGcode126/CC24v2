package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public void drive(double y, double x, double heading, float slowmode){
    bl.setPower(y + heading - x);
    br.setPower(y - heading + x);
    fr.setPower(y - heading - x);
    fl.setPower(y + heading + x);

    if (slowmode == 1){
        bl.setPower((y + heading - x)*0.3);
        br.setPower((y - heading + x)*0.3);
        fr.setPower((y - heading - x)*0.3);
        fl.setPower((y + heading + x)*0.3);
    }
    }
}
