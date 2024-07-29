package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.kd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.ki;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.kp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
DcMotor bl;
DcMotor br;
DcMotor fl;
DcMotor fr;
    double inputTurn;
    double releaseAngle;
double integral = 0;
double derivitive;
double error = 0;
double target;
double oldError;
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

       if(turn != 0){
           inputTurn = turn;
           releaseAngle = heading;
       }else{
           turn = releaseAngle +0.5;
           inputTurn = pid(turn,heading);
       }
if (slow == 1){
        bl.setPower((drive + strafe - inputTurn) * .25);
        br.setPower(( drive - strafe + inputTurn) * .25);
        fl.setPower((drive - strafe - inputTurn) * .25);
        fr.setPower((drive + strafe + inputTurn) * .25);


    }else{
        bl.setPower(drive + strafe - inputTurn);
        br.setPower( drive - strafe + inputTurn);
        fl.setPower(drive - strafe - inputTurn);
        fr.setPower(drive + strafe + inputTurn);


        }}
public double pid(double target, double value){
    error =  target - value;
    if (error>180){error-=360;}
    else if (error<-180){error+=360;}
    integral += error;
    derivitive = error - oldError;

//target - current
    //-360 if >180, +360 if <-180
   double turn = (error * (-kp)) + (integral * (-ki)) + (derivitive * (-kd));
    oldError = error;
    return turn;
    }
}
