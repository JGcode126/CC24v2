package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLOpen;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoRClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoROpen;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Scoring {
    Servo servoR;
    Servo servoL;
    Servo servoArm;
    CRServo duckSpinner;
    public SpinDuck spinDuck = SpinDuck.OFF;
    ElapsedTime timer = new ElapsedTime();



    public Scoring(HardwareMap hardwareMap){
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");
    }
    public void open(){
        servoR.setPosition(servoROpen);
        //opens forward
        servoL.setPosition(servoLOpen);
        // opens bachwards
    }
    public void closed(){
        servoR.setPosition(servoRClosed);
        servoL.setPosition(servoLClosed);
    }
    public void armUp(){
        servoArm.setPosition(1);
    }
    public void armDown(){
        servoArm.setPosition(0);
    }
    public void spinDuck(){duckSpinner.setPower(1);
        if (timer.seconds()>15){
          setDuckSpinner(SpinDuck.OFF);
        }}
    public void stopSpinDuck() {duckSpinner.setPower(0);}
    public double getTime(){
        return timer.seconds();
    }

    ElapsedTime runtime = new ElapsedTime();

    public enum SpinDuck{
        ON, OFF
    }
   public void setDuckSpinner(SpinDuck newPosition){

       switch (newPosition) {
           case ON:
               spinDuck = SpinDuck.ON;
               spinDuck();
               break;
           case OFF:
               spinDuck = SpinDuck.OFF;
               stopSpinDuck();
               break;
       }
   }
}
