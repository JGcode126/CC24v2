package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ArmSwitchStatement.DOWNOPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ArmSwitchStatement.UPOPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.SpinDuck.OFF;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.SpinDuck.ON;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLOpen;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoRClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoROpen;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash;

public class Scoring {
    Servo servoR;
    Servo servoL;
    Servo servoArm;
    CRServo duckSpinner;
    ColorSensor clawSensor;
    public SpinDuck spinDuck = OFF;
    public ArmSwitchStatement armSwitch = UPOPEN;
    ElapsedTime timer = new ElapsedTime();
    ScoringDash scoringDash;




    public Scoring(HardwareMap hardwareMap){
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");
        clawSensor = hardwareMap.get(ColorSensor.class, "clawSensor" );
        scoringDash = new ScoringDash();
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
    public void openUp(){
        open();
        armUp();
    }
    public  void  openDown(){
        open();
        armDown();
    }
    public void closedDown(){
        closed();
        armDown();
        if(clawColorBlue()){
            setArmState(DOWNOPEN);
        } else if (clawColorRed()) {
            setArmState(DOWNOPEN);
        }
    }
    public void closedUp(){
        closed();
        armUp();
    }
    public void spinDuck(){duckSpinner.setPower(1);
        if (timer.seconds()>15){
          setState(OFF);
        }}
    public boolean clawColorBlue(){
        if (clawSensor.blue() < scoringDash.getBlueThreshold()){
            return true;
        } else {
            return false;
        }
    }
    public boolean clawColorRed(){
        if (clawSensor.red() > scoringDash.getRedThreshold()){
            return true;
        } else {return false;}

    }
    public void stopSpinDuck() {duckSpinner.setPower(0);}
    public double getTime(){
        return timer.seconds();
    }
    public SpinDuck getState(){
        return spinDuck;
    }

    ElapsedTime runtime = new ElapsedTime();

    public enum SpinDuck{
        ON, OFF
    }
   public void spinState(){
       switch (spinDuck) {
           case ON:
               spinDuck();
               break;
           case OFF:
               stopSpinDuck();
               break;
       }
   }
   public enum ArmSwitchStatement{
        UPOPEN, DOWNOPEN, DOWNCLOSED, UPCLOSED
   }
   public void arm() {
       switch (armSwitch) {
           case UPOPEN:
               openUp();
               break;
           case DOWNOPEN:
               openDown();
               break;
           case DOWNCLOSED:
               closedDown();
               break;
           case UPCLOSED:
               closedUp();
               break;
       }
   }
   public void setState(SpinDuck state){

       timer.reset();
       spinDuck = state;
   }
   public  void  setArmState(ArmSwitchStatement state){
        timer.reset();
        armSwitch = state;
   }

    public ArmSwitchStatement getArmSwitch() {
        return armSwitch;
    }
}
