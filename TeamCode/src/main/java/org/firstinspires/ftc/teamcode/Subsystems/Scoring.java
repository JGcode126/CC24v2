package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ArmSwitchStatement.DOWNOPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ArmSwitchStatement.UPCLOSED;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ArmSwitchStatement.UPOPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.SpinDuck.OFF;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.SpinDuck.ON;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoLOpen;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoRClosed;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash.servoROpen;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.ScoringDash;

import java.text.BreakIterator;

public class Scoring {
    Servo servoR;
    Servo servoL;
    Servo servoArm;
    CRServo duckSpinner;
    RevColorSensorV3 clawSensor;
    public SpinDuck spinDuck = OFF;
    public ArmSwitchStatement armSwitch = UPOPEN;
    ElapsedTime timer = new ElapsedTime();
    ScoringDash scoringDash;
    MultipleTelemetry multTelemetry;
    TouchSensor  clawBreakBeam;
    boolean firstRun = true;





    public Scoring(HardwareMap hardwareMap){
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
        duckSpinner = hardwareMap.get(CRServo.class, "duckSpinner");
        clawSensor = hardwareMap.get(RevColorSensorV3.class, "clawSensor" );
        scoringDash = new ScoringDash();
        multTelemetry = new MultipleTelemetry();
        clawBreakBeam = hardwareMap.get(TouchSensor.class, "clawBreakBeam");

    }
    public void open(){
        servoR.setPosition(servoROpen);
        //opens forward
        servoL.setPosition(servoLOpen);
        // opens backwards
    }
    public int getR(){
        return clawSensor.red();
    }
    public int getG(){
        return clawSensor.green();
    }
    public int getB(){
        return clawSensor.blue();
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
        if (clawBreakBeam.isPressed()) {
            if (firstRun){
                firstRun = false;
                timer.reset();}
            if (timer.seconds() > 1) {
                setArmState(UPCLOSED);
            }
        }

    }
    public  void  openDown(){
        open();
        armDown();
        setArmState(UPOPEN);
    }
    public void closedDown(){
        closed();
        armDown();
        if(clawColorBlue() && clawDistClose()){
            setArmState(DOWNOPEN);
        } else if (clawColorRed() && clawDistClose()) {
            setArmState(DOWNOPEN);
        }
    }
    public boolean clawColorYellow(){
        if(clawSensor.red() > 70 && clawSensor.green() > 110 && clawSensor.blue() > 50){
            return true;
        } else {
            return false;
        }
    }
public void autoOpenYellow(){
        if(clawColorYellow() && clawDistClose()){
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
    public boolean getBeam(){
        return clawBreakBeam.isPressed();
    }


    public boolean clawDistClose(){
        return clawSensor.getDistance(DistanceUnit.MM) < scoringDash.getDistThreshold();
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
        double dist = clawSensor.getDistance(DistanceUnit.MM);
        multTelemetry.addData("beam", clawBreakBeam.isPressed());
        multTelemetry.update();
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
        firstRun = true;
        armSwitch = state;
   }

    public ArmSwitchStatement getArmSwitch() {
        return armSwitch;
    }
}
