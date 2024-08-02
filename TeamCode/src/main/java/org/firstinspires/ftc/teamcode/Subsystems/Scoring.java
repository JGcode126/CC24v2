package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Scoring {
    Servo clawR;
    Servo clawL;
    Servo arm;
    CRServo spinner;
    ScoreState scoreState;

    ColorSensor colorSensor;
    boolean bLedOn = true;


    float hsvValues[] = {0F,0F,0F};

    public Scoring(HardwareMap hardwareMap) {
        clawR = hardwareMap.get(Servo.class, "rightClaw");
        clawL = hardwareMap.get(Servo.class, "leftClaw");
        arm = hardwareMap.get(Servo.class, "arm");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        scoreState = ScoreState.TRANSFERUP;
    }

    public void resetHSV() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
    }
    public float getH() {return hsvValues[0];}
    public float getS() {return hsvValues[1];}
    public float getV() {return hsvValues[2];}

    // RINGS

    public boolean colorSensorOrange() {
    }

//    public boolean colorSensorRed() {
//        return getV() > 120 && getV() < 150;
//    }

//    public boolean colorSensorBlue() {
//        return getV() > 100 && getV() < 180;
//    }

    // PIXELS

    public boolean colorSensorWhite() {
        return getV() > 270 && getV() < 450;
    }

    public boolean colorSensorPurple() {
        return getV() > 140 && getV() < 160;
    }

    public boolean colorSensorGreen() {
        return getV() > 100 && getV() < 140;
    }

//    public boolean colorSensorYellow() {
//        return getH() > 2700 && getH() < 2990 && getS() > 3800 && getS() < 4100 && getV() > 950 && getV() < 1150;
//    }

    public void ringGrab() {
        clawR.setPosition(0.16);
        clawL.setPosition(0.8);
    }
    public void intake() {
        arm.setPosition(0);
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
    }
    public void pixelGrab() {
        clawR.setPosition(0.1);
        clawL.setPosition(0.9);
    }
    public void down() {
        arm.setPosition(0.395);
    }

    public void spin(boolean team) {
        if (team) {
            spinner.setPower(1);
        } else if (!team) {
            spinner.setPower(-1);
        }
    }
    public void stopSpin() {
        spinner.setPower(0);
    }

    public enum ScoreState  {
        PCLOSE, RCLOSE, TRANSFERUP, DOWN
    }
    public void scoring () {
        switch (scoreState) {
            case DOWN:
                down();
                break;
            case TRANSFERUP:
                intake();
                break;
            case RCLOSE:
                ringGrab();
                break;
            case PCLOSE:
                pixelGrab();
                break;
        }
    }
    public void setScoreState(ScoreState state) {
        scoreState = state;
    }
}
