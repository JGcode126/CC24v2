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
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);


    }
    public float getH() {return hsvValues[0];}
    public float getS() {return hsvValues[0];}
    public float getV() {return hsvValues[0];}

    // RINGS

    public boolean colorSensorOrange() {
        return getH() > 7400 && getH() < 7600 && getS() > 6850 && getS() < 7100 && getV() > 2000 && getV() < 2250;
    }

    public boolean colorSensorRed() {
        return getH() > 5000 && getH() < 5200 && getS() > 2500 && getS() < 2700 && getV() > 1350 && getV() < 1550;
    }

    public boolean colorSensorBlue() {
        return getH() > 850 && getH() < 1050 && getS() > 2550 && getS() < 2750 && getV() > 5500 && getV() < 5750;
    }

    // PIXELS

    public boolean colorSensorWhite() {
        return getH() > 4000 && getH() < 10000 && getS() > 7000 && getS() < 14000 && getV() > 7000 && getV() < 14000;
    }

    public boolean colorSensorPurple() {
        return getH() > 3600 && getH() < 3850 && getS() > 5200 && getS() < 5400 && getV() > 7200 && getV() < 7400;
    }

    public boolean colorSensorGreen() {
        return getH() > 850 && getH() < 1050 && getS() > 2700 && getS() < 3100 && getV() > 950 && getV() < 1150;
    }

    public boolean colorSensorYellow() {
        return getH() > 2700 && getH() < 2990 && getS() > 3800 && getS() < 4100 && getV() > 950 && getV() < 1150;
    }

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
