package org.firstinspires.ftc.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Scoring {
    Servo clawR;
    Servo clawL;
    Servo arm;
    CRServo spinner;
    ScoreState scoreState;
    public double distance = 9;

    RevColorSensorV3 colorSensor;
    boolean bLedOn = true;


    float hsvValues[] = {0F,0F,0F};

    public Scoring(HardwareMap hardwareMap) {
        clawR = hardwareMap.get(Servo.class, "rightClaw");
        clawL = hardwareMap.get(Servo.class, "leftClaw");
        arm = hardwareMap.get(Servo.class, "arm");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");
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
        return getH() > 50 && getH() < 65 && colorSensor.getDistance(DistanceUnit.MM) < distance;

    }

    public boolean colorSensorRed() {
        return getH() > 13 && getH() < 23 && colorSensor.getDistance(DistanceUnit.MM) < distance;
    }

    public boolean colorSensorBlue() {
        return getH() > 208 && getH() < 212 && getS() > 0.6 && getS() < 0.9 && colorSensor.getDistance(DistanceUnit.MM) < distance;
    }

    // PIXELS

    public boolean colorSensorWhite() {
        return getH() > 154 && getH() < 164 && colorSensor.getDistance(DistanceUnit.MM) < distance;
    }

    public boolean colorSensorPurple() {
        return getH() > 213 && getH() < 217 && getS() > 0.4 && getS() < 0.6 && colorSensor.getDistance(DistanceUnit.MM) < distance;
    }

    public boolean colorSensorGreen() {
        return getH() > 120 && getH() < 129 && colorSensor.getDistance(DistanceUnit.MM) < distance;
    }

    public boolean colorSensorYellow() {
        return getH() > 75 && getH() < 85 && colorSensor.getDistance(DistanceUnit.MM) < distance;
    }

    public boolean pixel() {
        return colorSensorGreen() || colorSensorPurple() || colorSensorYellow() || colorSensorGreen() || colorSensorWhite();
    }

    public boolean ring() {
        return colorSensorBlue() || colorSensorRed() || colorSensorOrange();
    }


    public void ringGrab() {
        clawR.setPosition(0.16);
        clawL.setPosition(0.8);
        boolean bLedOn = false;
    }
    public void intake() {
        arm.setPosition(0);
        clawR.setPosition(0.5);
        clawL.setPosition(0.5);
        boolean bLedOn = true;
    }
    public void pixelGrab() {
        clawR.setPosition(0.1);
        clawL.setPosition(0.9);
        boolean bLedOn = false;
    }
    public void down() {
        arm.setPosition(0.395);
        boolean bLedOn = false;
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
    public void scoring() {
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
    public void setScoreState (ScoreState state) {
        scoreState = state;
        colorSensor.enableLed(bLedOn);
    }
}
