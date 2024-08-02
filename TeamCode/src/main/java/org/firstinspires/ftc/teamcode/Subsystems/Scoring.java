package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Scoring {
    Servo clawR;
    Servo clawL;
    Servo arm;
    CRServo spinner;
    ScoreState scoreState;

    ColorSensor colorSensor;
    boolean bLedOn = true;
    public int getR() {return colorSensor.red()/25;}
    public int getG() {return colorSensor.green()/25;}
    public int getB() {return colorSensor.blue()/25;}

    public Scoring(HardwareMap hardwareMap) {
        clawR = hardwareMap.get(Servo.class, "rightClaw");
        clawL = hardwareMap.get(Servo.class, "leftClaw");
        arm = hardwareMap.get(Servo.class, "arm");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        scoreState = ScoreState.TRANSFERUP;

    }


    // RINGS

    public boolean colorSensorOrange() {
        if (colorSensor.red() > 245 && colorSensor.red() > 255 && colorSensor.green() > 250 && colorSensor.green() > 260 && colorSensor.blue() > 78 && colorSensor.blue() > 88) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorRed() {
        if (colorSensor.red() > 130 && colorSensor.red() < 145 && colorSensor.green() > 70 && colorSensor.green() < 82 && colorSensor.blue() > 47 && colorSensor.blue() < 55) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorBlue() {
        if (colorSensor.red() > 35 && colorSensor.red() < 45 && colorSensor.green() > 105 && colorSensor.green() < 115 && colorSensor.blue() > 240 && colorSensor.blue() < 250) {
            return true;
        } else {
            return false;
        }
    }

    // PIXELS

    public boolean colorSensorWhite() {
        if (colorSensor.red() > 195 && colorSensor.red() < 205 && colorSensor.green() > 330 && colorSensor.green() < 340 && colorSensor.blue() > 290 && colorSensor.blue() < 300) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorPurple() {
        if (colorSensor.red() > 78 && colorSensor.red() < 78 && colorSensor.green() > 96 && colorSensor.green() < 106 && colorSensor.blue() > 135 && colorSensor.blue() < 145) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorGreen() {
        if (colorSensor.red() > 50 && colorSensor.red() < 60 && colorSensor.green() > 164 && colorSensor.green() < 174 && colorSensor.blue() > 59 && colorSensor.blue() < 69) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorYellow() {
        if (colorSensor.red() > 162 && colorSensor.red() < 172 && colorSensor.green() > 233 && colorSensor.green() < 243 && colorSensor.blue() > 61 && colorSensor.blue() < 71) {
            return true;
        } else {
            return false;
        }
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
