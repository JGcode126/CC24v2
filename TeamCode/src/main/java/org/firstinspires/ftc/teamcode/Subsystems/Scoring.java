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

    public boolean colorSensorOrange() {
        if (colorSensor.red() > 245 && colorSensor.green() > 250 && colorSensor.blue() > 78) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorRed() {
        if (colorSensor.red() > 130 && colorSensor.green() > 70 && colorSensor.blue() > 47) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorBlue() {
        if (colorSensor.red() > 245 && colorSensor.green() > 250 && colorSensor.blue() > 78) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorWhite() {
        if (colorSensor.red() > 225 && colorSensor.green() > 390 && colorSensor.blue() > 345) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorYellow() {
        if (colorSensor.red() > 100 && colorSensor.green() > 150 && colorSensor.blue() > 38) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorGreen() {
        if (colorSensor.red() > 33 && colorSensor.green() > 105 && colorSensor.blue() > 35) {
            return true;
        } else {
            return false;
        }
    }

    public boolean colorSensorPurple() {
        if (colorSensor.red() > 100 && colorSensor.green() > 145 && colorSensor.blue() > 195) {
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
