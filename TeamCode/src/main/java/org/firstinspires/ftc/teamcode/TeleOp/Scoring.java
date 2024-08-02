package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class Scoring {


        Servo claw;
        TouchSensor breakBeam;
        TouchSensor limitSwitch;

        public Scoring(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "claw");
            breakBeam = hardwareMap.get(TouchSensor.class, "breakBeam");
            limitSwitch = hardwareMap.get(TouchSensor.class, "789");

        }
        while(){

        }
    }


