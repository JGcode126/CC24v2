package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Scoring {
    public class spinner {
        Servo claw;

        public spinner(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "duckSpinner");

        }
    }
}
