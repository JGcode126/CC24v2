package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class spinner {
    CRServo duckSpin;

    public spinner(HardwareMap hardwareMap){
        duckSpin = hardwareMap.get(CRServo.class, "duckSpinner");

    }
}
