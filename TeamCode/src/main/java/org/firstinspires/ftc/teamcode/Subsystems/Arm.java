package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    Servo arm;

    public Arm(HardwareMap hardwareMap){
        arm = hardwareMap.get(Servo.class, "arm");
    }
    public void deposit(){arm.setPosition(0.5);}
    public void pickUp(){arm.setPosition(0.2);}
}
