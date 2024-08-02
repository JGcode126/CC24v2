package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoMove {
    //Declare Motors
    Servo servo;
    CRServo spin;


    public ServoMove(HardwareMap hardwareMap) {
        //Instantiate motors
        servo = hardwareMap.get(Servo.class, "servo");
        spin = hardwareMap.get(CRServo.class, "spinner");

    }

    public void pos1(){
        servo.setPosition(0.5);
    }

    public void spinServo(){
        spin.setPower(1);
    }


    }


