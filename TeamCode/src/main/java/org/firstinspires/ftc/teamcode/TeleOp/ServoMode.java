package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoMode {
    Servo servo;
    static CRServo spin;

    public ServoMode(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class,"arm");
        spin = hardwareMap.get(CRServo.class, "spinner");

    }

    public void pos1(){servo.setPosition(0.42);}

    public void spinServo(){spin.setPower(1);}

    public void stopSpin(){spin.setPower(0);}

    public void armBack(){servo.setPosition(0);}

    public void spinN(){spin.setPower(-1);}
}
