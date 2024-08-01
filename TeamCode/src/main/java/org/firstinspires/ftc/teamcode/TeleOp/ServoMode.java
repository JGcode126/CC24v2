package org.firstinspires.ftc.teamcode.TeleOp;

import android.security.keystore.StrongBoxUnavailableException;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

public class ServoMode {
    Servo servo;
    static CRServo spin;

    public enum dSpinner {
        MOVING, STILL, LEFT
    }
    public enum ArmBackForward {
        FORWARD, BACK
    }
    dSpinner currentState;
    ArmBackForward currentState1;
    HardwareMap breakBeam;
    RevTouchSensor touchSensor;
    DigitalChannel digitalTouch;


    public ServoMode(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class,"arm");
        spin = hardwareMap.get(CRServo.class, "spinner");

    }
    public HardwareMap Breakbeam(HardwareMap hardwareMap){
        digitalTouch = hardwareMap.get(DigitalChannel.class, "breakBeam");;
    return breakBeam;
    }







    public void pos1(){servo.setPosition(0.42);}

    public void spinServo(){
        spin.setPower(1);

    }

    public void stopSpin(){spin.setPower(0);}

    public void armBack(){servo.setPosition(0);}

    public void spinN(){spin.setPower(-1);}
    public void spinState(dSpinner state){
        switch(state){
            case MOVING:
                currentState = dSpinner.MOVING;
                spinServo();
                break;
            case STILL:
                currentState = ServoMode.dSpinner.STILL;
                stopSpin();
                break;
            case LEFT:
                currentState = ServoMode.dSpinner.LEFT;
                spinN();
                break;

        }

    }

    public void armState(ArmBackForward state1) {
        switch (state1){
            case BACK:
                currentState1 = ArmBackForward.FORWARD;
                pos1();
                break;
            case FORWARD:
                currentState1 = ArmBackForward.BACK;
                armBack();
                break;
        }
}
    }

