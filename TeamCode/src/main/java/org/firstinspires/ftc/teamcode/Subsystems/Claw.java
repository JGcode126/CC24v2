package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

        public enum ClawState {
                OPEN, WAIT, CLOSE,
        }
        ClawState currentClawState = ClawState.OPEN;

        TouchSensor breakbeam;

        Servo claw;

        ElapsedTime time;

        public Claw(HardwareMap hardwareMap) {
                breakbeam = hardwareMap.get(TouchSensor.class, "clawSensor");
                claw = hardwareMap.get(Servo.class, "claw");
                ElapsedTime time = new ElapsedTime();
        }

//        public void close(boolean beamBroken){
//                if (beamBroken) {
//                        clawSensor.isPressed();
//
//                } else {
//                        beamBroken = false;
//                        clawSensor.isPressed();
//                }
//        }

        public boolean breamBroken(){
                return breakbeam.isPressed();
        }

        public void claw(double position) {
                claw.setPosition(position);
        }

        public void update(){
                switch(currentClawState) {
                        case OPEN:
                                claw.setPosition(-0.10);
                                if (!breamBroken()) {currentClawState = ClawState.WAIT; }
                                break;

                        case WAIT:
                                if (breamBroken()) {currentClawState = ClawState.CLOSE; }
                                break;

                        case CLOSE:
                                claw.setPosition(0.20);
                                break;
                }
        }

        public void open(){
                currentClawState = ClawState.OPEN;
        }




}
