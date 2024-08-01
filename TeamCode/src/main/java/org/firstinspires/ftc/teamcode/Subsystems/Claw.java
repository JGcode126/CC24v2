package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Claw {
    TouchSensor beam;
    Servo claw;

    public void Servo(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        beam = hardwareMap.get(TouchSensor.class, "breakbeam");
    }
}