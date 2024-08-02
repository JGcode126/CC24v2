package org.firstinspires.ftc.teamcode.zLibraries.HardwareDevices;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;

public class ServoEncoder {

    AnalogInput servo;


    public ServoEncoder(String mapName){
        servo = BaseOpMode.getHardwareMap().get(AnalogInput.class, mapName);
    }

    public double getPositionDegrees(){
        return servo.getVoltage() / 3.3 * 360;
    }

    public double getPositionRadians(){
        return servo.getVoltage() / 3.3 * 6.28;
    }

}
