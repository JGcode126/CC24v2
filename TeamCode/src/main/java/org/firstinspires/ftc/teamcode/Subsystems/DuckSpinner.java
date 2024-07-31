package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DuckSpinner {
    CRServo duckSpinner;
    public DuckSpinner(HardwareMap hardwareMap){
        duckSpinner = hardwareMap.get(CRServo.class, "duckspinner");
    }


    public void spin(){
        duckSpinner.setPower(1);

    }
    public void notSpin(){
        duckSpinner.setPower(0);

    }
}
