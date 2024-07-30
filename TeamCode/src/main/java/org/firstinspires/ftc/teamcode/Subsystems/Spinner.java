package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spinner {
    CRServo spinner;

    public Spinner(HardwareMap hardwareMap){
        spinner = hardwareMap.get(CRServo.class, "spinner");
    }
    public void spinServo(){spinner.setPower(1);}
    public void stopSpinner(){spinner.setPower(1);}
 
}
