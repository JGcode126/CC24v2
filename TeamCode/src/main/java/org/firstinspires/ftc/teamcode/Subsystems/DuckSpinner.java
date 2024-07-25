package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckSpinner {
    CRServo spinner;

    public DuckSpinner(HardwareMap hardwareMap) {
        spinner = hardwareMap.get(CRServo.class, "spinner");
    }

    public void Spin(boolean blue) {
        if (blue) {
            spinner.setPower(1);
        }
        if (!blue) {
            spinner.setPower(-1);
        }
    }
}
