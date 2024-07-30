package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckSpinner {
    CRServo duckSpin;

    public DuckSpinner(HardwareMap hardwareMap) {
        duckSpin = hardwareMap.get(CRServo.class, "duckSpin");
    }

    public void spinDuckForward() {
        duckSpin.setPower(1);
        duckSpin.setPower(0);
    }
    public void spinDuckBackward() {
        duckSpin.setPower(-1);
        duckSpin.setPower(0);
    }
}
