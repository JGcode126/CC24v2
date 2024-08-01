package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckSpinner {
    CRServo duckSpinner;

    public DuckSpinner(HardwareMap hardwareMap) {
        duckSpinner = hardwareMap.get(CRServo.class, "duckSpin");
    }

    public void duckSpinnerSwitch(int condition) {
        switch (condition) {
            case 0:
                duckSpinner.setPower(0);
                break;
            case 1:
                duckSpinner.setPower(1);
                break;
            case 2:
                duckSpinner.setPower(-1);
                break;
        }
    }
}