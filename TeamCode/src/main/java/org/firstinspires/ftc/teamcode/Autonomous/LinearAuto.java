package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.common.reflection.qual.GetMethod;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;


public class LinearAuto extends LinearOpMode {

    // Declare Subsystems

    public void initialize(){
        // Initialize Subsystems
        setOpMode(this);
    }


    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if(opModeIsActive()){
            //Run Auto

        }
    }
}
