package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    TouchSensor breakBeam;
    TouchSensor limitSwitch;


    @Override   
    public void init() {
        //Set timer to 0
        runtime.reset();

        dt = new Drivetrain(hardwareMap);

        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);


        //Code that runs when you hit init
    }

    @Override
    public void start(){
        //Code that runs when you hit start
    }



    @Override
    public void loop() {
    dt.driveDO(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    }

//    @Override
//    public void stop(){
//        //Code that runs when you hit stop
//
//        telemetry.addData("Runtime", runtime);
//        telemetry.update();
//
//    }


