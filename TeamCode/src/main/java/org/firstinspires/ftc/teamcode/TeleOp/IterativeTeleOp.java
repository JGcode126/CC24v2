package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.angleMode.DEGREES;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems
    Drivetrain drive;
    IMU gyro;
    DuckSpinner duckSpinner;
    Servo claw;
    //Timer
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        //Set timer to 0
        duckSpinner = new DuckSpinner(hardwareMap);
        runtime.reset();
        drive = new Drivetrain(hardwareMap);
        claw = hardwareMap.get(Servo.class, "claw");
        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                gyro.initialize(parameters);
        //Code that runs when you hit init

    }

    @Override
    public void start() {
        //Code that runs when you hit start
            gyro.resetYaw();
    }
    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        drive.driveDO(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (gamepad1.left_trigger == 1) {
            gyro.resetYaw();
        }
        if (gamepad1.left_bumper) {
            duckSpinner.duckSpinnerSwitch(1);
        } else {
            duckSpinner.duckSpinnerSwitch(0);
        }
        if (gamepad1.right_bumper) {
            duckSpinner.duckSpinnerSwitch(2);
        } else {
            duckSpinner.duckSpinnerSwitch(0);
        }
        if (gamepad1.dpad_left) {
            claw.setPosition(0);
        }
        if (gamepad1.dpad_right) {
            claw.setPosition(0.5);
        }
        if (gamepad1.dpad_right) {
            claw.setPosition(0.35);
        }
         telemetry.addData("Heading", -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }



    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
