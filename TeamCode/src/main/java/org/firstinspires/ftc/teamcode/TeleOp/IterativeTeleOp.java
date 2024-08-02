package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.IterativeTeleOp.duckSpin.OFF;
import static org.firstinspires.ftc.teamcode.TeleOp.IterativeTeleOp.duckSpin.ON;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kd;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Ki;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.Kp;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.dGain;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.iGain;
import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.pGain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    IMU gyro;
    Scoring scoring;
    DuckSpinner spin;
    DcMotor arm;
    TouchSensor breakbeam;
    public enum duckSpin {
        ON, OFF
    }
    duckSpin spinner1;
    duckSpin spinner2;


    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        //Code that runs when you hit init
        drivetrain = new Drivetrain(hardwareMap);
        scoring = new Scoring(hardwareMap);
        spin = new DuckSpinner(hardwareMap);
        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        gyro.initialize(parameters);
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        breakbeam = hardwareMap.get(TouchSensor.class, "breakbeam");
        spinner1 = ON;
        spinner2 = ON;
    }

    @Override
    public void start(){
        gyro.resetYaw();
        //Code that runs when you hit start
;
    }

    double turn = 0;
    double error = 0;
    double lastError = 0;
    double integral = 0;
    double derivative = 0;
    double target = 0;
    int distance = 0;
    int armError = 0;
    int armLastError = 0;
    int armP = 0;
    int armI = 0;
    int armD = 0;
    boolean left = false;
    boolean right = false;
    double speed = 0.7;
    boolean goal = true;


    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        if (gamepad1.right_stick_x != 0) {
            drivetrain.driverOriented(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),speed, gamepad1.square);
            target = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        } else {
            error = target - gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            integral += error;
            derivative = lastError - error;
            lastError = error;
            turn = (error * Kp) + (integral * Ki) + (derivative * Kd);
            drivetrain.driverOriented(gamepad1.left_stick_y, -gamepad1.left_stick_x, -turn, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), speed, gamepad1.square);
        }
        if (gamepad1.dpad_up) {
            gyro.resetYaw();
            target = 0;
        }
        if (gamepad1.left_bumper) {
            scoring.open();
        }
        if (breakbeam.isPressed()) {
            goal = false;
            if (runtime.seconds() > 1) {
                scoring.open();
                goal = true;
            }
        }
        if (goal) {
            runtime.reset();
        }
        if (gamepad1.circle) {
            scoring.ring();
        }
        if (gamepad1.right_bumper) {
            scoring.pixel();
        }
        if (gamepad1.dpad_left) {
            runtime.reset();
            left = true;
            right = false;
        }
        if (gamepad1.dpad_right) {
            runtime.reset();
            left = false;
            right = true;
        }
        if (left) {
            switch (spinner1) {
                case ON:
                    if (getRuntime() <= 30) {
                        spinner1 = OFF;
                    }
                    spin.spinDuckForward();
                    break;
                case OFF:
                    break;
            }
        }
        if (right) {
            switch (spinner2) {
                case ON:
                    if (getRuntime() <= 30) {
                        spinner2 = OFF;
                    }
                    spin.spinDuckBackward();
                    break;
                case OFF:
                    break;
            }
        }
        if (distance > 50) {
            speed = 0.3;
        } else {
            speed = 0.7;
        }
        if (gamepad1.right_trigger > 0.05) {
            distance = 825;
            telemetry.addData("dpad down", distance);
        }
        if (gamepad1.left_trigger > 0.05) {
            distance = 50;
            telemetry.addData("dpad up", distance);
        }
        telemetry.addData("heading", gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("target", target);
        arm.setTargetPosition(distance);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armError = distance - arm.getCurrentPosition();
//        armP = armError;
//        armI += armError;
//        armD = armLastError - armError;
//        armLastError = armError;
//        arm.setPower((armP * pGain) + (armI * iGain) + (armD * dGain));
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
