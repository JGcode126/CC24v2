package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    Claw claw;


    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        setOpMode(this);
        dt = new Drivetrain(hardwareMap);
        claw = new Claw(hardwareMap);


        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.UP,
              RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        gyro.initialize(parameters);
        //Code that runs when you hit init
    }

    @Override
    public void start(){

        gyro.resetDeviceConfigurationForOpMode();
        //Code that runs when you hit start

    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        dt.driveDO(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), gamepad1.right_trigger, gamepad1.triangle );
        //dt.altDrive(gamepad1.right_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger);

        if (gamepad1.left_bumper) {
            dt.duckSpinner(1);
        } else if (gamepad1.dpad_down) {
            dt.liftArm(-.5);
        } else if (gamepad1.dpad_up) {
            dt.liftArm(.5);
        } else if (gamepad1.dpad_left) {
            claw.close();

        } else if (gamepad1.dpad_right) {
            claw.open();
        } else if (gamepad1.right_bumper) {
            dt.duckSpinner(-1);
        } else {
            dt.duckSpinner(0);
            dt.liftArm(0);
        }

        if (gamepad1.square) {
            claw.resetBreakBeam();
        }

//        if (gamepad1.triangle) {
//            dt.claw(-0.15);
//        } else if (gamepad1.square) {
//            dt.claw(0.15);
//
//        }

        if (gamepad1.cross) {
            gyro.resetYaw();
        }

//        if (!sense.breamBroken()) {
//            dt.claw(0.20);
//        } else {
//            dt.claw(-0.10);
//        }

        claw.update();





        //dt.drivenonDO(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger);


        multTelemetry.addData("beam broken", claw.breamBroken());
        multTelemetry.addData("input turn", dt.inputTurn);
        multTelemetry.addData("release angle", dt.releaseAngle);
        multTelemetry.addData("target", dt.target);
        multTelemetry.addData("target angle", dt.targetAngle);
        multTelemetry.addData("Heading", gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        multTelemetry.update();
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop



        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
