package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import android.graphics.Point;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;


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

        gyro.resetYaw();
        //Code that runs when you hit start

    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        dt.driveDO(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), gamepad1.right_trigger, gamepad1.dpad_right );

        if (gamepad1.left_bumper) {
            dt.duckSpinner(1);
        } else if (gamepad1.dpad_down) {
            dt.liftArm(-.75);
        } else if (gamepad1.dpad_up) {
            dt.liftArm(.75);
        } else if (gamepad1.dpad_left) {
            dt.claw(-0.15);
        } else if (gamepad1.dpad_right) {
            dt.claw(0.15);
        } else if (gamepad1.right_bumper) {
            dt.duckSpinner(-1);
        } else {
            dt.duckSpinner(0);
            dt.liftArm(0);
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


        //dt.drivenonDO(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger);



        telemetry.addData("input turn", dt.inputTurn);
        telemetry.addData("release angle", dt.releaseAngle);
        telemetry.addData("target", dt.target);
        telemetry.addData("target angle", dt.targetAngle);
        telemetry.addData("Heading", gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop



        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
