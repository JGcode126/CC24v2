package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain;
    IMU gyro;

    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        //Code that runs when you hit init
        drivetrain = new Drivetrain(hardwareMap);
        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        gyro.initialize(parameters);
    }

    @Override
    public void start(){
        gyro.resetYaw();
        //Code that runs when you hit start
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        drivetrain.driverOriented(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),0.7, gamepad1.right_trigger);
        if (gamepad1.left_trigger >= 0.05) {
            gyro.resetYaw();
        }
    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
