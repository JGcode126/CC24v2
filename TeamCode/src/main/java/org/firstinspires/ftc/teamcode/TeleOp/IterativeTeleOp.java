package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    DuckSpinner duckSpinner;
    Scoring scoring;
    boolean blue;



    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        //Code that runs when you hit init
        dt = new Drivetrain(hardwareMap);

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        gyro.initialize(parameters);

        duckSpinner = new DuckSpinner(hardwareMap);

        while (!gamepad1.b && !gamepad1.a) {
            if (gamepad1.b) {
                blue = true;
                
            }

            if (gamepad1.a) {
                blue = false;
            }
        }

    }

    @Override
    public void start(){
        //Code that runs when you hit start
        gyro.resetYaw();
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        dt.driveDO(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_trigger, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if(gamepad1.left_trigger > .05) {
            duckSpinner.Spin(blue);
        }
        if(gamepad1.left_bumper) {
            scoring.ringGrab();
        }
        if(gamepad1.right_bumper) {
            scoring.pixelGrab();
        }
        if(gamepad1.dpad_down) {
            scoring.armDown();
        }
        if(gamepad1.dpad_up) {
            scoring.transferUp();
        }


    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", runtime);
        telemetry.update();

    }

}
