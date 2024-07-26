package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems
    ServoMode arm;
    ServoMode spin;
    IMU gyro;
    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;

    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        dt = new Drivetrain(hardwareMap);

        gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters perameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        //Code that runs when you hit init
        arm = new ServoMode(hardwareMap);
        spin = new ServoMode(hardwareMap);
    }

    @Override
    public void start() {
        gyro.resetYaw();
        //Code that runs when you hit start
    }

    @Override
    public void loop() {
        dt.driveDO(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, gamepad2.right_trigger, gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //Code that *LOOPS* after you hit start
        if (gamepad2.triangle) {
            arm.pos1();
        }
        if (gamepad2.square) {
            arm.armBack();
        }
        if (gamepad2.left_bumper) {
            spin.spinServo();
        }
        if (gamepad2.right_bumper) {
            spin.spinN();
        }
        if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            spin.stopSpin();
        }}


        @Override
        public void stop () {
            //Code that runs when you hit stop

            telemetry.addData("Runtime", runtime);
            telemetry.update();

        }

    }
