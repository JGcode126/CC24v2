package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.TeleOp.ServoMode.ArmBackForward.BACK;
import static org.firstinspires.ftc.teamcode.TeleOp.ServoMode.ArmBackForward.FORWARD;
import static org.firstinspires.ftc.teamcode.TeleOp.ServoMode.dSpinner.LEFT;
import static org.firstinspires.ftc.teamcode.TeleOp.ServoMode.dSpinner.MOVING;
import static org.firstinspires.ftc.teamcode.TeleOp.ServoMode.dSpinner.STILL;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
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
    DigitalChannel breakBeam;

    @Override
    public void init() {
        setOpMode(this);
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
        breakBeam = hardwareMap.get(DigitalChannel.class,"breakBeam");
        breakBeam.setMode(DigitalChannel.Mode.INPUT);
    }


    ServoMode.dSpinner spinner;
    @Override
    public void start() {
        gyro.resetYaw();
        //Code that runs when you hit start
    }

    @Override
    public void loop() {
        if (breakBeam.getState()){
            arm.armState(BACK);
        }
        multTelemetry.addData("breakBeam",breakBeam.getState());
        multTelemetry.update();





        dt.driveDO(gamepad2.left_stick_y, gamepad2.left_stick_x, gamepad2.right_stick_x, gamepad2.right_trigger, gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (gamepad2.triangle) {
            arm.armState(BACK);
        }


        if (gamepad2.square) {
            arm.armState(FORWARD);
        }
        if (gamepad2.left_bumper) {
            spin.spinState(LEFT);
        }
        if (gamepad2.right_bumper) {
            spin.spinState(MOVING);
        }
        if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
            spin.spinState(STILL);
        }}


        @Override
        public void stop () {
            //Code that runs when you hit stop

            telemetry.addData("Runtime", runtime);
            telemetry.update();

        }

    }
