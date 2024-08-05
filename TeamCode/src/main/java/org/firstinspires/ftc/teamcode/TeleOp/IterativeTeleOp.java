package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems
    DcMotor arm;
    Drivetrain drive;
    IMU gyro;
    DuckSpinner duckSpinner;
    Servo claw;
    TouchSensor beam;
    //Timer
    ElapsedTime runtime = new ElapsedTime();
    WebcamName webcam1;
    VisionPortal visionPortal;
    org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor visionProcessor;
    AprilTagProcessor aprilTag;

    @Override
    public void init() {
        //Set timer to 0
        beam = hardwareMap.get(TouchSensor.class, "breakbeam");
        arm = hardwareMap.get(DcMotor.class, "arm");
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
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
aprilTag = new AprilTagProcessor.Builder()
        .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
       // .setLensIntrinsics()
        .build();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessors(visionProcessor, aprilTag)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

    }

    @Override
    public void start() {
        //Code that runs when you hit start
            gyro.resetYaw();
    }
    int position = 0;
    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        drive.driveDO(-gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.right_trigger, -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        if (gamepad1.left_trigger == 1) {
            gyro.resetYaw();
        }
        if (gamepad1.left_bumper) {
            duckSpinner.duckSpinnerSwitch(1);
        } else if (gamepad1.right_bumper) {
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
        if (gamepad1.dpad_up) {
            claw.setPosition(0.35);
        }
        if (gamepad1.cross) {
            position = 0;
        }
        if (gamepad1.triangle) {
            position = 750;
        }
        if (beam.isPressed()) {
            claw.setPosition(0);
        }
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Heading", -gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop(){
        //Code that runs when you hit stop
        telemetry.addData("Runtime", runtime);
        telemetry.update();
    }

}
