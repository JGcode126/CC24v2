package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Iterative TeleOp Vis", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode{

    //Declare Subsystems
    private BasicVisionProcessor visionProcessor;
    //Timer
    ElapsedTime runtime = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    DuckSpinner duckSpinner;
    Scoring scoring;
    boolean blue;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size camRev = new Size(1280, 720);



    @Override
    public void init() {
        //Set timer to 0
        runtime.reset();
        //Code that runs when you hit init

        telemetry.update();
//        aprilTag = new AprilTagProcessor.Builder()
//                //feel free to change units here to whatever works best for you
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
//                //Input lens intrinsics from your camera here
//                .setLensIntrinsics(622.064, 622.064, 309.916, 204.916)
//                .build();
        visionProcessor = new BasicVisionProcessor();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
//                .addProcessors(visionProcessor, aprilTag)
                .addProcessor(visionProcessor)
//                .setCameraResolution(camRev)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start(){
        //Code that runs when you hit start
    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start

    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

    }

}
