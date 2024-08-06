package org.firstinspires.ftc.teamcode.TeleOp;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends BaseOpMode {



    Scoring scoring;
    boolean blue;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size camRev = new Size(1280, 720);
    private BasicVisionProcessor visionProcessor = new BasicVisionProcessor();

    Size size = new Size(1280,720);
    @Override
    public void externalInit() {
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessor)
                .setCameraResolution(size)
                .enableLiveView(true)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessor, 0);

        waitForStart();
    }

    @Override
    public void externalLoop() {


    }
}
