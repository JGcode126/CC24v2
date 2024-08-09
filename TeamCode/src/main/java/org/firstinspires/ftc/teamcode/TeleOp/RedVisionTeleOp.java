package org.firstinspires.ftc.teamcode.TeleOp;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Vision.RedCone;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="Red Vision", group="Iterative Opmode")
public class RedVisionTeleOp extends BaseOpMode {



    Scoring scoring;
    boolean blue;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size camRev = new Size(800, 600);
    private RedCone visionProcessorRed = new RedCone();

    Size size = new Size(800,600);
    @Override
    public void externalInit() {
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessorRed)
                .setCameraResolution(size)
                .enableLiveView(true)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessorRed, 0);

        waitForStart();
    }

    @Override
    public void externalLoop() {
        multTelemetry.addData("Pos", visionProcessorRed.getRectPos());


    }
}
