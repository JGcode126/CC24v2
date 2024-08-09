package org.firstinspires.ftc.teamcode.TeleOp;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Vision.BlueCone;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="Blue Vision", group="Iterative Opmode")
public class BlueVisionTeleOp extends BaseOpMode {



    Scoring scoring;
    boolean blue;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size camRev = new Size(800, 600);
    private BlueCone visionProcessorBlue = new BlueCone();

    Size size = new Size(800,600);
    @Override
    public void externalInit() {
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessorBlue)
                .setCameraResolution(size)
                .enableLiveView(true)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessorBlue, 0);

        waitForStart();
    }

    @Override
    public void externalLoop() {
        multTelemetry.addData("Pos", visionProcessorBlue.getRectPos());


    }
}
