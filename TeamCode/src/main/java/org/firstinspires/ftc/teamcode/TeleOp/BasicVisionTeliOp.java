package org.firstinspires.ftc.teamcode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends BaseOpMode {
    private BasicVisionProcessor visionProcessor = new BasicVisionProcessor();

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    @Override
    public void externalInit() {
        telemetry.update();
        aprilTag = new AprilTagProcessor.Builder()
                //feel free to change units here to whatever works best for you
                .setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS)
                //Input lens intrinsics from your camera here
                .setLensIntrinsics(622.064, 622.064, 309.916, 204.916)
                .build();


        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessors(visionProcessor, aprilTag)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        waitForStart();
    }

    @Override
    public void externalLoop() {


    }
}
