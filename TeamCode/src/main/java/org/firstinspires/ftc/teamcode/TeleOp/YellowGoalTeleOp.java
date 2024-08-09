package org.firstinspires.ftc.teamcode.TeleOp;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.Vision.BlueGoal;
import org.firstinspires.ftc.teamcode.Vision.RedCone;
import org.firstinspires.ftc.teamcode.Vision.RedGoal;
import org.firstinspires.ftc.teamcode.Vision.YellowGoal;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="Yellow Goal", group="Iterative Opmode")
public class YellowGoalTeleOp extends BaseOpMode {



    Scoring scoring;
    boolean blue;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size camRev = new Size(400, 300);
    private YellowGoal visionProcessorYellow = new YellowGoal();

    Size size = new Size(400,300);
    @Override
    public void externalInit() {
        telemetry.update();
        aprilTag = new AprilTagProcessor.Builder().build();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 2");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessorYellow)
                .setCameraResolution(size)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessorYellow, 0);

        waitForStart();
    }

    @Override
    public void externalLoop() {
        multTelemetry.addData("Correction", YellowGoal.moveToMiddle());
    }
}
