package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor.targetDetected;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;



@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends BaseOpMode {
    private BasicVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    @Override
    public void externalInit() {

        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionProcessor = new BasicVisionProcessor();

        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessor, 0);
        BasicVisionProcessor.visionTimer.reset();
        waitForStart();
    }

    @Override
    public void externalLoop() {
multTelemetry.update();
multTelemetry.addData("object detected", targetDetected);
        multTelemetry.addData("H", visionProcessor.centerH);
        multTelemetry.addData("S", visionProcessor.centerS);
        multTelemetry.addData("V", visionProcessor.centerV);

    }


}
