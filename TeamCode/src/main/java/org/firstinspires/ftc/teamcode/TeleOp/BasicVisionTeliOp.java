package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
//import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;



@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends BaseOpMode {
    private BasicVisionProcessor visionProcessor = new BasicVisionProcessor();
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    @Override
    public void externalInit() {
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessor)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessor, 0);

        waitForStart();
    }

    @Override
    public void externalLoop() {


    }
}
