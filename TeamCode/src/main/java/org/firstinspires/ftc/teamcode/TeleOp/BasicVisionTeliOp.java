package org.firstinspires.ftc.teamcode.TeleOp;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;



@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends OpMode {
    private BasicVisionProcessor visionProcessor = new BasicVisionProcessor();
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size size = new Size(1280,800);
    @Override
    public void init() {
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .addProcessor(visionProcessor)
                .setCameraResolution(size)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionProcessor, 0);


    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){

    }

    @Override
    public void loop() {


    }
    @Override
    public void stop(){

    }
}
