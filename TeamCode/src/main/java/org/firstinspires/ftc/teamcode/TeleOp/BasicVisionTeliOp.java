package org.firstinspires.ftc.teamcode.TeleOp;


import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BasicVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;



@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends OpMode {
    private BasicVisionProcessor visionProcessor = new BasicVisionProcessor();
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    Size size = new Size(1280, 720);

    public void init() {
        telemetry.update();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                .setCameraResolution(size)
                .addProcessor(visionProcessor)
                .build();


        FtcDashboard.getInstance().startCameraStream(visionProcessor, 60);

    }
@Override
   public void start() {
        System.out.println("never gonna see this");
    }

    @Override
    public void loop() {

    }


}
