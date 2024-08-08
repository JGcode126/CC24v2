package org.firstinspires.ftc.teamcode.KCP.TestOpModes;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.TeleOp.IterativeTeleOp;

@Autonomous(name="Odometer Test", group="Testing")
public class OdometryTest extends BaseOpMode {

    public long lastLoop;

    TwoWheelOdometry odo;

    @Override
    public void externalInit() {
        odo = new TwoWheelOdometry(0,0,0);

//        odo = new TwoWheelOdometry(0,0,0);

        odo.localize();
        multTelemetry.addData("status","initialized");
        multTelemetry.addData("Init Heading",Location.heading());

    }

    @Override
    public void externalInitLoop(){
        odo.localize();
        multTelemetry.addData("Init Loop Heading",Location.heading());
    }

    @Override
    public void externalStart() {
        lastLoop = System.nanoTime();
    }

    @Override
    public void externalLoop() {
//        apriltags2.update();
//        BaseOpMode.addData("numDetections", apriltags2.getNumDetections());
//        if (apriltags2.getClosestByID(2) != null){
//            odo.localize(2,apriltags2.getClosestByID(2).ftcPose.x/2.54, apriltags2.getClosestByID(2).ftcPose.y/2.54,Math.toRadians(apriltags2.getClosestByID(2).ftcPose.yaw),2);
//            BaseOpMode.addData("AprilTagY", apriltags2.getClosestByID(2).ftcPose.y/2.54);
//            BaseOpMode.addData("AprilTagX", apriltags2.getClosestByID(2).ftcPose.x/2.54);
//            BaseOpMode.addData("AprilTagHeading", Math.toRadians(apriltags2.getClosestByID(2).ftcPose.yaw));
//        } else {
        //if it doesn't detect anything, use deadwheels
        odo.localize();
    //}

        double[] Raw = odo.getRawValues();
//            double dTime = System.nanoTime() - lastLoop;
//            lastLoop = System.nanoTime();
//            double hz = 1000000000.0 / (dTime);
//            WolfpackOpMode.addData("Heartz <3", hz);
        BaseOpMode.addData("OdoX", Location.x());
        BaseOpMode.addData("OdoY", Location.y());
        BaseOpMode.addData("OdoHeading", Location.heading());
//
        BaseOpMode.addData("\t", " ");
        BaseOpMode.addData("Veritical Encoder", Raw[0]);
        BaseOpMode.addData("Horizontal Encoder", Raw[1]);
    }
}
