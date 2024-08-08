package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefish.controller.Waypoint;
import com.roboctopi.cuttlefish.queue.CustomTask;
import com.roboctopi.cuttlefish.queue.DelayTask;
import com.roboctopi.cuttlefish.queue.PointTask;
import com.roboctopi.cuttlefish.utils.Pose;

import org.firstinspires.ftc.teamcode.cuddlefish.CuddleInitIDK;


@TeleOp(name="Example Field-Centric Driver Encoders", group="Example ")
public class CuddleOpMode extends CuddleInitIDK {
    public void onInit() {
        super.onInit();
    }
    public void main() {
        super.main();

    }

    public void mainLoop()
    {
        super.mainLoop();
        queue.addTask(new PointTask(new Waypoint(new Pose(0.0,0.0,0.0),0.5), ptpController));

        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Localizer X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Localizer Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Localizer R:",encoderLocalizer.getPos().getR());
        telemetry.update();
    }
}