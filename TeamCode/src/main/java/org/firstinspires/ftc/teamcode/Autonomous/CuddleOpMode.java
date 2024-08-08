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
    boolean tasksQueued = false;

    public void onInit() {
        super.onInit();
    }
    public void main() {
        super.main();
        if(tasksQueued == true){

        }
    }

    public void mainLoop()
    {
        super.mainLoop();

        encoderLocalizer.update();
        System.out.println(encoderLocalizer.getPos());
        telemetry.addData("Localizer X:",encoderLocalizer.getPos().getX());
        telemetry.addData("Localizer Y:",encoderLocalizer.getPos().getY());
        telemetry.addData("Localizer R:",encoderLocalizer.getPos().getR());
        telemetry.update();
    }
}