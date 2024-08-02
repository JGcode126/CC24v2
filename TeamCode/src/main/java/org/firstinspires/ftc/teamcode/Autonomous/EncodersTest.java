package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;

@Autonomous(name = "Encoder Test")
public class EncodersTest extends BaseOpMode{
    TwoWheelOdometry odo;

    @Override
    public void externalInit() {
        odo = new TwoWheelOdometry(0,0,0);
        odo.localize();
    }

    @Override
    public void externalLoop() {
        odo.localize();
        BaseOpMode.addData("x", Location.x());
        BaseOpMode.addData("y", Location.y());
        BaseOpMode.addData("heading", Location.heading());
    }
    //78 / 104.13  * -0.00091912743
    //78 / -52.36 * -0.0005236551576
}
