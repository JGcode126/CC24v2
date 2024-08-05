package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;

@Autonomous(name = "OdoTest")
public class OdometryTest extends BaseOpMode {

    TwoWheelOdometry odo;

    @Override
    public void externalInit(){
        odo = new TwoWheelOdometry(0, 0, 0);
        odo.localize();
    }

    @Override
    public void externalLoop(){
        odo.localize();
        BaseOpMode.addData("x", Location.x());
        BaseOpMode.addData("y", Location.y());
        BaseOpMode.addData("h", Location.heading());
    }
}
