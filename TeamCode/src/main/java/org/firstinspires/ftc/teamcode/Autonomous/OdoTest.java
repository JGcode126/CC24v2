package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.KCP.Localization.Location.location;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;
 @Autonomous(name="OdoTest", group="OdoTestMode")
public class OdoTest extends BaseOpMode{

    TwoWheelOdometry odo;

    @Override
    public void externalInit() {
        odo = new TwoWheelOdometry(0, 0, 0);
        odo.localize();

    }

    @Override
    public void externalLoop() {
        odo.localize();
        BaseOpMode.addData("x", Location.x() / 2);
        BaseOpMode.addData("y", Location.y() * 10);
        BaseOpMode.addData("h", Location.heading());


    }
}
