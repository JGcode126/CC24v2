package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.KCP.Movement;

@Autonomous(name = "Hold Position PID" )
public class HoldPositionPID extends BaseOpMode{

    Movement drive;

    @Override
    public void externalInit() {
        drive = new Movement(0,0,0);
    }

    @Override
    public void externalLoop() {
        drive.update();
        drive.holdPosition(0,0,0);
    }
}
