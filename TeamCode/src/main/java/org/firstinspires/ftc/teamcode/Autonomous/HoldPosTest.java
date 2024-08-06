package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.KCP.Movement;

@Autonomous(name = "holdpos")
public class HoldPosTest extends BaseOpMode{
    Movement drive;

    @Override
    public void externalInit() {
        drive = new Movement(0,0,0);
    }
    public void externalLoop(){
        drive.update();
        drive.hold(0,0,0);
    }
}
