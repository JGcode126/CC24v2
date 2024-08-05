package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MantisBlade;

@TeleOp(name = "MantisBlade", group = "Iterative OpMode")
public class blade extends OpMode{
    MantisBlade mantisBlade;

    @Override
    public void init() {
        mantisBlade = new MantisBlade(hardwareMap);
    }

   @Override
    public void start() {

    }

    //240,-945


    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.25 && mantisBlade.data1() != -940) {
            mantisBlade.extend(-.05);
        } else if (gamepad1.right_trigger > 0.25 && mantisBlade.data1() != 250) {
            mantisBlade.retract(.05);
        } else {
            mantisBlade.still();
        }

        //mantisBlade.failSafe();

        telemetry.addData("target position", mantisBlade.data2());
        telemetry.addData("current position", mantisBlade.data1());
        telemetry.update();

    }

    @Override
    public void stop() {

    }
}
