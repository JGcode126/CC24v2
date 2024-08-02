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

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.25) {
            mantisBlade.extend(-.6);
        } else if (gamepad1.right_trigger > 0.25) {
            mantisBlade.retract(.6);
        } else {
            mantisBlade.still();
        }

    }

    @Override
    public void stop() {

    }
}
