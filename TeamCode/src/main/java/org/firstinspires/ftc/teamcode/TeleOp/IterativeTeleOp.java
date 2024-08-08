package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

//import static org.firstinspires.ftc.teamcode.Utilities.DashConstants.PIDdash.lineScal;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoreState.DOWN;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoreState.PCLOSE;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoreState.RCLOSE;
import static org.firstinspires.ftc.teamcode.Subsystems.Scoring.ScoreState.TRANSFERUP;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.setOpMode;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends BaseOpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerArm = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    Scoring scoring;
    boolean blue;



//    public static boolean superSlow;

    @Override
    public void externalInit() {

        setOpMode(this);
        //Set timer to 0
        timer.reset();
        timerArm.reset();
        //Code that runs when you hit init
        dt = new Drivetrain(hardwareMap);
        scoring = new Scoring(hardwareMap);





//        gyro = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        gyro.initialize(parameters);
//        scoring = new Scoring(hardwareMap);
    }

    @Override
    public void externalInitLoop() {
        if (gamepad1.a) {
            blue = true;
        }
        if (gamepad1.b) {
            blue = false;
        }
        telemetry.addData("Are you on blue alliance", blue);
    }

    @Override
    public void externalStart(){
        //Code that runs when you hit start

    }

    @Override
    public void externalLoop() {
        //Code that *LOOPS* after you hit start
        if (gamepad1.left_bumper) {
            scoring.setScoreState(RCLOSE);
        }
        if (gamepad1.right_bumper) {
            scoring.setScoreState(PCLOSE);
        }
        if (gamepad1.dpad_down) {
            if (timerArm.seconds() > .6) {
                scoring.setScoreState(DOWN);
                timerArm.reset();
            } else {
                timerArm.reset();
            }

        }
        if (gamepad1.dpad_up) {
            scoring.setScoreState(TRANSFERUP);
        }
        if (gamepad1.left_trigger > .05) {
            scoring.spin(blue);
        } else {
            scoring.stopSpin();
        }
        if (scoring.ring()) {
            if (timer.seconds() > 5) {
                scoring.setScoreState(RCLOSE);
                timer.reset();
            } else if (scoring.pixel()) {
                if (timer.seconds() > 5) {
                    scoring.setScoreState(PCLOSE);
                    timer.reset();
                }
            } else {
                timer.reset();
            }
        }

        dt.driving(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);


        scoring.scoring();

        BaseOpMode.addData("Ring", scoring.ring());
        BaseOpMode.addData("Pixel", scoring.pixel());
        BaseOpMode.addData("H", scoring.getH());
        BaseOpMode.addData("V", scoring.getV());
        BaseOpMode.addData("S", scoring.getS());

        scoring.resetHSV();

    }

    @Override
    public void externalStop(){
        //Code that runs when you hit stop

        BaseOpMode.addData("Runtime", timer);

    }
}
