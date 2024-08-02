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

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring;

@TeleOp(name="Iterative TeleOp", group="Iterative Opmode")
public class IterativeTeleOp extends OpMode {

    //Declare Subsystems

    //Timer
    ElapsedTime timer = new ElapsedTime();
    Drivetrain dt;
    IMU gyro;
    Scoring scoring;
    boolean blue;


//    public static boolean superSlow;

    @Override
    public void init() {

        setOpMode(this);
        //Set timer to 0
        timer.reset();
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
    public void init_loop() {
        if (gamepad1.a) {
            blue = true;
        }
        if (gamepad1.b) {
            blue = false;
        }
        telemetry.addData("Are you on blue alliance", blue);
    }

    @Override
    public void start(){
        //Code that runs when you hit start

    }

    @Override
    public void loop() {
        //Code that *LOOPS* after you hit start
        if (gamepad1.left_bumper) {
            scoring.setScoreState(RCLOSE);
        }
        if (gamepad1.right_bumper) {
            scoring.setScoreState(PCLOSE);
        }
        if (gamepad1.dpad_down) {
            scoring.setScoreState(DOWN);
        }
        if (gamepad1.dpad_up) {
            scoring.setScoreState(TRANSFERUP);
        }
        if (gamepad1.left_trigger > .05) {
            scoring.spin(blue);
        } else {
            scoring.stopSpin();
        }
        dt.driving(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_trigger);

        multTelemetry.update();

        scoring.scoring();

        multTelemetry.addData("H", scoring.getH());
        multTelemetry.addData("S", scoring.getS());
        multTelemetry.addData("V", scoring.getV());
//        multTelemetry.addData("Red", scoring.colorSensorRed());
        multTelemetry.addData("Green", scoring.colorSensorGreen());
//        multTelemetry.addData("Blue", scoring.colorSensorBlue());
        multTelemetry.addData("Purple", scoring.colorSensorPurple());
        multTelemetry.addData("Orange", scoring.colorSensorOrange());
        multTelemetry.addData("White", scoring.colorSensorWhite());
//        multTelemetry.addData("Yellow", scoring.colorSensorYellow());

        scoring.resetHSV();

    }

    @Override
    public void stop(){
        //Code that runs when you hit stop

        telemetry.addData("Runtime", timer);
        telemetry.update();

    }
}
