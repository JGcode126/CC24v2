package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.Vector2d;

public class Drivetrain {
    //Declare Motors
    DcMotor motorfr;

    DcMotor motorfl;

    DcMotor motorbr;

    DcMotor motorbl;


    public Drivetrain(HardwareMap hardwareMap) {
        //Instantiate motors
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        //settingreverse
        motorfl.setDirection(DcMotorSimple.Direction.REVERSE);
        motorbr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Callable drive functions
    public void drive(double drivePower){
    motorfr.setPower(drivePower);
    motorfl.setPower(drivePower);
    motorbr.setPower(drivePower);
    motorfl.setPower(drivePower);
    }
    public void driveDO(double drive, double strafe, double turn, double slow, double heading){
        Vector2d driveVector  = new Vector2d(strafe, drive);
        Vector2d rotatedvector = driveVector.rotate(Math.toRadians(heading));
        drive = rotatedvector.y;
        strafe  = rotatedvector.x;


        if(slow > 0.05){
            motorfl.setPower((drive + strafe + turn) * -0.25);
            motorfr.setPower((drive + strafe + turn) * -0.25);
            motorbl.setPower((drive + strafe + turn) * -0.25);
            motorbr.setPower((drive + strafe + turn) * -0.25);
        }else {
            motorfl.setPower((drive + strafe + turn) * -0.75);
            motorfr.setPower((drive + strafe + turn) * -0.75);
            motorbl.setPower((drive + strafe + turn) * -0.75);
            motorbr.setPower((drive + strafe + turn) * -0.75);
        }


    }

  /*  public class PID{
        private double proportionalW;
        private double integralW;
        private double derivativeW;

        private boolean isTuning = true;

        private double integralSum = 0;

        private double previousError = 0;
        private long previousTime = System.currentTimeMillis();

        public PID(double proportional, double integral, double derivative){
        this.proportionalW = proportional;
        this.integralW = integral;
        this.derivativeW = derivative;
        }
        public double update(double error, boolean isTuning){
            integralSum += error;

            double deltaTime = (System.currentTimeMillis() - previousTime) / 1000;
            double deltaError = error - previousError;
            double rateOfChange = deltaError/deltaTime;

            previousError = error;
            previousTime = System.currentTimeMillis();

            double pComponent = error * proportionalW;
            double iComponent = integralSum * integralW;
            double dComponent = rateOfChange * derivativeW;

            if (isTuning){
                multTelemetry.addData("P", pComponent);
                multTelemetry.addData("I", iComponent);
                multTelemetry.addData("D", dComponent);
            }

            return pComponent + iComponent + dComponent;
        }
    }*/
}
