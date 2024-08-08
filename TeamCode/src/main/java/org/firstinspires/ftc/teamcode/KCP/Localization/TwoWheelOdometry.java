package org.firstinspires.ftc.teamcode.KCP.Localization;

import static org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode.hardware;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.hardwareMap;
import static org.firstinspires.ftc.teamcode.Utilities.OpModeUtils.multTelemetry;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.BaseOpMode;
import org.firstinspires.ftc.teamcode.Sensors.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.zLibraries.HardwareDevices.MotorEncoder;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TwoWheelOdometry extends Location{

    //et variables and measure precision of instruments
    private double imuOffset = 0;
    private int imuNumber = 0;

    //location variables to be read by other classes

    //what the imu reading should be rounded to for maximum accuracy
    double IMUMaximumPrecision = 0.01;
    double IMUMaxNum;

    //defining the center of the robot to be the center of the robot
    //Values For JamieV2
    double horizontalOffset = 0.5; //vertical distance from horizontal encoder to center of robot
    double verticalOffset = 6; //
    //Values for Test Chassis
    //double verticalOffset = 7.5;
    //double horizontalOffset  7.25;
    //last vertical and horizontal encoder readings
    double hPrevDist, vPrevDist;

    //new readings
    double newVertical, newHorizontal, newHeading;
    //change in readings / calculated change
    double dVertical, dHorizontal, dHeading, dX, dY;
    public final MotorEncoder verticalEncoder;
    public final MotorEncoder horizontalEncoder;
    public SparkFunOTOS otos;
    //public final BadBNO055 gyro;

    //intialize odometry
    public TwoWheelOdometry(double startX, double startY, double startHeading){
        super(startX, startY);
        IMUMaxNum = 1/IMUMaximumPrecision;

        verticalEncoder = new MotorEncoder(Hardware.verticalEncoder, Hardware.verticalEncoderTicksToCM);
        horizontalEncoder = new MotorEncoder(Hardware.horizontalEncoder, Hardware.horizontalEncoderTicksToCM);

        otos = hardware.get(SparkFunOTOS.class, "sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-3.5, 5, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.01951);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(startX, startY, startHeading);
        otos.setPosition(currentPosition);


        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        verticalEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
        vPrevDist = verticalEncoder.getPosition();
        hPrevDist = horizontalEncoder.getPosition();

        Location.heading = startHeading;

        //     gyro.setOffsetAngle(Math.toDegrees(startHeading));

    }

    @Override
    public void setCurrentHeading(double heading) {
        SparkFunOTOS.Pose2D newPosition = new SparkFunOTOS.Pose2D(x(), y(), heading);
        Location.heading = heading;
    }

    //Uses Odo Pods
    public void localize(){
//
//        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
//        newVertical = verticalEncoder.getPosition();
//        newHorizontal = horizontalEncoder.getPosition();
////        newHeading = gyro.getHeading();
        newHeading = otos.getPosition().h;

        while (newHeading > Math.PI){
            newHeading -= Math.PI;
        }

        while (newHeading < -Math.PI){
            newHeading += Math.PI;
        }

//
//        //get change in heading, Vertical encoder, and horizontal encoder
//        dVertical = newVertical - vPrevDist;
//        dHorizontal = newHorizontal - hPrevDist;
//        dHeading = newHeading - h;
//
//        while (dHeading < -Math.PI) { // For example 355 to 5 degrees
//            dHeading += 2 * Math.PI;
//        }
//        while (dHeading > Math.PI) { // For example 5 to 355 degrees // IDT NECESSARY
//            dHeading -= 2 * Math.PI;
//        }
//
//        //Math: https://www.desmos.com/calculator/sfpde8dhcw - incorrect
//        //needs more images to be properly explained
//
//        //catch the divide by 0
//        //TODO use line based when delta heading in range
//        if(dHeading == 0) {
//            dX = dHorizontal;
//            dY = dVertical;
//        }else {
//
//
//            //normal odometry
//
////            double arcRad = (dVertical - (verticalOffset * dHeading)) / dHeading;
////            ///  \/ highly debated
////            double number = arcRad + dHorizontal - (horizontalOffset * dHeading);
////
////            dX = fastCos(dHeading) * (number) - arcRad;
////            dY = fastSin(dHeading) * (number);
////
////            dX = dHorizontal * fastCos(dHeading) + dVertical * fastSin(dHeading);
////            dY = dHorizontal * fastSin(dHeading) + dVertical * fastCos(dHeading);
//
//            //
//            double arcRad1 = (dVertical - (verticalOffset * dHeading)) / dHeading;
//            //
//            double arcRad2 = (dHorizontal - (horizontalOffset * dHeading)) / dHeading;
//
//
//            dY = Math.sin(dHeading) * arcRad1 - Math.cos(dHeading) * arcRad2 + arcRad2;
//            dX = Math.sin(dHeading) * arcRad2 + Math.cos(dHeading) * arcRad1 - arcRad1;
//        }
//
//
//        //rotating to absolute coordinates vs robot relative calculated above
//        location[0] += Math.cos(h) * dX - Math.sin(h) * dY;
//        location[1] += Math.sin(h) * dX + Math.cos(h) * dY;
        Location.heading = newHeading;
        BaseOpMode.addData("newHeading", newHeading);
//
//        //update reference values to current position
//        vPrevDist = newVertical;
//        hPrevDist = newHorizontal;
        location[0] = otos.getPosition().x;
        location[1] = otos.getPosition().y;
    }



    @Override
    public void aprilLocalize(AprilTagDetection tagNum, double x, double y, double heading, double cameraNumber) {


        double tagID = tagNum.id;

        x -= Constants.camOffsetX;
        y += Constants.camOffsetY;

        if (tagID == 1){
            x = Constants.tag1x - x;
            y = Constants.backdropY - y;

        }
        else if (tagID == 2){
            x = Constants.tag2x - x;
            y = Constants.backdropY - y;
        }
        else if (tagID == 3){
            x = Constants.tag3x - x;
            y = Constants.backdropY - y;
        }
        else if (tagID == 4){
            x = Constants.tag4x - x;
            y = Constants.backdropY - y;
        }
        else if (tagID == 5){
            x = Constants.tag5x - x;
            y = Constants.backdropY - y;
        }
        else if (tagID == 6){
            x = Constants.tag6x - x;
            y = Constants.backdropY - y;
        }
        else if (tagID == 7){
            x = Constants.tag7x - x;
            y = Constants.backWallY + x;
        }
        else if (tagID == 8){
            x = Constants.tag8x - x;
            y = Constants.backWallY + y;
        }
        else if (tagID == 9){
            x = Constants.tag9x - x;
            y = Constants.backWallY + y;
        }
        else if (tagID == 10){
            x = Constants.tag10x - x;
            y = Constants.backWallY + y;
        }

        location[0] = x;
        location[1] = y;
        Location.heading = heading;
    }


    public double[] getRawValues(){
        return new double[]{verticalEncoder.getPosition(), horizontalEncoder.getPosition(), newHeading};
    }

    public double getHighestX(){
        double highestX = 0;
        if (highestX < TwoWheelOdometry.x()){
            highestX = TwoWheelOdometry.x();
        }
        return highestX;
    }
    public double getHighestY(){
        double highestY = 0;
        if (highestY < TwoWheelOdometry.y()){
            highestY = TwoWheelOdometry.y();
        }
        return highestY;
    }


    public double getVelocityY(){
        return velocity[1];
    }

}