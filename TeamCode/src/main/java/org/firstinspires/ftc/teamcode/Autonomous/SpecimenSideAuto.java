package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.lang.Math;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "SpecimenSideAuto")
public class SpecimenSideAuto extends LinearOpMode {

    public DcMotor frontleft;
    public DcMotor backleft;
    public DcMotor frontright;
    public DcMotor backright;
    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private Servo wrist;
    private Servo transfer;
    private CRServo leftclaw;
    private CRServo rightclaw;
    private ColorSensor frontColorSensor;
    private IMU gyro_IMU;
    private double lastError = 0;
    private double integralSum = 0;
    private Servo leftclawSpecimen;
    private Servo rightclawSpecimen;
    private TouchSensor intakeTouch;
    private TouchSensor outtakeTouch;
    private DistanceSensor rearDistanceSensor;

    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.9;
    ElapsedTime timer = new ElapsedTime();

    public float backMotors;
    public float frontMotors;

    public float Vertical;
    public float Horizontal;
    public float Pivot;


    YawPitchRollAngles robotOrientation;
    double currAngle;

    VisionPortal myVisionPortal;
    AprilTagProcessor myAprilTagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        outtakeSlide = hardwareMap.get(DcMotor.class, "outtakeSlide");
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");
        wrist = hardwareMap.get(Servo.class, "wrist");
        rightclaw = hardwareMap.get(CRServo.class, "rightclaw");
        leftclaw = hardwareMap.get(CRServo.class, "leftclaw");
        transfer = hardwareMap.get(Servo.class, "bucketServo");
        frontColorSensor = hardwareMap.get(ColorSensor.class, "frontColorSensor");
        gyro_IMU = hardwareMap.get(IMU.class, "imu");
        leftclawSpecimen = hardwareMap.get(Servo.class, "leftclawSpecimen");
        rightclawSpecimen = hardwareMap.get(Servo.class, "rightclawSpecimen");
        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        outtakeTouch = hardwareMap.get(TouchSensor.class, "outtakeTouch");
        rearDistanceSensor = hardwareMap.get(DistanceSensor.class, "rearDistanceSensor");

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //initializeVisionPortal();
        stopResetEncoder();
        gyro_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        gyro_IMU.resetYaw();
        //myVisionPortal.resumeStreaming();
        waitForStart();
        if (opModeIsActive()) {

            wrist.setPosition(0.6);
             TwoSamplePush();
             SpecimenHang();
        }
    }



    private void SpecimenHang () {
        PickUpSampleAndHang(true);

        turnToAngle(0,0.5);

        strafeRight(86,0.8);

        leftclawSpecimen.setPosition(0.5);
        rightclawSpecimen.setPosition(-0.5);

        moveBackward(24,0.4);

        PickUpSampleAndHang(false);

        strafeDiagonal(130,1);
        stopResetEncoder();

    }

    private void PickUpSampleAndHang(boolean firstTime)
    {
        double robotSpeed = 0.6;
        double firstDistance = 85;
        double secondDistance = 70;
        double strafeDistance = firstTime ? firstDistance : secondDistance;
        leftclawSpecimen.setPosition(-0.5);
        rightclawSpecimen.setPosition(0.5);
        while (leftclawSpecimen.getPosition() != -0.5 && rightclawSpecimen.getPosition()!= 0.5 && opModeIsActive()){
            telemetry.addData("specimenClawsBusy : LeftClawPosition : ", leftclawSpecimen.getPosition());
            telemetry.update();
        }
        sleep(1000);

        moveForward(50,robotSpeed);
        strafeLeft(strafeDistance,robotSpeed);
        turnToAngle(180,robotSpeed);

        outtakeSlide.setTargetPosition((int)(384.5*3.5));
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide.setPower(0.5);
        while(outtakeSlide.isBusy() &&  opModeIsActive()) {}

        moveBackward(17,0.5,15);

        telemetry.addData("robot is away from submersible", rearDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();

        outtakeSlide.setTargetPosition(-(int)(384.5*2.5));
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide.setPower(-1);
        while(outtakeSlide.isBusy() &&  opModeIsActive()) {}

        if(firstTime)
            moveForward(40,robotSpeed);


        if(firstTime) {
            outtakeSlide.setTargetPosition(0);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(-1);
            while (outtakeSlide.isBusy() && opModeIsActive()) {
            }
            outtakeSlide.setPower(0);

            ElapsedTime timer = new ElapsedTime();
            outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlide.setPower(-1);
            timer.startTime();
            while (timer.time(TimeUnit.MILLISECONDS) < 250) {
            }
            outtakeSlide.setPower(0);
        }

    }

    private void TwoSamplePush () {
        moveForward(10,0.6);
        strafeRight(53,0.6);
        moveBackward(5.6, 0.6);
    }

    private void initializeVisionPortal() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setLensIntrinsics(830.595, 830.595, 329.848, 250.309);        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private void stopResetEncoder() {
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double Power(double MotorPPR, double SlideRotations, String Direction, DcMotor Motor) {
        double targetPos = (SlideRotations * MotorPPR);

        double currPos = Motor.getCurrentPosition();

        telemetry.addData("Power calculation target", targetPos);
        telemetry.update();

        if (Direction.equals("Extend") && currPos >= targetPos) {
            timer.reset();
            return 0.1;
        } else if (Direction.equals("Retract") && currPos <= targetPos) {
            timer.reset();
            return 0;
        }

        double error = targetPos - currPos;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        telemetry.addData("Power calculation error", (error * Kp));
        telemetry.update();
        sleep(1000);

        telemetry.addData("Power calculation derivative", (derivative * Kd));
        telemetry.update();
        sleep(1000);

        telemetry.addData("Power calculation power", (error * Kp) + (integralSum * Ki) + (derivative * Kd));
        telemetry.update();
        sleep(1000);

        return (error * Kp) + (integralSum * Ki) + (derivative * Kd);
    }

    public void stopRobot () {

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
        while (opModeIsActive()) {

        }
    }

    private void moveForward (double distance, double power) {
        YawPitchRollAngles robotStartOrientation = gyro_IMU.getRobotYawPitchRollAngles();

        backleft.setTargetPosition((int) distance * 18);
        backright.setTargetPosition((int) distance * 18);
        frontleft.setTargetPosition((int) distance * 18);
        frontright.setTargetPosition((int) distance * 18);

        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backleft.setPower(power);
        backright.setPower(power);
        frontleft.setPower(power);
        frontright.setPower(power);

        while (opModeIsActive() && (((frontleft.isBusy()) || (frontright.isBusy())) || ((backleft.isBusy()) || (backright.isBusy())))) {

        }

        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);

        // Change the orientation if it is not in the orientation that we started with
        YawPitchRollAngles robotEndOrientation = gyro_IMU.getRobotYawPitchRollAngles();
//        double error = robotStartOrientation.getYaw()-robotEndOrientation.getYaw();
//        if (Math.abs(error) > 2) {
//            turnToAngle((int)(robotStartOrientation.getYaw()),0.5);
//        }
        telemetry.addData("move forward starting yaw",robotStartOrientation.getYaw());
        telemetry.addData("move forward ending yaw",robotEndOrientation.getYaw());
        telemetry.update();



        stopResetEncoder();
    }

    private void turnToAngle (int angle, double power) {

        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotOrientation = gyro_IMU.getRobotYawPitchRollAngles();

        if (robotOrientation.getYaw(AngleUnit.DEGREES) < 0) {
            currAngle = Math.abs(robotOrientation.getYaw(AngleUnit.DEGREES));
        } else {
            currAngle = 360 - robotOrientation.getYaw(AngleUnit.DEGREES);
        }
        currAngle = currAngle % 360;
        if (((angle - currAngle) + 360) % 360 < 180) {
            power = -power;
        }
        while (opModeIsActive() && Math.abs(currAngle - angle) > 5) {
            backleft.setPower(-power);
            backright.setPower(power);
            frontleft.setPower(-power);
            frontright.setPower(power);
            robotOrientation = gyro_IMU.getRobotYawPitchRollAngles();

            if (robotOrientation.getYaw(AngleUnit.DEGREES) < 0) {
                currAngle = Math.abs(robotOrientation.getYaw(AngleUnit.DEGREES));
            } else {
                currAngle = 360 - robotOrientation.getYaw(AngleUnit.DEGREES);
            }
            currAngle = currAngle % 360;
            telemetry.update();
        }
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);
        stopResetEncoder();
    }

    public void moveBackward (double distance, double power) {
        moveBackward(distance,power,0);
    }


    public void moveBackward (double distance, double power, double distanceFromObject) {
        YawPitchRollAngles robotStartOrientation = gyro_IMU.getRobotYawPitchRollAngles();
        backleft.setTargetPosition((int) -(distance * 18));
        backright.setTargetPosition((int) -(distance * 18));
        frontleft.setTargetPosition((int) -(distance * 18));
        frontright.setTargetPosition((int) -(distance * 18));
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setPower(-power);
        backright.setPower(-power);
        frontleft.setPower(-power);
        frontright.setPower(-power);

        while (opModeIsActive() && (((frontleft.isBusy()) || (frontright.isBusy())) || ((backleft.isBusy()) || (backright.isBusy())))) {
            telemetry.addData( "distance", rearDistanceSensor.getDistance(DistanceUnit.CM))  ;
            telemetry.update();
            if(distanceFromObject >0 && rearDistanceSensor.getDistance(DistanceUnit.CM) <= distanceFromObject) {
                    break;
            }
        }
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);

        YawPitchRollAngles robotEndOrientation = gyro_IMU.getRobotYawPitchRollAngles();
        double error = robotStartOrientation.getYaw()-robotEndOrientation.getYaw();
        if (Math.abs(error) > 2) {
          //  turnToAngle((int)(robotStartOrientation.getYaw()),0.5);
        }
        telemetry.addData("move forward starting yaw",robotStartOrientation.getYaw());
        telemetry.addData("move forward ending yaw",robotEndOrientation.getYaw());
        telemetry.update();                                                                 stopResetEncoder();
    }

    public void strafeDiagonal (double distance, double power) {
        backleft.setTargetPosition((int) (distance * 20));
        frontright.setTargetPosition((int) (distance * 20));
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setPower(power-0.2);
        frontright.setPower(power);

        while (opModeIsActive() && (((frontleft.isBusy()) || (frontright.isBusy())) || ((backleft.isBusy()) || (backright.isBusy())))) {

        }
        stopResetEncoder();
    }

    public void strafeRight (double distance, double power) {
        YawPitchRollAngles robotStartOrientation = gyro_IMU.getRobotYawPitchRollAngles();

        backleft.setTargetPosition((int) -(distance * 20));
        backright.setTargetPosition((int) distance * 20);
        frontleft.setTargetPosition((int) distance * 20);
        frontright.setTargetPosition((int) -(distance * 20));
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setPower(-power);
        backright.setPower(power);
        frontleft.setPower(power);
        frontright.setPower(-power);

        while (opModeIsActive() && (((frontleft.isBusy()) || (frontright.isBusy())) || ((backleft.isBusy()) || (backright.isBusy())))) {

        }
        YawPitchRollAngles robotEndOrientation = gyro_IMU.getRobotYawPitchRollAngles();
        double error = robotStartOrientation.getYaw()-robotEndOrientation.getYaw();
        if (Math.abs(error) > 2) {
          //  turnToAngle((int) (robotStartOrientation.getYaw()), 0.5);
        }
        telemetry.addData("Strafe Right starting yaw",robotStartOrientation.getYaw());
        telemetry.addData("Strafe Right ending yaw",robotEndOrientation.getYaw());
        telemetry.update();
        stopResetEncoder();
    }

    public void strafeLeft (double distance, double power) {
        YawPitchRollAngles robotStartOrientation = gyro_IMU.getRobotYawPitchRollAngles();
        backleft.setTargetPosition((int) distance * 20);
        backright.setTargetPosition((int) -(distance * 20));
        frontleft.setTargetPosition((int) -(distance * 20));
        frontright.setTargetPosition((int) distance * 20);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setPower(power);
        backright.setPower(-power);
        frontleft.setPower(-power);
        frontright.setPower(power);

        while (opModeIsActive() && (((frontleft.isBusy()) || (frontright.isBusy())) || ((backleft.isBusy()) || (backright.isBusy())))) {

        }

        YawPitchRollAngles robotEndOrientation = gyro_IMU.getRobotYawPitchRollAngles();
        double error = robotStartOrientation.getYaw() - robotEndOrientation.getYaw();
        if (Math.abs(error) > 2) {
            //turnToAngle((int) (robotStartOrientation.getYaw()), 0.5);
        }
        telemetry.addData("Strafe Right starting yaw", robotStartOrientation.getYaw());
        telemetry.addData("Strafe Right ending yaw", robotEndOrientation.getYaw());
        telemetry.update();
        stopResetEncoder();

    }
    /*
    public void AdjustXBasedOnAprilTag (int AprilTagID) {

        boolean FinishMoving = false;
        int RetryAligning = 0;
        int minXrange;
        int maxXrange;

        if ((AprilTagID == 1) || (AprilTagID == 2) || (AprilTagID == 4) || (AprilTagID == 5)) {
            minXrange = 0;
            maxXrange = 2;
        } else {
            minXrange = -2;
            maxXrange = 0;
        }

        double avgXRange = ((maxXrange + minXrange) / 2);

        while (opModeIsActive() && FinishMoving == false) {

            ArrayList<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

            if ((RetryAligning < 3) && (JavaUtil.listLength(myAprilTagDetections) == 0)) {

                telemetry.addLine("No April Tags Detected");
                telemetry.update();

                if ((AprilTagID == 1) || (AprilTagID == 4)) {
                    strafeLeft(5, 0.3);
                } else if ((AprilTagID == 3) || (AprilTagID == 6)) {
                    strafeRight(5, 0.3);
                }
                myAprilTagDetections = myAprilTagProcessor.getDetections();
                RetryAligning += 1;
            } else {
                FinishMoving = true;
            }

            for (AprilTagDetection Detection_item2 : myAprilTagDetections) {
                AprilTagDetection Detection = Detection_item2;
                if (Detection.metadata != null && Detection.id == AprilTagID) {
                    double x = Detection.ftcPose.x;
                    if (x <= minXrange) {
                        strafeRight(2.5 * Math.abs(x - avgXRange), 0.3);
                    } else if (x >= maxXrange) {
                        strafeLeft(2.5 * Math.abs(x - avgXRange), 0.3);
                    } else {
                        backleft.setPower(0);
                        backright.setPower(0);
                        frontleft.setPower(0);
                        frontright.setPower(0);
                        FinishMoving = true;
                    }
                }
            }
        }
    }

    public void GoToRange (int AprilTagID) {
        stopResetEncoder();

        boolean FinishMoving = false;
        int RetryAligning = 0;
        double maxYvalue = 13.5;
        double minYvalue = 12;
        double avyYRange = ((maxYvalue + minYvalue) / 2);

        while ((opModeIsActive()) && (FinishMoving == false)) {

            ArrayList<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

            if ((RetryAligning < 3) && (JavaUtil.listLength(myAprilTagDetections) == 0)) {

                telemetry.addLine("No AprilTags Detected");
                telemetry.update();

                if ((AprilTagID == 1) || (AprilTagID == 4)) {
                    strafeLeft(5, 0.3);
                } else if ((AprilTagID == 3) || (AprilTagID == 6)) {
                    strafeRight(5, 0.3);
                }
                myAprilTagDetections = myAprilTagProcessor.getDetections();
                RetryAligning += 1;

            } else {
                FinishMoving = true;
            }
            for (AprilTagDetection Detection_item3 : myAprilTagDetections) {
                AprilTagDetection Detection = Detection_item3;

                if (Detection.metadata != null && Detection.id == AprilTagID) {

                    double y = Detection.ftcPose.y;
                    if (y >= maxYvalue) {
                        moveBackward(2.5 * Math.abs(y - maxYvalue), 0.3);
                    } else if ( y <= minYvalue) {
                        moveForward(2.5 * Math.abs(y - minYvalue), 0.3);
                    } else {
                        backleft.setPower(0);
                        backright.setPower(0);
                        frontleft.setPower(0);
                        frontright.setPower(0);
                        FinishMoving = true;
                    }
                }
            }
        }
    }
     */
    public void AdjustX (int AprilTagID) {

        boolean FinishMoving = false;
        int minXrange = 0;
        int maxXrange = 0;

        if ((AprilTagID == 11) || (AprilTagID == 12) || (AprilTagID == 13) || (AprilTagID == 14) || (AprilTagID == 15) || (AprilTagID == 16)) {
            minXrange = 0;
            maxXrange = 2;
        }

        double avgXRange = ((maxXrange + minXrange) / 2);

        while (opModeIsActive() && !FinishMoving) {

            ArrayList<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

            for (AprilTagDetection Detection_item2 : myAprilTagDetections) {
                AprilTagDetection Detection = Detection_item2;
                if (Detection.metadata != null && Detection.id == AprilTagID) {
                    double x = Detection.ftcPose.x;
                    if (x <= minXrange) {
                        strafeRight(2.5 * Math.abs(x - avgXRange), 0.3);
                    } else if (x >= maxXrange) {
                        strafeLeft(2.5 * Math.abs(x - avgXRange), 0.3);
                    } else {
                        backleft.setPower(0);
                        backright.setPower(0);
                        frontleft.setPower(0);
                        frontright.setPower(0);
                        FinishMoving = true;
                    }
                }
            }
        }
    }

    public void AdjustRange (int AprilTagID) {
        stopResetEncoder();

        boolean FinishMoving = false;
        double maxYvalue = 13.5;
        double minYvalue = 12;
        double avyYRange = ((maxYvalue + minYvalue) / 2);

        while ((opModeIsActive()) && (FinishMoving == false)) {

            ArrayList<AprilTagDetection> myAprilTagDetections = myAprilTagProcessor.getDetections();

            for (AprilTagDetection Detection_item3 : myAprilTagDetections) {
                AprilTagDetection Detection = Detection_item3;

                if (Detection.metadata != null && Detection.id == AprilTagID) {

                    double y = Detection.ftcPose.y;
                    if (y >= maxYvalue) {
                        moveBackward(2.5 * Math.abs(y - maxYvalue), 0.3);
                    } else if ( y <= minYvalue) {
                        moveForward(2.5 * Math.abs(y - minYvalue), 0.3);
                    } else {
                        backleft.setPower(0);
                        backright.setPower(0);
                        frontleft.setPower(0);
                        frontright.setPower(0);
                        FinishMoving = true;
                    }
                }
            }
        }
    }
}
