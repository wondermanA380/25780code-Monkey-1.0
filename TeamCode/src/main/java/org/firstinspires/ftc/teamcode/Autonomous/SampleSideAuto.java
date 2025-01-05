package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
@Autonomous(name = "SampleSideAuto")
public class SampleSideAuto extends LinearOpMode {

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
    private IMU imu;
    private IMU gyro_IMU;
    private double lastError = 0;
    private double integralSum = 0;
    private Servo leftclawSpecimen;
    private Servo rightclawSpecimen;
    private TouchSensor intakeTouch;

    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.9;
    ElapsedTime timer = new ElapsedTime();

    public float backMotors;
    public float frontMotors;

    public float Vertical;
    public float Horizontal;
    public float Pivot;
    public float outtakeSlideLimitTopBasket = 384.5f*7.75f;
    public float intakeSlideLimitExtend = 384.5f*3.75f;


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


        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);
        //intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        //initializeVisionPortal();
        stopResetEncoder();
        gyro_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        gyro_IMU.resetYaw();
        float intakeSlideLimit = (3.75f * 384.5f) + 55.2f;

        //myVisionPortal.resumeStreaming();
        waitForStart();

        if (opModeIsActive()) {
            wrist.setPosition(0.6);
            wrist.setPosition(1);
            wrist.setPosition(0.6);
            leftclawSpecimen.setPosition(1);
            rightclawSpecimen.setPosition(-1);
            transfer.setPosition(.95);

            DropPreload();
            PickUpAndDropFisrtSample();
            PickUpAndDropSecodSample();
            Park();
        }
    }


    private void DropPreload (){
        //move back 91.44 cm
        moveBackward(89.94, .75);
        //outtake slide to high basked and drop sample
        DropInHighBasket();

    }
    private void PickUpAndDropFisrtSample() {
        strafeLeft(42.6, .6);
        turnToAngle(254, .55);
        //might change to 270
        boolean samplePickedUp = PickUpAndTransferSample();
        turnToAngle(315, .55);
        moveBackward(24.6, .7);
        if(samplePickedUp) {
            DropInHighBasket();
        }
    }
    private void PickUpAndDropSecodSample() {
        turnToAngle(285, .55);//might change to 20
        moveForward(15, .75);
        boolean samplePickedUp =PickUpAndTransferSample();
        moveBackward(15, .75);
        turnToAngle(315, .55);
        if(samplePickedUp) {
            DropInHighBasket();
        }
    }

    private void Park(){
        turnToAngle(300, .5);

        backleft.setTargetPosition((int) 60 * 18);
        backright.setTargetPosition((int) 60 * 18);
        frontleft.setTargetPosition((int) 60 * 18);
        frontright.setTargetPosition((int) 60 * 18);

        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backleft.setPower(1);
        backright.setPower(1);
        frontleft.setPower(1);
        frontright.setPower(1);

        while (opModeIsActive() && (((frontleft.isBusy()) || (frontright.isBusy())) || ((backleft.isBusy()) || (backright.isBusy())))) {
            intakeSlide.setTargetPosition((int)(384.5*3.75));
            intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeSlide.setPower(1);
        }

        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);

        // Change the orientation if it is not in the orientation that we started with




        stopResetEncoder();

    }
    public boolean PickUpAndTransferSample(){
        boolean samplePickedUp = false;
        // Extend intake linear slide and pick up sample
        intakeSlide.setTargetPosition((int)(384.5*3.75));
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(.5);
        wrist.setPosition(-1);
        while (intakeSlide.isBusy()) {
            leftclaw.setPower(-0.9);
            rightclaw.setPower(0.9);
            if(frontColorSensor.alpha() > 500){
                telemetry.addData("alpha", frontColorSensor.alpha());
                telemetry.update();//rename to back color sensor
                leftclaw.setPower(0);
                rightclaw.setPower(0);
                samplePickedUp = true;
            }

        }
        telemetry.addLine("intake is done");
        telemetry.update();
        leftclaw.setPower(0);
        rightclaw.setPower(0);
        intakeSlide.setPower(0);

        if(frontColorSensor.alpha() > 500) {
            samplePickedUp = true;
        }

        // Retract intake linear slide
        intakeSlide.setTargetPosition(0);
        intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(.8);
        wrist.setPosition(1);
        while (intakeSlide.isBusy() && !intakeTouch.isPressed() && intakeSlide.getCurrentPosition() > 20) {
            telemetry.addData("intake touch", intakeTouch.isPressed());
            telemetry.addData("intake pos", intakeSlide.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addLine("intake retracted");
        telemetry.update();
        intakeSlide.setPower(0);

        // Send to transfer
        while(frontColorSensor.alpha() > 500) {//rename to back color sensor
            telemetry.addLine("in transfer loop");
            telemetry.update();
            leftclaw.setPower(0.7);
            rightclaw.setPower(-0.7);
        }
        telemetry.addLine("transfer loop finished");
        leftclaw.setPower(0);
        rightclaw.setPower(0);

        return  samplePickedUp;
    }
    public void DropInHighBasket() {
        // Extend out take linear slide to  drop sample
        wrist.setPosition(0.6);
        outtakeSlide.setTargetPosition((int)(384.5*7.75));
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide.setPower(1);
        while (outtakeSlide.isBusy() && opModeIsActive()) {

        }
        outtakeSlide.setPower(0);

        transfer.setPosition(.55);
        sleep(1250);
        transfer.setPosition(.95);
        outtakeSlide.setTargetPosition(0);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide.setPower(1);
        while (outtakeSlide.isBusy() && opModeIsActive() && outtakeSlide.getCurrentPosition() > 100) {
            telemetry.addData("outtake pos", outtakeSlide.getCurrentPosition());
            telemetry.update();
        }
        outtakeSlide.setPower(0);
        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void highBasket () {
        //robot faces sideways (backside facing basket) with preloaded sample
        //move back and set linear slide position
        //move back and drop sample into high basket
        //move forward and bring linear slide down
        //turn 90 degrees
        //move forward and extend linear slide
        //pick up sample
        //drop into transfer
        //go back and extend vertical linear slide
        //go back and drop sample in the high basket
        //
    }

    private void specimenHang () {
        moveForward(4,0.5);
        strafeLeft(30,0.5);
        moveBackward(25,0.5);
        leftclawSpecimen.setPosition(-0.5);
        rightclawSpecimen.setPosition(0.5);
        outtakeSlide.setTargetPosition(-50); //todo: find rotations of slide
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //moveBackward();
        //turnToAngle(180,0.5);
        //strafeRight();
        //moveBackward(); todo: move back while extending outtake slide
        //outtakeSlide.setPower(Power(384.5, 7.75, "Extend", outtakeSlide));
        while (outtakeSlide.isBusy()){
            robotMovemnt(.75f);
        }

        while (leftclawSpecimen.getPosition() != 0.5 && rightclawSpecimen.getPosition()!= -0.5){
            telemetry.addData("specimenClawsBusy", leftclawSpecimen.getPosition());
            telemetry.update();
        }
        //move forward
        //turn 180
        //strafe right
        //go forward and position linear slide
        //go forward a little and move the linear slide down to hang
        //stay to park
    }

    private void twoSamplePush () {
        moveForward(50,0.5);
        strafeRight(52,0.5);
        moveForward(75,0.5);
        strafeRight(30,0.5);
        moveBackward(105,0.5);
        moveForward(105,0.5);
        strafeRight(35,0.5);
        moveBackward(105,0.5);
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

    public void robotMovemnt(float speedPower){
        backMotors = speedPower;
        frontMotors = speedPower;

        Vertical  = -(gamepad1.left_stick_y);
        Horizontal = gamepad1.left_stick_x;
        Pivot = gamepad1.right_stick_x;

        backleft.setPower(backMotors * (Pivot + (Vertical - Horizontal)));
        backright.setPower(backMotors * ((-Pivot) + (Vertical + Horizontal)));
        frontleft.setPower(frontMotors * (Pivot + (Vertical + Horizontal)));
        frontright.setPower(frontMotors * ((-Pivot) + (Vertical - Horizontal)));
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

        if (Direction.equals("Extend") && currPos >= targetPos) {
            return 0.1;
        } else if (Direction.equals("Retract") && currPos <= targetPos) {
            return 0;
        }

        double error = targetPos - currPos;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

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
        double error = robotStartOrientation.getYaw()-robotEndOrientation.getYaw();
        if (Math.abs(error) > 2) {
            telemetry.addData("error",Math.abs(error));
            telemetry.update();

        }



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

        }
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);

        YawPitchRollAngles robotEndOrientation = gyro_IMU.getRobotYawPitchRollAngles();
        double error = robotStartOrientation.getYaw()-robotEndOrientation.getYaw();
        if (Math.abs(error) > 2) {
            telemetry.addData("error",Math.abs(error));
            telemetry.update();

        }


        stopResetEncoder();
    }

    public void strafeRight (double distance, double power) {

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
        stopResetEncoder();
    }

    public void strafeLeft (double distance, double power) {

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