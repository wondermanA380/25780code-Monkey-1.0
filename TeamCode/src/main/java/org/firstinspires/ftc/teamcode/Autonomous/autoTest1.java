package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import java.util.ArrayList;
//public class Drive  {
//    public class moveForward implements Action(){
//
//    }
//    public Action followTrajectory(Trajectory t) {
//        return new moveForward();
//    }
//
//    public Action turn(double angle) {
//        return new TodoAction();
//    }
//
//    public Action moveToPoint(double x, double y) {
//        return new TodoAction();
//    }
//}
@Autonomous(name="autoTest1", group="OpMode")
public class autoTest1 extends LinearOpMode {

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor arm;
    private DcMotor wrist;
    private Servo leftclaw;
    private Servo rightclaw;
    private IMU imu;
    private IMU gyro_IMU;
    private DistanceSensor RightSensor;
    private DistanceSensor LeftSensor;
    YawPitchRollAngles robotOrientation;
    double currAngle;

    VisionPortal myVisionPortal;
    AprilTagProcessor myAprilTagProcessor;

    public void stopRobot() {

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

    private void moveForward (double distance, double power){

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
    }

    public void moveBackward (double distance, double power) {

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
    }
    public void runOpMode() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");
        RightSensor = hardwareMap.get(DistanceSensor.class, "RightSensor");
        LeftSensor = hardwareMap.get(DistanceSensor.class, "LeftSensor");
        gyro_IMU = hardwareMap.get(IMU.class, "imu");



        waitForStart();

        if (opModeIsActive()) {


            while (opModeIsActive()) {



                }
            }
        }
    }



