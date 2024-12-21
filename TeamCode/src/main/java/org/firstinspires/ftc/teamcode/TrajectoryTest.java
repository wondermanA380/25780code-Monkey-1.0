package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Timer;

@TeleOp(name="TrajectoryTest", group="Linear OpMode")
public class TrajectoryTest extends LinearOpMode {

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


    @Override
    public void runOpMode() throws InterruptedException {

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


        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ElapsedTime myStopWatch = new ElapsedTime();

        //Claw claw = new Claw(hardwareMap);
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
        sleep(10000);
        wrist.setPosition(0.6);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-52)
                .strafeTo(new Vector2d(31.5, -52))
                .setTangent(Math.toRadians(90))
                .lineToY(-58)

                .lineToY(-40)
                .strafeToLinearHeading(new Vector2d(0,-40),80)

                .strafeTo(new Vector2d(0,-34))

                .strafeTo(new Vector2d(0,-52))
                .strafeToLinearHeading(new Vector2d(31.5,-52),0)
                .strafeTo(new Vector2d(31.5,-58))
                .strafeToLinearHeading(new Vector2d(0,-40),80)
                .strafeTo(new Vector2d(0,-34));
        Action trajectoryActionChosen;
//        leftclawSpecimen.setPosition(-0.5);
//        rightclawSpecimen.setPosition(0.5);
//        while (leftclawSpecimen.getPosition() != -0.5 && rightclawSpecimen.getPosition()!= 0.5 && opModeIsActive()){
//
//        }
        trajectoryActionChosen = tab1.build();

        waitForStart();
        if (opModeIsActive()) {

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.2, 0.2), 0.2));

            Actions.runBlocking(trajectoryActionChosen);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }


//        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-10, -57.5, Math.toRadians(90)))
//
//
//                .lineToY(-42);
//                .splineTo(new Vector2d(35,-35),0)
//                .strafeTo(new Vector2d(35,-10))
////                .strafeTo(new Vector2d(42,-10))
////                .strafeTo(new Vector2d(42,-58));
//
////                .strafeTo(new Vector2d(-10,-37));
//
////                .lineToY(-32)
////                .strafeTo(new Vector2d(-37,-20))
////                .lineToY(-22.5);
//
//        waitForStart();
//        if (opModeIsActive()) {
//            trajectoryActionChosen = tab2.build();
//            Actions.runBlocking(trajectoryActionChosen);
//        }
//        telemetry.addData("x", drive.pose.position.x);
//        telemetry.addData("y", drive.pose.position.y);
//        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
//        telemetry.update();


    }
}

