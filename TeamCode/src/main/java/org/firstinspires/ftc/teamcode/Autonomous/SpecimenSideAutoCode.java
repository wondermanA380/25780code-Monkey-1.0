package org.firstinspires.ftc.teamcode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name="SpecimenSide", group="Linear OpMode")
public class SpecimenSideAutoCode extends LinearOpMode {

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
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

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
        outtakeSlide.setTargetPosition(0);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlide.setPower(-1);
        while (outtakeSlide.isBusy() && opModeIsActive()) {
        }
        outtakeSlide.setPower(0);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-52)
                .strafeTo(new Vector2d(31.5, -52))
                .setTangent(Math.toRadians(90))
                .lineToY(-59);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(31.5,-59,Math.toRadians(90)))

                .strafeToLinearHeading(new Vector2d(0,-40),Math.toRadians(270));


        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(0,-40,Math.toRadians(270)))
                .lineToY(-35);

        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(0,-35,Math.toRadians(270)))
                .lineToY(-45)
                .strafeToLinearHeading(new Vector2d(37,-45),Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .lineToY(-10)
                .strafeTo(new Vector2d(45,-10))
                .setTangent(Math.toRadians(90))
                .lineToY(-60)
                .lineToY(-10)
                .strafeTo(new Vector2d(55,-10))
                .setTangent(Math.toRadians(90))
                .lineToY(-60);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(0,-35,Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(31.5,-52),Math.toRadians(90))
                .strafeTo(new Vector2d(31.5,-5));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(31.5,-58,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-40),Math.toRadians(270))
                .strafeTo(new Vector2d(0,-40));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(0,-40,Math.toRadians(270)))
                .strafeTo(new Vector2d(0,-35));

        Action trajectoryMoveToSpecimen = tab1.build();
        Action trajectoryPickUpSpecimen = tab2.build();
        Action trajectoryMoveToSubmersible = tab3.build();
        Action trajectoryPushSamples = tab7.build();
        Action trajectoryPickUpSpecimen2 = tab4.build();
        Action trajectoryMoveToSubmersible2 = tab5.build();
        Action trajectoryDropSpecimen = tab6.build();

        waitForStart();
        if (opModeIsActive()) {

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1, 1), 1));

            Actions.runBlocking(trajectoryMoveToSpecimen);

            leftclawSpecimen.setPosition(-0.5);
            rightclawSpecimen.setPosition(0.5);
            while (leftclawSpecimen.getPosition() != -0.5 && rightclawSpecimen.getPosition()!= 0.5 && opModeIsActive()){
                telemetry.addData("specimenClawsBusy : LeftClawPosition : ", leftclawSpecimen.getPosition());
                telemetry.update();
            }
            sleep(1000);

            Actions.runBlocking(trajectoryPickUpSpecimen);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
            outtakeSlide.setTargetPosition((int)(384.5*3.5));
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(0.5);
            while(outtakeSlide.isBusy() &&  opModeIsActive()) {

            }
            outtakeSlide.setPower(0);
            Actions.runBlocking(trajectoryMoveToSubmersible);
            //outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlide.setTargetPosition(-(int)(384.5*3));
            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlide.setPower(-1);
            timer.reset();
            while(outtakeSlide.isBusy() &&  opModeIsActive()) {
                telemetry.addData("outtake slide busy", outtakeSlide.isBusy());
                telemetry.addData("is outtake pressed", outtakeTouch.isPressed());
                telemetry.update();
                if (timer.seconds() > 1){
                    break;
                }
            }
            outtakeSlide.setPower(0);
            leftclawSpecimen.setPosition(0.5);
            rightclawSpecimen.setPosition(-0.5);

            Actions.runBlocking(trajectoryPushSamples);


//            Actions.runBlocking(trajectoryPickUpSpecimen2);
//
//            leftclawSpecimen.setPosition(-0.5);
//            rightclawSpecimen.setPosition(0.5);
//            while (leftclawSpecimen.getPosition() != -0.5 && rightclawSpecimen.getPosition()!= 0.5 && opModeIsActive()){
//                telemetry.addData("specimenClawsBusy : LeftClawPosition : ", leftclawSpecimen.getPosition());
//                telemetry.update();
//            }
//            sleep(1000);
//
//            Actions.runBlocking(trajectoryMoveToSubmersible2);
//
//            outtakeSlide.setTargetPosition((int)(384.5*3.5));
//            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outtakeSlide.setPower(0.5);
//            while(outtakeSlide.isBusy() &&  opModeIsActive()) {}
//
//           Actions.runBlocking(trajectoryDropSpecimen);
//
//            outtakeSlide.setTargetPosition(-(int)(384.5*2.5));
//            outtakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            outtakeSlide.setPower(-1);
//            while(outtakeSlide.isBusy() &&  opModeIsActive()) {}


        }

    }
}


