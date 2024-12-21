package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Teleop for 12/21 scrimage", group="OpMode")
public class ClawTestTeleop extends LinearOpMode{

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private Servo wrist;
    private Servo intakeClaw;
    private Servo transfer;
    private Servo leftclawSpecimen;
    private Servo rightclawSpecimen;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime outtakeTimer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 0.9;

    public float backMotors;
    public float frontMotors;

    public float Vertical;
    public float Horizontal;
    public float Pivot;

    float outtakeSpeed = 0.75f;
    float intakeSpeed = 0.75f;


    float intakeSlideLimit = (3.75f * 384.5f) + 55.2f;
    float outtakeSlideTopBasket = (7.75f * 384.5f);
    float outtakeSlideBottomBasket = (4f * 384.5f);

    public boolean detectsColorLoop = false;

    @Override
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        outtakeSlide = hardwareMap.get(DcMotor.class, "outtakeSlide");
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        transfer = hardwareMap.get(Servo.class, "bucketServo");



        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        outtakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {





            while (opModeIsActive()) {
                robotMovemnt(0.8f);
                intakeHandling();
                slideHandlings();
                outtakeHandling();
            }
        }

    }
    public void slideHandlings(){
        if (intakeSlide.getCurrentPosition() < intakeSlideLimit) {
            intakeSlide.setPower(intakeSpeed * (gamepad1.right_trigger - gamepad1.left_trigger));
        } else if (intakeSlide.getCurrentPosition() >= intakeSlideLimit) {
            intakeSlide.setPower(-0.5);
        }


        if (outtakeSlide.getCurrentPosition() < outtakeSlideTopBasket) {
            outtakeSlide.setPower(outtakeSpeed * (gamepad2.right_trigger - gamepad2.left_trigger));
        } else if (outtakeSlide.getCurrentPosition() >= outtakeSlideTopBasket) {
            outtakeSlide.setPower(0);
        }
    }

    public void intakeHandling() {

        if(gamepad2.y){
            intakeClaw.setPosition(.58);
        }
        if(gamepad2.a){
            intakeClaw.setPosition(0.475);
        }
        if(gamepad2.b){
            wrist.setPosition(1);
        }
        if(gamepad2.x){
            wrist.setPosition(-1);
        }

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

    public void outtakeHandling(){
        if (gamepad2.left_bumper) {
            transfer.setPosition(0.55);
        } else if (gamepad2.right_bumper) {
            transfer.setPosition(0.95);
        }

        //opens the specimen claw
    }

}