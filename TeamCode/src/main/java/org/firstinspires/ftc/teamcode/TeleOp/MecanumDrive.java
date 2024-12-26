package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.TeleOp.Constants.*;

@TeleOp(name="MecanumDrive", group="OpMode")
public class MecanumDrive extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeSlide, outtakeSlide, hangMotor;
    private Servo intakeClaw, intakeWrist, outtakeClaw, outtakeWrist;

    ElapsedTime outtakeTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        outtakeSlide = hardwareMap.get(DcMotor.class, "outtakeSlide");
        intakeSlide = hardwareMap.get(DcMotor.class, "intakeSlide");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                robotMovement();
                slideControls();
                sampleScoring();
                hanging();
            }
        }
    }

    public void robotMovement(){
        robotMovement(DEFUALT_ROBOT_SPEED);
    }

    public void robotMovement(float speedPower){
        float vertical = -gamepad1.left_stick_y;
        float horizontal = gamepad1.left_stick_x;
        float pivot = gamepad1.right_stick_x;

        backLeft.setPower(speedPower * (pivot + vertical - horizontal));
        backRight.setPower(speedPower * (-pivot + vertical + horizontal));
        frontLeft.setPower(speedPower * (pivot + vertical + horizontal));
        frontRight.setPower(speedPower * (-pivot + vertical - horizontal));
    }

    public void slideControls() {
        // TODO: See if can be made smoother (PID?)
        if (intakeSlide.getCurrentPosition() < INTAKE_SLIDE_LIMIT) {
            intakeSlide.setPower(INTAKE_SPEED * (gamepad1.right_trigger - gamepad1.left_trigger));
        } else if (intakeSlide.getCurrentPosition() >= INTAKE_SLIDE_LIMIT) {
            intakeSlide.setPower(-0.5);
        }

        // TODO: See if can be made smoother (PID?)
        if (outtakeSlide.getCurrentPosition() < OUTTAKE_SLIDE_TOP_BASKET) {
            outtakeSlide.setPower(OUTTAKE_SPEED * (gamepad2.right_trigger - gamepad2.left_trigger));
        } else if (outtakeSlide.getCurrentPosition() >= OUTTAKE_SLIDE_TOP_BASKET) {
            outtakeSlide.setPower(0);
        }
    }

    public void sampleScoring() {
        if (gamepad2.dpad_down) {
            //intake wrist down
            intakeWrist.setPosition(IW_DOWN);
        } else if (gamepad2.dpad_up) {
            //intake wrist up
            intakeWrist.setPosition(IW_UP);
        } else if (gamepad2.b) {
            //intake claw open
            intakeClaw.setPosition(IC_OPEN);
        } else if (gamepad2.a) {
            //intake claw close
            intakeClaw.setPosition(IC_CLOSE);
        } else if (gamepad2.x) {
            //transfer to outtake
            intakeWrist.setPosition(IW_UP);
            outtakeClaw.setPosition(OC_OPEN);
            timer(500);
            outtakeWrist.setPosition(OW_DOWN);
            timer(1000);
            outtakeClaw.setPosition(OC_CLOSE);
            timer(500);
            intakeClaw.setPosition(IC_OPEN);
        } else if (gamepad2.dpad_left) {
            outtakeWrist.setPosition(OW_DOWN);
        } else if (gamepad2.dpad_right) {
            outtakeWrist.setPosition(OW_UP);
        } else if (gamepad2.left_bumper) {
            outtakeClaw.setPosition(OC_CLOSE);
        } else if (gamepad2.right_bumper) {
            outtakeClaw.setPosition(OC_OPEN);
        }
    }

    public void hanging() {
        hangMotor.setPower(gamepad2.left_stick_y);
    }

    public void timer(float milliseconds) {
        outtakeTimer.reset();
        while (outtakeTimer.milliseconds() <= milliseconds) {
            robotMovement();
        }
    }
}