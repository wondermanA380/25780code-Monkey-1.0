
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

@TeleOp(name="BlueTeleOp", group="OpMode")
public class BlueTeleOp extends LinearOpMode{

    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor outtakeSlide;
    private DcMotor intakeSlide;
    private Servo wrist;
    private Servo transfer;
    private CRServo leftclaw;
    private CRServo rightclaw;
    private ColorSensor colorSensor;
    private Servo leftclawSpecimen;
    private Servo rightclawSpecimen;
    private TouchSensor intakeTouch;
    private TouchSensor outtakeTouch;


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
        rightclaw = hardwareMap.get(CRServo.class, "rightclaw");
        leftclaw = hardwareMap.get(CRServo.class, "leftclaw");
        transfer = hardwareMap.get(Servo.class, "bucketServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "frontColorSensor");
        leftclawSpecimen = hardwareMap.get(Servo.class, "leftclawSpecimen");
        rightclawSpecimen = hardwareMap.get(Servo.class, "rightclawSpecimen");
        intakeTouch = hardwareMap.get(TouchSensor.class, "intakeTouch");
        outtakeTouch = hardwareMap.get(TouchSensor.class, "outtakeTouch");

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

            float outtakeSpeed = 0.75f;
            float intakeSpeed = 0.75f;

            colorSensor.enableLed(false);

            float intakeSlideLimit = (3.75f * 384.5f) + 55.2f;
            float outtakeSlideTopBasket = (7.75f * 384.5f);
            float outtakeSlideBottomBasket = (4f * 384.5f);


            boolean isIntakeSlideBusy;
            boolean isOuttakeSlideBusy;
            transfer.setPosition(.95);

            intakeSlide.setPower(-1);
            ElapsedTime intakeInitializationTimer = new ElapsedTime();
            intakeInitializationTimer.startTime();
            while (!intakeTouch.isPressed() && intakeInitializationTimer.time(TimeUnit.MILLISECONDS)  < 1000 ) {

            }
            intakeSlide.setPower(0);
            intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*outtakeSlide.setPower(-1);
            ElapsedTime outtakeslideInitializationTimer = new ElapsedTime();
            outtakeslideInitializationTimer.startTime();
            telemetry.addData("outtake touchsensor",outtakeTouch.isPressed() );
            telemetry.update();
            while (!outtakeTouch.isPressed() && outtakeslideInitializationTimer.time(TimeUnit.MILLISECONDS)  < 5000 ) {
                telemetry.addData("outtake initialization",outtakeslideInitializationTimer.time(TimeUnit.MILLISECONDS) );
                telemetry.update();
            }

            telemetry.addData("outtake initialization Done",outtakeslideInitializationTimer.time(TimeUnit.MILLISECONDS) );
            telemetry.update();
            outtakeSlide.setPower(0);
            outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            while (opModeIsActive()) {

                //telemetry data for testing
                telemetry.addData("outtake slide pos", outtakeSlide.getCurrentPosition());
                telemetry.addData("color sensor red", colorSensor.red());
                telemetry.addData("color sensor blue", colorSensor.blue());
                telemetry.addData("color sensor green", colorSensor.green());
                telemetry.addData("color sensor alpha", colorSensor.alpha());
                telemetry.addData("intake slide position", intakeSlide.getCurrentPosition());
                telemetry.update();


                //mecanum drive movement
                robotMovemnt(0.8f);


                //intake slide movement: manual
                if (intakeSlide.getCurrentPosition() < intakeSlideLimit) {
                    intakeSlide.setPower(intakeSpeed * (gamepad1.right_trigger - gamepad1.left_trigger));
                } else if (intakeSlide.getCurrentPosition() >= intakeSlideLimit) {
                    intakeSlide.setPower(-0.5);
                }

                //outtake slide momvemet: manual
                //outtakeSlide.setPower(outtakeSpeed * (gamepad2.left_trigger - gamepad2.right_trigger));

                if (outtakeSlide.getCurrentPosition() < outtakeSlideTopBasket) {
                    outtakeSlide.setPower(outtakeSpeed * (gamepad2.right_trigger - gamepad2.left_trigger));
                } else if (outtakeSlide.getCurrentPosition() >= outtakeSlideTopBasket) {
                    outtakeSlide.setPower(0);
                }
                //ejection code for blue sample
                intakeHandling();

                if(gamepad1.dpad_down){
                    outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                //wrist movement: manual
                if (gamepad2.dpad_down) {

                    //moves wirst down
                    wrist.setPosition(-1);

                } else if (gamepad2.dpad_up) {

                    //moves wrist up
                    wrist.setPosition(1);
                }

                if (gamepad2.left_bumper) {
                    transfer.setPosition(0.55);
                } else if (gamepad2.right_bumper) {
                    transfer.setPosition(0.95);
                }

                /*if (outtakeTouch.isPressed()) {
                    outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                 if (intakeTouch.isPressed()) {
                    intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }*/

                //if color is not blue: will automaticaly move the wrist up, retract the intake linear slide, and transfer the sample from the intake to the outtake
                if (gamepad2.b) {
                    leftclaw.setPower(0);
                    rightclaw.setPower(0);
                    wrist.setPosition(1);

                    timer.reset();
                    outtakeSlide.setPower(Power(384.5, 0, "Retract", outtakeSlide));

                    if (outtakeSlide.getPower() != 0) {
                        isOuttakeSlideBusy = true;
                    } else {
                        isOuttakeSlideBusy = false;
                    }
                    while (isOuttakeSlideBusy && opModeIsActive()) {
                        outtakeSlide.setPower(Power(384.5, 0, "Retract", outtakeSlide));
                        telemetry.addLine("outtake is busy");
                        telemetry.addData("outtake slide position", outtakeSlide.getCurrentPosition());
                        telemetry.update();

                        robotMovemnt(.75f);

                        if (outtakeSlide.getCurrentPosition() < 15) {
                            isOuttakeSlideBusy = false;
                        } else if (gamepad2.dpad_left) {
                            outtakeSlide.setPower(0);

                            isOuttakeSlideBusy = false;
                        }
                    }
                    outtakeSlide.setPower(0);
                    transfer.setPosition(0.9);
                    while(wrist.getPosition() != 1 && transfer.getPosition() != 0.9 && opModeIsActive()){
                        robotMovemnt(.75f);
                    }
                    intakeSlide.setPower(Power(384.5, 0, "Retract", intakeSlide));

                    if (intakeSlide.getPower() != 0) {
                        isIntakeSlideBusy = true;
                    } else {
                        isIntakeSlideBusy = false;
                    }

                    while (isIntakeSlideBusy && opModeIsActive()) {

                        telemetry.addLine("intake is busy");
                        telemetry.addData("intake slide position", intakeSlide.getCurrentPosition());
                        telemetry.update();

                        robotMovemnt(.75f);

                        if (intakeSlide.getCurrentPosition() < 15) {
                            isIntakeSlideBusy = false;
                        } else if ( gamepad2.dpad_right) {
                            intakeSlide.setPower(0);
                            isIntakeSlideBusy = false;
                        }
                    }
                    //are changing the way is busy is calculatee
                    if(intakeSlide.getCurrentPosition() < 15 || intakeTouch.isPressed()){
                        telemetry.addData("intake slide position", intakeSlide.getCurrentPosition());
                        telemetry.update();

                        intakeSlide.setPower(0);

                        robotMovemnt(0.75f);

                        telemetry.addLine("intake is done");
                        telemetry.update();
                        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        outtakeTimer.reset();

                        while (outtakeTimer.milliseconds() <= 250) {
                            robotMovemnt(.75f);
                        }
                        while ((colorSensor.blue() > 500 || colorSensor.alpha() > 500)&& opModeIsActive()) {
                            telemetry.addLine("detects color sensor values");
                            telemetry.addData("color sensor red", colorSensor.red());
                            telemetry.addData("color sensor blue", colorSensor.blue());
                            telemetry.addData("color sensor green", colorSensor.green());
                            telemetry.addData("color sensor alpha", colorSensor.alpha());
                            telemetry.update();
                            leftclaw.setPower(0.7);
                            rightclaw.setPower(-0.7);

                            robotMovemnt(0.75f);
                            if(colorSensor.blue() <500 && colorSensor.alpha()<500)
                            {
                                telemetry.addLine("Breaking out of color check loop");
                                telemetry.update();
                                break;
                            }
                        }

                        leftclaw.setPower(0);
                        rightclaw.setPower(0);

                        outtakeTimer.reset();
                        while (outtakeTimer.milliseconds() <= 125) {
                            robotMovemnt(.75f);
                        }

                        wrist.setPosition(0.6);
                    }


                    //once Y is pressed, outtake slide will move all the way up and drop the sample into the high bucket
                } else if (gamepad2.y) {
                    leftclawSpecimen.setPosition(1);
                    rightclawSpecimen.setPosition(-1);

                    timer.reset();

                    outtakeSlide.setPower(Power(384.5, 7.75, "Extend", outtakeSlide));
                    if (outtakeSlide.getPower() != 0) {
                        isOuttakeSlideBusy = true;
                    } else {
                        isOuttakeSlideBusy = false;
                    }

                    while (isOuttakeSlideBusy) {
                        robotMovemnt(0.75f);
                        outtakeSlide.setPower(Power(384.5, 7.75, "Extend", outtakeSlide));

                        if (outtakeSlide.getCurrentPosition() >= (outtakeSlideTopBasket - 15)) {
                            isOuttakeSlideBusy = false;
                        }
                    }
                    outtakeSlide.setPower(0);
                    outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    transfer.setPosition(0.55); //might change the value to be positive

                } else if (gamepad2.x) {
                    leftclawSpecimen.setPosition(1);
                    rightclawSpecimen.setPosition(-1);

                    timer.reset();
                    outtakeSlide.setPower(Power(384.5, 4, "Extend", outtakeSlide));
                    if (outtakeSlide.getPower() != 0) {
                        isOuttakeSlideBusy = true;
                    } else {
                        isOuttakeSlideBusy = false;
                    }

                    while (isOuttakeSlideBusy) {
                        robotMovemnt(0.75f);

                        outtakeSlide.setPower(Power(384.5, 4, "Extend", outtakeSlide));

                        if (outtakeSlide.getCurrentPosition() >= (outtakeSlideBottomBasket - 15)) {
                            isOuttakeSlideBusy = false;
                        }
                    }
                    outtakeSlide.setPower(0);
                    outtakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    transfer.setPosition(0.55); //might change the value to be positive
                }

                //manually ejects the sample from the intake
                if (gamepad1.x) {
                    leftclaw.setPower(0.9);
                    rightclaw.setPower(-0.9);


                } else{
                    intakeHandling();

                }

                //closes the specimen claw
                if (gamepad1.a) {
                    leftclawSpecimen.setPosition(-1);
                    rightclawSpecimen.setPosition(0.5);
                }

                //opens the specimen claw
                if (gamepad1.b) {
                    leftclawSpecimen.setPosition(1);
                    rightclawSpecimen.setPosition(-1);

                }

                //slows the robots movements for precice  movement
                if(gamepad1.right_bumper){
                    robotMovemnt(0.01f);

                }else{
                    //turns the speed back up
                    robotMovemnt(0.75f);
                }

                if (gamepad2.dpad_right) {
                    outtakeSlide.setPower(0);
                    //outtakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

            }
        }

    }


    //motor PIDF contorller
    public double Power(double MotorPPR, double SlideRotations, String Direction, DcMotor Motor) {
        double targetPos = (SlideRotations * MotorPPR);
        double currPos = Motor.getCurrentPosition();

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

        return (error * Kp) + (integralSum * Ki) + (derivative * Kd);
    }

    public void intakeHandling() {


        if (colorSensor.red() > 500 && colorSensor.alpha() < 500) {
            leftclaw.setPower(0.9);
            rightclaw.setPower(-0.9);
        } else if(gamepad2.a) {
            leftclaw.setPower(-0.9);
            rightclaw.setPower(0.9);
        } else {
            leftclaw.setPower(0);
            rightclaw.setPower(0);
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

    public void intakeSlideMovement(float intakeSpeed, float slideLimit) {
        if (intakeSlide.getCurrentPosition() < slideLimit) {
            intakeSlide.setPower(intakeSpeed * (gamepad1.right_trigger - gamepad1.left_trigger));
        } else if (intakeSlide.getCurrentPosition() >= slideLimit) {
            intakeSlide.setPower(-0.5);
        }
    }

    public void outtakeSlideMovement (float outtakeSpeed, float slideLimit) {
        if (outtakeSlide.getCurrentPosition() < slideLimit) {
            outtakeSlide.setPower(outtakeSpeed * (gamepad2.right_trigger - gamepad2.left_trigger));
        } else if (outtakeSlide.getCurrentPosition() >= slideLimit) {
            outtakeSlide.setPower(0);
        }
    }

    public void outtakeSample() {
        if (gamepad2.x) {
            leftclaw.setPower(0.9);
            rightclaw.setPower(-0.9);
        }
    }
}