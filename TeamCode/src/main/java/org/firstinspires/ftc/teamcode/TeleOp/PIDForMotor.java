package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "PIDForMotor")
public class PIDForMotor extends LinearOpMode {
    DcMotorEx Motor;

    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 1.0;
    public static double Ki = 0.0;
    public static double Kd = 1.0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    //public List<ValuesVector> dictionary = new List<ValuesVector>();

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        Motor = hardwareMap.get(DcMotorEx.class, "LinearSlide");

        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (opModeIsActive()) {
            //Change MotorPPR based on GoBILDA value for the exact motor
            double MotorPPR = 384.5;
            //If LinearSlide was changed, Change this value to how many inches it takes to fully extend the slide
            double FullExtendRotations = 6.75;
            //Subtracting 5 is to make sure the Motor does not overshoot or get damaged
            double targetPosition = (FullExtendRotations * MotorPPR) - 5;

            while (opModeIsActive()) {

                if (gamepad1.a) {
                    double power = returnPower(targetPosition, Math.abs(Motor.getCurrentPosition()), "Extend");

                    packet.put("power", power);
                    packet.put("position", Motor.getCurrentPosition());
                    packet.put("velocity", Motor.getVelocity());
                    packet.put("error", lastError);
                    //sleep(2000);
                    if (Motor.getCurrentPosition() < targetPosition) {
                        Motor.setPower(power);
                    } else {
                        Motor.setPower(0);
                    }

                    dashboard.sendTelemetryPacket(packet);

                } else if (gamepad1.b) {
                    double power = returnPower(0, Math.abs(Motor.getCurrentPosition()), "Retract");

                    packet.put("power", power);
                    packet.put("position", Motor.getCurrentPosition());
                    packet.put("velocity", Motor.getVelocity());
                    packet.put("error", lastError);
                    //sleep(2000);
                    if (Motor.getCurrentPosition() > 0) {
                        Motor.setPower(power);
                    } else {
                        Motor.setPower(0);
                    }

                    dashboard.sendTelemetryPacket(packet);
                } else {
                    Motor.setPower(0);
                }
            }
        }
    }

    public double returnPower(double reference, double state, String Direction) {
        if (Direction == "Extend") {
            if (state >= reference) {
                return 0;
            }
        } else if (Direction == "Retract") {
            if (state <= reference) {
                return 0;
            }
        }

        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        //timer.reset();

        return (error * Kp) + (integralSum * Ki) + (derivative * Kd);
    }
}
