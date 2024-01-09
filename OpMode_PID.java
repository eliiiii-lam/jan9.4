package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class OpMode_PID extends LinearOpMode {
    DcMotorEx fL;
    DcMotorEx fR;
    DcMotorEx bL;
    DcMotorEx bR;

    DcMotorEx elbow;
    DcMotorEx elbow2;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77952;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static double p = 0.02, i = 0.15, d = 0.000;

    public static double f = 0.1;



    private final double ticks_in_degrees = 5281.1 / 180.0;

    private PIDController controller;


    ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        fL = hardwareMap.get(DcMotorEx.class, "fL");
        fR = hardwareMap.get(DcMotorEx.class, "fR");
        bL = hardwareMap.get(DcMotorEx.class, "bR");
        bR = hardwareMap.get(DcMotorEx.class, "bR");

        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow2 = hardwareMap.get(DcMotorEx.class, "elbow2");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        elbow2.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        int [] targets = {1200,0,1200};

        int delaybetweenposinMS = 3000;

        for (int targetValue : targets) {
            PIDControl(elbow, elbow2, targetValue);
            sleep(delaybetweenposinMS);

        }






    }


    public void PIDControl(DcMotorEx elbow, DcMotorEx elbow2, int targetValue) {
        double ff = Math.cos(Math.toRadians(targetValue / ticks_in_degrees)) * f;

        double minPower = 0.1;

        int timeout = 0; //Counter to avoid infinite loops

        while (opModeIsActive() && timeout < 1000) {
            controller.setPID(p, i, d);
            int armPos1 = elbow.getCurrentPosition();
            int armPos2 = elbow2.getCurrentPosition();
            double pid1 = controller.calculate(armPos1, targetValue);
            double pid2 = controller.calculate(armPos2, targetValue);

            int error1 = targetValue - armPos1;
            int error2 = targetValue - armPos2;


            double power1 = pid1 + ff;

            double power2 = pid2 + ff;

            double correction1 = controller.calculate(error1,0);
            double correction2 = controller.calculate(error2, 0);

            power1 += correction1;
            power2 += correction2;

            elbow.setPower(power1);
            elbow2.setPower(power2);


            telemetry.addData("pos1", armPos1);
            telemetry.addData("pos2", armPos2);
            telemetry.addData("target", targetValue);

            telemetry.update();


            if (Math.abs(armPos1 - targetValue) < 10 && Math.abs(armPos2 - targetValue) < 10) {
                elbow.setPower(Math.signum(pid1) * Math.max(minPower, Math.abs(pid1)));
                elbow2.setPower(Math.signum(pid2) * Math.max(minPower, Math.abs(pid2)));


                if (Math.abs(pid1) < 0.1 && Math.abs(pid2) < 0.1) {
                    break; //if both PID outputs are very low then break the loop
                }
            } else {
            elbow.setPower(power1);
            elbow2.setPower(power2);
            }
            timeout++;



        }
        elbow.setPower(0);
        elbow2.setPower(0);

    }
}





//////////////////////////////////////////////////////////////////////////////




