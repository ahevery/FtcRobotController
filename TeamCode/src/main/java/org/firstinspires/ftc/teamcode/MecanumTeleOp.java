package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import kotlin._Assertions;

@TeleOp
public class MecanumTeleOp extends OpMode {

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor slide_left = null;
    private DcMotor slide_right = null;

    private int slide_position = 0;
    private final int slide_max_position = 3800;
    private final int slide_min_position = 0;
    private final int slide_step = 10;


    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        front_right  = hardwareMap.get(DcMotor.class, "frontRightMotor");
        back_left	 = hardwareMap.get(DcMotor.class, "backLeftMotor");
        back_right   = hardwareMap.get(DcMotor.class, "backRightMotor");
        slide_left   = hardwareMap.get(DcMotor.class, "slideLeft");
        slide_right  = hardwareMap.get(DcMotor.class, "slideRight");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);

        slide_left.setTargetPosition(slide_position);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide_right.setTargetPosition(slide_position);
        slide_right.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        // slides
        if (gamepad1.dpad_up) {
            slide_position= Math.min(slide_max_position, slide_position + slide_step);
        } else if (gamepad1.dpad_down) {
            slide_position-=slide_step;
            slide_position= Math.max(slide_min_position, slide_position - slide_step);
        }

        if(slide_right.isBusy()) {
            slide_right.setPower(1.0);
        } else {
            slide_right.setPower(0);
        }

        slide_right.setTargetPosition(slide_position);
        slide_left.setTargetPosition(slide_position);

        if(slide_left.isBusy()) {
            slide_left.setPower(1.0);
        } else {
            slide_left.setPower(0);
        }

        // DRIVE CODE
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        telemetry.addData("Left Stick Y", y);
        telemetry.addData("Left Stick X", x);
        telemetry.addData("Right Stick X", rx);
        telemetry.addData("Slide target position", slide_position);
        telemetry.addData("Slide right position", slide_right.getCurrentPosition());
        telemetry.addData("Slide left position", slide_left.getCurrentPosition());
        telemetry.update();
    }
}