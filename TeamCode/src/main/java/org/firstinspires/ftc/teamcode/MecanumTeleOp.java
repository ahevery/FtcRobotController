package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MecanumTeleOp extends OpMode {

    enum Mode {
        INIT, NORMAL, LOAD, UNLOAD, HANG
    }

    /*
     * The mecanum drivetrain involves four separate motors that spin in
     * different directions and different speeds to produce the desired
     * movement at the desired speed.
     */

    private final int slide_max_position = 3800;
    private final int slide_min_position = 0;
    private final int slide_step = 10;
    private final int arm_max_position = 1250;
    private final int arm_min_position = 0;
    private final int arm_angle_sweep = 200;
    private final int arm_step = 5;
    private final double intake_max_position = 1250;
    private final double intake_min_position = 0;
    private final double intake_step = 0.02;
    // declare and initialize four DcMotors.
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private DcMotor slide_left = null;
    private DcMotor slide_right = null;
    private DcMotor arm = null;
    private Servo intake_position_servo = null;
    private CRServo intake_spin_servo = null;
    private Servo lift_left = null;
    private Servo lift_right = null;
    private int slide_position = 0;
    private int arm_position = 0;
    private double intake_position = 0.003;
    private double lift_position = 0;

    private Mode last_mode = Mode.INIT;
    private Mode mode = Mode.INIT;

    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        front_right = hardwareMap.get(DcMotor.class, "frontRightMotor");
        back_left = hardwareMap.get(DcMotor.class, "backLeftMotor");
        back_right = hardwareMap.get(DcMotor.class, "backRightMotor");
        slide_left = hardwareMap.get(DcMotor.class, "slideLeft");
        slide_right = hardwareMap.get(DcMotor.class, "slideRight");
        arm = hardwareMap.get(DcMotor.class, "armMotor");
        intake_spin_servo = hardwareMap.get(CRServo.class, "intake_spin");
        intake_position_servo = hardwareMap.get(Servo.class, "intake_position");
        lift_left = hardwareMap.get(Servo.class, "left lift");
        lift_right = hardwareMap.get(Servo.class, "right lift");


        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setTargetPosition(slide_position);
        slide_left.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide_right.setTargetPosition(slide_position);
        slide_right.setDirection(DcMotorSimple.Direction.FORWARD);
        slide_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setTargetPosition(arm_position);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_spin_servo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        last_mode = mode;
        if (last_mode == Mode.HANG) {
            if (!gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                mode = Mode.NORMAL;
            }
        } else {
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                mode= (gamepad1.right_bumper || gamepad2.right_bumper) ? Mode.HANG : Mode.UNLOAD;
            } else {
                if (gamepad1.right_bumper || gamepad2.right_bumper ) {
                    mode = Mode.LOAD;
                } else if (mode != Mode.INIT){
                    mode =  Mode.NORMAL;
                }
            }
        }

        if (last_mode != mode ) {
            double ticks_per_degree = ((double)arm_max_position) / 200.0;
            switch (mode) {
                case NORMAL:
                    break;
                case LOAD:
                    slide_position = 0;
                    arm_position = (int)(160 * ticks_per_degree);
                    break;
                case UNLOAD:
                    arm_position = (int)(45.0 * ticks_per_degree);
                    slide_position = (int)(.75 * slide_max_position);
                    break;
                case HANG:
                    arm_position = 0;
                    slide_position = (int)(.5 * slide_max_position);
                    break;
            }
        }

        // DRIVE CODE
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        if (mode == Mode.LOAD || mode == Mode.UNLOAD) {
            // go into fine adjustment
            y = y * 0.15;
            x = x * 0.15;
            rx = rx * 0.15;
        }

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


        // slides
        slide_position = (int)clip(slide_position, slide_min_position, slide_max_position, slide_step,
                gamepad1.dpad_up || gamepad2.dpad_up,
                gamepad1.dpad_down || gamepad2.dpad_down);
        slide_right.setTargetPosition(slide_position);
        slide_right.setPower(slide_right.isBusy() ? 1.0 : 0);
        slide_left.setTargetPosition(slide_position);
        slide_left.setPower(slide_left.isBusy() ? 1.0 : 0);

        arm_position = (int) clip(arm_position, arm_min_position, arm_max_position,  arm_step * (gamepad1.right_stick_y +gamepad2.right_stick_y),
                true, false);
        arm.setTargetPosition(arm_position);
        arm.setPower(arm.isBusy() ? .25 : 0);
        double arm_angle = (double)arm_angle_sweep *((double)arm_position / (double)arm_max_position);
        switch (mode) {
            case NORMAL:
                break;
            case LOAD:
                intake_position = ((arm_angle-180.0)/180.0  + .75);
                break;
            case UNLOAD:
                intake_position = .8;
                break;
            case HANG:
                intake_position = .5;
                break;
        }
        intake_position_servo.setPosition(intake_position);

        // back lift position
        lift_position=mode == Mode.HANG ? 1.0 : 0;
        lift_left.setPosition(lift_position);
        lift_right.setPosition(1 - lift_position);


        //  intake spin
        if (gamepad1.right_trigger > 0.2 || gamepad2.right_trigger > 0.2) {
            intake_spin_servo.setPower(1);
        } else if (gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2) {
            intake_spin_servo.setPower(-1);
        } else {
            intake_spin_servo.setPower(0);
        }

        telemetry.addData("Mode", mode.toString());
        telemetry.addData("Slide target position", slide_position);
        telemetry.addData("Slide right position", slide_right.getCurrentPosition());
        telemetry.addData("Slide left position", slide_left.getCurrentPosition());
        telemetry.addData("Arm target angle", arm_angle);
        telemetry.addData("Arm position", arm.getCurrentPosition());
        telemetry.addData("Lift position", lift_position);
        telemetry.addData("Intake target position", intake_position);
        telemetry.addData("Intake position", intake_position_servo.getPosition());
        telemetry.addData("right_trigger", gamepad1.right_trigger);
        telemetry.addData("left_trigger", gamepad1.left_trigger);
        telemetry.addData("spin", intake_spin_servo.getPower());
        telemetry.update();
    }

    private double clip(double current, double min, double max, double step, boolean add, boolean subtract) {
        current = current + (add ? step : 0) - (subtract ? step : 0);
        current = Math.max(min, current);
        current = Math.min(max, current);
        return current;
    }


}