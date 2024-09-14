package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Autonomous
public final class lockTo extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            waitForStart();

            while (true) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(0.01)
                                .lineToX(0)
                                .build());
            }
        }
    }
