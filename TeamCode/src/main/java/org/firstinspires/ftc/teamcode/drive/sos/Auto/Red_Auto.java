package org.firstinspires.ftc.teamcode.drive.sos.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sos.MaristBaseRobot2022_Quad;

import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="RoadRunner Red Auto", group="Auto")
@Config

public class Red_Auto extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the Arms and Servo System
        MaristBaseRobot2022_Quad robot = new MaristBaseRobot2022_Quad();
        robot.init(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        //0.5 (open), 0.85 (close)

        Pose2d startPose = new Pose2d(7, 65, Math.toRadians(180));

        drive.setPoseEstimate(startPose);
        // to spike tape, position three
        Trajectory thirdTape = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(14.11, -31.50, Math.toRadians(0.00)))
                .build();

        // GLOBAL!!!!
        Trajectory third = drive.trajectoryBuilder(thirdTape.end())
                .lineToSplineHeading(new Pose2d(10.02, -58.30, Math.toRadians(0.00)))
                .build();
        // to board
        Trajectory fourth = drive.trajectoryBuilder(third.end())
                .splineTo(new Vector2d(51.52, -36.71), Math.toRadians(0.00))
                .build();
        // park in middle terminal
        Trajectory fifth = drive.trajectoryBuilder(fourth.end())
                .lineToSplineHeading(new Pose2d(47.86, -9.33, Math.toRadians(0.00)))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        //drive.followTrajectory(first);
        drive.followTrajectory(thirdTape);

        runtime.reset();


    }


}