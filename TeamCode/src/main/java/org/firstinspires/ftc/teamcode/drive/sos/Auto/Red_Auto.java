package org.firstinspires.ftc.teamcode.drive.sos.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.sos.MaristBaseRobot2022_Quad;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Config

public class Red_Auto extends LinearOpMode {
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize the Arms and Servo System
        MaristBaseRobot2022_Quad robot = new MaristBaseRobot2022_Quad();
        robot.init(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        //0.5 (open), 0.85 (close)

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        Trajectory first = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-3, 4))
                .build();
        Trajectory second = drive.trajectoryBuilder(first.end())
                .lineToSplineHeading((new Pose2d(-1, 5, Math.toRadians(90))))
                .build();



        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(first);
        //drive.followTrajectory(second);

        runtime.reset();


    }


}