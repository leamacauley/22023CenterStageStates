package org.firstinspires.ftc.teamcode.drive.sos.Auto;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.drive.sos.MaristBaseRobot2022_Quad;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
@Config

public class Auto_Blue_Backstage extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    // Initialize the Arms and Servo System
    MaristBaseRobot2022_Quad robot = new MaristBaseRobot2022_Quad();
    private ElapsedTime runtime = new ElapsedTime();
    private int zone = 1; // Default if Team Prop not found

    private int watchTime = 5; // Watch for 5 seconds

    /* Declare Camera Fields */
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";

    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    public void runOpMode() {

        //0.5 (open), 0.85 (close)

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < watchTime)) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        telemetry.addData("Zone", zone);
        telemetry.update();

        // Run Autonomous Based on Team Prop Position
        if (zone == 1) {
            zoneOne();
        }
        else if (zone == 2) {
            zoneTwo();
        }
        else {
            zoneThree();
        }
        delay(10); // Optional to hold OpMode open to display Zone Selection.


        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();



        waitForStart();

        if(isStopRequested()) return;

        runtime.reset();


    }
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            //telemetry.update();
        }
    }
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_ASSET)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(new String[]{"Team Prop"})
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (x < 200) {
                zone = 2;
            }
            else if (x > 0) {
                zone = 3;
            }
            else {
                zone = 1;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    private void zoneThree() {
        Pose2d startPose = new Pose2d(10.84, 61.16, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toSpikeTape = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(11.25, 31.70), Math.toRadians(180.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toSpikeTape.end())
                .splineTo(new Vector2d(37.02, 32.11), Math.toRadians(0.00))
                .splineTo(new Vector2d(50.73, 42.14), Math.toRadians(0.00))
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(toBoard.end())
                .lineTo(new Vector2d(48.68, 62.18))
                .splineTo(new Vector2d(62.67, 62.06), Math.toRadians(0.00))
                .build();

        //  following
        drive.followTrajectorySequence(toSpikeTape);
        delay(1000);
        robot.leftOpen();
        delay(1000);
        robot.armRunTo(1600,0.8);
        delay(1000);
        drive.followTrajectorySequence(toBoard);
        delay(1000);
        robot.rightOpen();
        drive.followTrajectorySequence(park);
        robot.armRunTo(0,0.8);

    }
    public void zoneTwo() {
        Pose2d startPose = new Pose2d(10.84, 61.16, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        TrajectorySequence toSpikeTape = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(11.65, 33.99), Math.toRadians(270.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toSpikeTape.end())
                .splineTo(new Vector2d(10.43, 52.98), Math.toRadians(90))
                .splineTo(new Vector2d(51.14, 35.80), Math.toRadians(0.00))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBoard.end())
                .lineTo(new Vector2d(50.30, 59.83))
                .build();

        //  following
        drive.followTrajectorySequence(toSpikeTape);
        delay(1000);
        robot.leftOpen();
        delay(1000);
        robot.armRunTo(1600,0.8);
        delay(1000);
        drive.followTrajectorySequence(toBoard);
        delay(1000);
        robot.rightOpen();
        drive.followTrajectorySequence(park);
        robot.armRunTo(0,0.8);

    }

    public void zoneOne() {
        Pose2d startPose = new Pose2d(10.84, 61.16, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence toSpikeTape = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(14.40, 30.02), Math.toRadians(0.00))
                .build();

        TrajectorySequence toBoard = drive.trajectorySequenceBuilder(toSpikeTape.end())
                .lineTo(new Vector2d(11.15, 52.33))
                .splineTo(new Vector2d(50.70, 30.22), Math.toRadians(109.65))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(toBoard.end())
                .lineTo(new Vector2d(47.86, 60.03))
                .splineTo(new Vector2d(61.66, 59.83), Math.toRadians(0.00))
                .build();

        //  following
        drive.followTrajectorySequence(toSpikeTape);
        delay(1000);
        robot.leftOpen();
        delay(1000);
        robot.armRunTo(1600,0.8);
        delay(1000);
        drive.followTrajectorySequence(toBoard);
        delay(1000);
        robot.rightOpen();
        drive.followTrajectorySequence(park);
        robot.armRunTo(0,0.8);

    }



}