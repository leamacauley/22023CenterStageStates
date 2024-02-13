package org.firstinspires.ftc.teamcode.drive.sos;

/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Modified by michaudc 2017
Additional modifications by michaudc 2021
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Commenting style:
 * plain text: explainations (lmao I can't spell)
 * CAPS: functions
 */

/**
 * CONTROLS
 *  Gamepad1:
 *  a) movement     usual
 *  b) shooting     x botton
 *   c) climbing     a button (lift up LA), b button (lift robot)
 *  Gamepad 2:
 * a) slider        (triggers, a button = 0, b button = up)
 * b) intake        (bumpers)
 * c) v4b           (control with joystick, x button, y button)
 */

//@Config
@TeleOp

public class StatesDriveCode extends OpMode {

    /* Declare OpMode members. */

    MaristBaseRobot2022_Quad robot = new MaristBaseRobot2022_Quad();


    double clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    int armPosLeft = 0;           // starts armPos (of slider 1) at 0
    int armPosRight = 0;          // for slider 2

    int climbPos = 0;           // for climber

    private double SPEED_CONTROL = 0.8;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Go SOS!", "Robot Ready");

        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armPosLeft = robot.leftArm.getCurrentPosition();
        armPosRight = robot.rightArm.getCurrentPosition();
        climbPos = robot.bottomArm.getCurrentPosition();

        robot.topArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //  GAMEPAD 1 CONTROLS ---------------------------------------------------------------------

        // Movement
        double leftX;
        double leftY;
        double rightX;

        leftX = gamepad1.left_stick_x;
        leftY = gamepad1.left_stick_y ;
        rightX = gamepad1.right_stick_x ;

        double leftRearPower = (leftY + leftX - rightX);
        double leftFrontPower = (leftY - leftX - rightX);
        double rightRearPower = (leftY - leftX + rightX);
        double rightFrontPower = (leftY + leftX + rightX);

        robot.leftFront.setPower(leftFrontPower * SPEED_CONTROL);
        robot.leftRear.setPower(leftRearPower * SPEED_CONTROL);
        robot.rightFront.setPower(rightFrontPower * SPEED_CONTROL);
        robot.rightRear.setPower(rightRearPower * SPEED_CONTROL);

        if(gamepad1.left_bumper) {
            SPEED_CONTROL = 0.3;
        }
        if(gamepad1.right_bumper) {
            SPEED_CONTROL = 0.8;
        }

        // SHOOT PLANE
        if (gamepad1.b){
            robot.shoot();
        }

        // CLIMB

        // ARM STUFFS
        double climbPower = gamepad1.right_trigger - gamepad1.left_trigger;

        // UP
        // limit arm power
        if (climbPower > 0.2) {
            climbPos += climbPower* 50;  //8
        }

        // DOWN
        if (climbPower < -0.2) {
            climbPos += climbPower * 50;   //4
        }

        /**
         if(climbPos > 12100) {
         climbPos = 12100;
         }
         **/


        robot.bottomArm.setPower(0.9);
        robot.bottomArm.setTargetPosition(climbPos);
        robot.bottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(gamepad1.y) {
            robot.angleClimb();
        }
        if(gamepad1.a) {
            robot.angleBack();
        }

        int pos = robot.topArm.getCurrentPosition();

        if(gamepad1.dpad_down) {
            robot.armRunTo(pos-50, 0.9);
        }
        if(gamepad1.dpad_up) {
            robot.armRunTo(pos+50,0.9);
        }

        // GAMEPAD 2 CONTROLS ----------------------------------------------------------------------


        // ARM STUFFS
        double armMotorPower = gamepad2.right_trigger - gamepad2.left_trigger;

        if (armMotorPower > 0.2) {
            armPosLeft += armMotorPower* 8;  //8
            armPosRight += armMotorPower* 8;  //8
        }

        // DOWN
        if (armMotorPower < -0.2) {
            armPosLeft += armMotorPower * 6;   //4
            armPosRight += armMotorPower * 6;
        }

        if(gamepad2.a) {
            robot.closeClaw();
            armPosLeft = 5;
            armPosRight = 5;
        }

        // Set power and target positions based on user input

        robot.leftArm.setTargetPosition(armPosLeft);
        robot.rightArm.setTargetPosition(armPosRight);

        robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftArm.setPower(0.6);
        robot.rightArm.setPower(0.6);

        // OPEN
        if (gamepad2.right_bumper) {
            robot.openClaw();
        }

        // CLOSE
        if (gamepad2.left_bumper) {
            robot.closeClaw();
        }

        if(gamepad2.dpad_up) {  // up
            robot.rightOpen();
        }
        if(gamepad2.dpad_down) {    //down
            robot.rightClose();
        }
        if(gamepad2.dpad_left) {    // left
            robot.leftOpen();
        }
        if(gamepad2.dpad_right) {   // right
            robot.leftClose();
        }



        // Arm Presets - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        // FULLY DOWN
        if(gamepad2.a) {
            armPosLeft = 0;
            armPosRight = 0;
        }

        if(gamepad2.b) {
            armPosLeft = 1500;
            armPosRight = 1500;
        }


        // SAFETY CODE BOTTOM
        if(armPosLeft <= 0) {
            armPosLeft = 0;
        }
        if(armPosRight <= 0) {
            armPosRight = 0;
        }


        // SAFETY CODE TOP
        if(armPosLeft >= 2400) {
            armPosLeft = 2400;
        }
        if(armPosRight >= 2400) {
            armPosRight = 2400;
        }


        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        // VIRTUAL FOUR BAR

        // DOWNWARD DOG
        if(gamepad2.x) {
            robot.armRunTo(0, 0.5);     // brings four bar down to the ground
        }
        // UPWARD DOG
        if(gamepad2.y) {
            robot.armRunTo(560,0.5);     // puts four bar in the scoring position
        }

        // ----------------------------------------------------------------------------------------

        // Send telemetry message to signify robot running;

        telemetry.addData("ClimberPos: %.2f", climbPos);
        telemetry.addData("SliderPos: %.2f", armPosLeft);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
