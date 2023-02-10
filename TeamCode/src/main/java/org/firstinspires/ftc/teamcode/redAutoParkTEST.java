/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.openCV.SignalColor;
import org.firstinspires.ftc.teamcode.openCV.camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.Math;
import java.sql.SQLOutput;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="auto TEST")

public class redAutoParkTEST extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotor = null;
    private OpenCvWebcam webcam;
    private camera pipeline;
    private SignalColor color;
    private SignalColor ifYellow;

    Servo servo_claw;
    Servo servo_lift_r;
    Servo servo_lift_l;
    double servo_claw_pos;
    double servo_lift_l_pos;
    double servo_lift_r_pos;
    //in
    static final double SERVO_CLAW_INIT = .2;
    //out
    static final double SERVO_CLAW_GRAB = .47;

    static final double SERVO_LIFT_R_FRONT = .15;
    static final double SERVO_LIFT_R_FMID = .25;
    static final double SERVO_LIFT_R_BACK = .95;
    static final double SERVO_LIFT_R_BMID = .55;

    static final double SERVO_LIFT_L_FRONT = .93;
    static final double SERVO_LIFT_L_FMID = .83;
    static final double SERVO_LIFT_L_BACK = .2;
    static final double SERVO_LIFT_L_BMID = .6;


    private Thread telemetryH = new Thread() {
        @Override
        public void run() {
            while (opModeInInit() || opModeIsActive()) {
                telemetry.addData("rf", rightFront.getCurrentPosition());
                telemetry.addData("color", pipeline.getBiggestArea());
                telemetry.update();
            }
        }
    };
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        pipeline = new camera(telemetry);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                //if the camera cannot open
                telemetry.addData("camera status: ", "error");
            }

        });

        double startTime = System.currentTimeMillis();
        while (pipeline.getBiggestArea() == SignalColor.IDK && System.currentTimeMillis() - startTime < 3000) {
            telemetry.addData("camera status: ", "not ready");
            telemetry.update();
        }

        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        servo_claw = hardwareMap.servo.get("servo_claw");
        servo_lift_l = hardwareMap.servo.get("servo_lift_l");
        servo_lift_r = hardwareMap.servo.get("servo_lift_r");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Send telemetry message to indicate successful Encoder reset
        telemetryH.start();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        servo_claw_pos = SERVO_CLAW_GRAB;
        servo_claw.setPosition(servo_claw_pos);
        servo_lift_l_pos = SERVO_LIFT_L_FRONT;
        servo_lift_r_pos = SERVO_LIFT_R_FRONT;
        servo_lift_l.setPosition(servo_lift_l_pos);
        servo_lift_r.setPosition(servo_lift_r_pos);
        color = pipeline.getBiggestArea();




        //add in camera code

        forwardDrive(.2, 1200);
        strafeRight(.4, 1725);
        sleep(1000);

        ifYellow = pipeline.getIfYellow();

        //if (ifYellow == SignalColor.YELLOW){
            strafeLeft(.2, 200);
            forwardDrive(-.2, 250);
            servo_lift_l_pos = SERVO_LIFT_L_BMID;
            servo_lift_r_pos = SERVO_LIFT_R_BMID;
            servo_lift_l.setPosition(servo_lift_l_pos);
            servo_lift_r.setPosition(servo_lift_r_pos);

            liftMotor.setTargetPosition(691);
            liftMotor.setDirection(DcMotor.Direction.REVERSE);
            liftMotor.setPower(.8);

            sleep(2000);
            forwardDrive(.2, 200);

            servo_claw_pos = SERVO_CLAW_INIT;
            servo_claw.setPosition(servo_claw_pos);
            sleep(1000);
            forwardDrive(-.2, 200);
            //reset
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setPower(.15);
            liftMotor.setPower(0);
            sleep(300);
            servo_lift_l_pos = SERVO_LIFT_L_FRONT;
            servo_lift_r_pos = SERVO_LIFT_R_FRONT;
            servo_lift_l.setPosition(servo_lift_l_pos);
            servo_lift_r.setPosition(servo_lift_r_pos);
            forwardDrive(.2, 200);


       // }

        if (color == SignalColor.ORANGE) {
            //parking location one -- straight left
          //  forwardDrive(.2, 1150);
            strafeLeft(.4, 2875); //go all the way left
//            color = SignalColor.IDK;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);

        } else if (color == SignalColor.GREEN) {
            //parking location two -- straight
          //  forwardDrive(.2, 1150);
            strafeLeft(.4, 1725); //go back to beginning
//            color = SignalColor.IDK;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        } else if (color == SignalColor.PURPLE) {
            //three straight right
//            forwardDrive(.2, 1150);
//            strafeRight(.4, 1150);
            strafeLeft(.4, 375); //random value
//            color = SignalColor.IDK;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
        } else if (color == SignalColor.IDK) {
            color = SignalColor.GREEN;
        }



//         Step through each leg of the path,
//         Note: Reverse movement is obtained by setting a negative distance (not speed)
//         encoderDrive(DRIVE_SPEED,  0,  0, 20, -24, 5);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED, 0, 0, 20, 0, 4.0);
//        encoderDrive(DRIVE_SPEED, 20, 0, 20, 0, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftFInches, double leftBInches, double rightFInches, double rightBInches,
                             double timeoutS) {
        int newLeftFTarget;
        int newLeftBTarget;
        int newRightFTarget;
        int newRightBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
//            newLeftFTarget = leftFront.getCurrentPosition() + (int)(leftFInches * COUNTS_PER_INCH);
//            newLeftBTarget = leftBack.getCurrentPosition() + (int)(leftBInches * COUNTS_PER_INCH);
//            newRightFTarget = rightFront.getCurrentPosition() + (int)(rightFInches * COUNTS_PER_INCH);
//            newRightBTarget = rightBack.getCurrentPosition() + (int)(rightBInches * COUNTS_PER_INCH);
//            leftFront.setTargetPosition(newLeftFTarget);
//            leftBack.setTargetPosition(newLeftBTarget);
//            rightFront.setTargetPosition(newRightFTarget);
//            rightBack.setTargetPosition(newRightBTarget);

            // Turn On RUN_TO_POSITION

//            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            double maxSpeed = 0.85;

            leftFront.setPower(Math.abs(speed) * maxSpeed);
            leftBack.setPower(Math.abs(speed) * maxSpeed);
            rightFront.setPower(Math.abs(speed) * maxSpeed);
            rightBack.setPower(Math.abs(speed) * maxSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                //telemetry.addData("Running to",  " %7d :%7d", newLeftFTarget, newLeftBTarget, newRightFTarget, newRightBTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftBack.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                if(ifYellow == SignalColor.YELLOW ){
                    telemetry.addData("see yellow", "\" at %7d :%7d\",", ifYellow);
                }
                telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // sleep(250);   // optional pause after each move.
        }
    }

    public void forwardDrive(double power, int position) {

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        System.out.println("in forward drive");

        leftFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
        rightFront.setPower(-power);


        //wait until reaches position
        while (Math.abs(rightFront.getCurrentPosition()) < position && opModeIsActive()) {
            telemetry.addData("position: ", rightFront.getCurrentPosition());
            telemetry.update();

        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeLeft(double power, int position) {

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        System.out.println("in strafe l");

        leftFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(-power);

        //wait until finishes turning
        while (Math.abs(rightFront.getCurrentPosition()) < position && opModeIsActive()) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void strafeRight(double power, int position) {

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        rightFront.setPower(power);

        //wait until finishes turning
        while (Math.abs(rightFront.getCurrentPosition()) < position && opModeIsActive()) {
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }
}