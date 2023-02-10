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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="iterative TEST")
//@Disabled
public class iterativeTEST extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //    port 1
    private DcMotor leftFront = null;
    //    port 3
    private DcMotor leftBack = null;
    //    port 0
    private DcMotor rightFront = null;
    //    port 2
    private DcMotor rightBack = null;

    private DcMotor liftMotor = null;

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


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");


        servo_claw = hardwareMap.servo.get("servo_claw");
        servo_claw_pos = SERVO_CLAW_INIT;
        servo_lift_r = hardwareMap.servo.get("servo_lift_r");
        servo_lift_r_pos = SERVO_LIFT_R_FRONT;
        servo_lift_l = hardwareMap.servo.get("servo_lift_l");
        servo_lift_l_pos = SERVO_LIFT_L_FRONT;


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        servo_claw.setPosition(servo_claw_pos);
        servo_lift_r.setPosition(servo_lift_r_pos);
        servo_lift_l.setPosition(servo_lift_l_pos);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        double liftPower;
//        double

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        double turn = -gamepad1.right_stick_y;
//        double drive = gamepad1.right_stick_x;
//
//        leftFrontPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        leftBackPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        rightFrontPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//        rightBackPower   = Range.clip(drive - turn, -1.0, 1.0) ;
        final double JOYSTICK_SEN = .2;
        // if mathabs < joystick -> (?) 0 else (:) set to leftstick

        double lx = Math.abs(gamepad1.left_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.left_stick_x;
        //lx is turning
        double rx = Math.abs(gamepad1.right_stick_x)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_x;
        //rx is strafing
        double ry = Math.abs(gamepad1.right_stick_y)< JOYSTICK_SEN ? 0 : gamepad1.right_stick_y;
        //front and back

        double rt = Math.abs(gamepad2.right_trigger)< JOYSTICK_SEN ? 0 : gamepad2.right_trigger;
        double lt = Math.abs(gamepad2.left_trigger)< JOYSTICK_SEN ? 0 : gamepad2.left_trigger;

        if (gamepad2.x) {
            servo_claw_pos = SERVO_CLAW_INIT;
        }
        if (gamepad2.b) {
            servo_claw_pos = SERVO_CLAW_GRAB;
        }

        double rScaled = Range.clip(rt, 0, 1.0);
        double lScaled = Range.clip(lt, 0, 1.0);
        //lift up
        if (gamepad2.right_bumper) {
            servo_lift_l_pos = SERVO_LIFT_L_FRONT;
            servo_lift_r_pos = SERVO_LIFT_R_FRONT;
            servo_lift_r.setPosition(servo_lift_r_pos);
            servo_lift_l.setPosition(servo_lift_l_pos);
        } else if (gamepad2.left_bumper) {
            servo_lift_l_pos = SERVO_LIFT_L_BACK;
            servo_lift_r_pos = SERVO_LIFT_R_BACK;
            servo_lift_r.setPosition(servo_lift_r_pos);
            servo_lift_l.setPosition(servo_lift_l_pos);
        }
        if (rScaled > 0) {
            servo_lift_l_pos = SERVO_LIFT_L_FMID;
            servo_lift_r_pos = SERVO_LIFT_R_FMID;
            servo_lift_l.setPosition(servo_lift_l_pos);
            servo_lift_r.setPosition(servo_lift_r_pos);
        }
        if (lScaled > 0) {
            servo_lift_l_pos = SERVO_LIFT_L_BMID;
            servo_lift_r_pos = SERVO_LIFT_R_BMID;
            servo_lift_l.setPosition(servo_lift_l_pos);
            servo_lift_r.setPosition(servo_lift_r_pos);
        }



        servo_claw.setPosition(servo_claw_pos);

        if (gamepad2.y) {
            liftMotor.setTargetPosition(691);
            liftMotor.setDirection(DcMotor.Direction.REVERSE);
            liftMotor.setPower(1);
        } else if (gamepad2.a) {
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setPower(.15);
        } else {
            liftMotor.setPower(0);
        }


        leftBackPower    = Range.clip(-lx - rx - ry, -1.0, 1.0) ;
        leftFrontPower    = Range.clip(-lx + rx - ry, -1.0, 1.0) ;
        rightFrontPower   = Range.clip(-lx + rx + ry, -1.0, 1.0) ;
        rightBackPower   = Range.clip(-lx - rx + ry, -1.0, 1.0) ;


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        double maxSpeed =0.75;
        leftFront.setPower(leftFrontPower*maxSpeed);
        leftBack.setPower(leftBackPower*maxSpeed);
        rightFront.setPower(rightFrontPower*maxSpeed);
        rightBack.setPower(rightBackPower*maxSpeed);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("lf", leftFront.getCurrentPosition());
        telemetry.addData("rf", rightFront.getCurrentPosition());
        telemetry.addData("lb", leftBack.getCurrentPosition());
        telemetry.addData("rb", rightBack.getCurrentPosition());
        telemetry.addData("LIFT", liftMotor.getCurrentPosition());


        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}