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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Working TeleOp", group="Meets")
//@Disabled
public class Working_TeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private DcMotor liftMotor;
    private Servo leftGrabber;
    private Servo rightGrabber;
    //boolean grabberOpen = true;

    Servo elbowL;
    Servo wristL;
    Servo elbowR;
    Servo wristR;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        leftGrabber = hardwareMap.get(Servo.class, "left_grabber");
        rightGrabber = hardwareMap.get(Servo.class, "right_grabber");

        elbowL = hardwareMap.get(Servo.class, "elbowL");
        wristL = hardwareMap.get(Servo.class, "wristL");
        elbowR = hardwareMap.get(Servo.class, "elbowR");
        wristR = hardwareMap.get(Servo.class, "wristR");

        /*
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d", liftMotor.getCurrentPosition());
        telemetry.update();
        */

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        double leftGrabberPosition = 1.0;
        double rightGrabberPosition = 0.0;

        leftGrabber.setPosition(leftGrabberPosition);
        rightGrabber.setPosition(rightGrabberPosition);

        elbowL.setPosition(0.0);
        wristL.setPosition(0.6);
        elbowR.setPosition(0.0);
        wristR.setPosition(0.2);




        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double rotate  = gamepad1.left_stick_x;
            double strafe = -gamepad1.right_stick_x;
            double lift = gamepad2.left_stick_y;


            if (gamepad2.right_bumper){
                leftGrabberPosition = 0.7;
                rightGrabberPosition = 0.3;
                //grabberOpen = true;
            }

            else if (gamepad2.left_bumper){
                leftGrabberPosition = 0.5;
                rightGrabberPosition = 0.5;
            }

            else if (gamepad2.left_trigger > 0.1){
                leftGrabberPosition = 0.6;
                rightGrabberPosition = 0.4;
            }

            else if (gamepad2.right_trigger > 0.1) {
                leftGrabberPosition = 1.0;
                rightGrabberPosition = 0.0;
            }


            frontLeftPower   = Range.clip(drive - rotate + strafe, -1, 1);
            frontRightPower  = Range.clip(drive + rotate + strafe, -1, 1);
            backLeftPower    = Range.clip(drive - rotate - strafe, -1, 1);
            backRightPower   = Range.clip(drive + rotate - strafe, -1, 1);


            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            liftMotor.setPower(lift);
            leftGrabber.setPosition(leftGrabberPosition);
            rightGrabber.setPosition(rightGrabberPosition);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front_left (%.2f), front_right (%.2f), back_left (%.2f), " +
                    "back_right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Servos", "left_grabber_arm (%.2f), right_grabber_arm (%.2f)", leftGrabberPosition, rightGrabberPosition);
            telemetry.addData("Lift", "lift_motor (%.2f)", lift);
            //telemetry.addData("Lift encoder", "lift encoder (%.7d)", liftMotor.getCurrentPosition());
            //telemetry.addData("Grabber state", "grabberOpen (%b)", grabberOpen);
            telemetry.update();
        }
    }
}