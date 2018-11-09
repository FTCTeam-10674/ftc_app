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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Howard Auto", group="Pushbot")
@Disabled
public class BDepot_Auto1 extends LinearOpMode {

    /* Declare OpMode members. */
    HwMap_Ruckus howard   = new HwMap_Ruckus();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 0.5;
    static final double     STRAFE_SPEED            = 0.8;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        howard.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        howard.flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        howard.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          howard.flDrive.getCurrentPosition(),
                          howard.frDrive.getCurrentPosition(),
                          howard.blDrive.getCurrentPosition(),
                          howard.brDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //lower robot from hanging position
        /*howard.armWinch.setPower(howard.WINCH_POWER);
        sleep(howard.TIME_TO_EXTEND);
        howard.armWinch.setPower(0);
        howard.latch.setPosition(howard.LATCH_OPEN);
        howard.armWinch.setPower(-howard.WINCH_POWER);
        sleep(howard.TIME_TO_EXTEND);
        howard.armWinch.setPower(0);*/

        //CALIBRATE GYRO AND/OR COMPUTER VISION CODE

        encoderDrive(DRIVE_SPEED, 30, 30, 3);
        encoderStrafe(STRAFE_SPEED, 8.48, 2);
        senseColor(5);
        //...







        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newleftTarget;
        int newrightTarget;

        howard.flDrive.setDirection(DcMotor.Direction.FORWARD);
        howard.frDrive.setDirection(DcMotor.Direction.REVERSE);
        howard.blDrive.setDirection(DcMotor.Direction.FORWARD);
        howard.brDrive.setDirection(DcMotor.Direction.REVERSE);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newleftTarget = howard.flDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newrightTarget = howard.frDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            howard.flDrive.setTargetPosition(newleftTarget);
            howard.frDrive.setTargetPosition(newrightTarget);

            newleftTarget = howard.blDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newrightTarget = howard.brDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            howard.blDrive.setTargetPosition(newleftTarget);
            howard.brDrive.setTargetPosition(newrightTarget);

            // Turn On RUN_TO_POSITION
            howard.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            howard.flDrive.setPower(Math.abs(speed));
            howard.frDrive.setPower(Math.abs(speed));
            howard.blDrive.setPower(Math.abs(speed));
            howard.brDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (howard.flDrive.isBusy() && howard.frDrive.isBusy() && howard.blDrive.isBusy() && howard.brDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newleftTarget,  newrightTarget, newleftTarget, newrightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            howard.flDrive.getCurrentPosition(),
                                            howard.frDrive.getCurrentPosition(),
                        howard.blDrive.getCurrentPosition(),
                        howard.brDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            howard.flDrive.setPower(0);
            howard.frDrive.setPower(0);
            howard.blDrive.setPower(0);
            howard.brDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            howard.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            howard.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            howard.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            howard.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderStrafe (double speed,
                               double strafeInches,
                               double timeoutS) {
        int newFRBLTarget;
        int newFLBRTarget;

        howard.flDrive.setDirection(DcMotor.Direction.FORWARD);
        howard.frDrive.setDirection(DcMotor.Direction.REVERSE);
        howard.blDrive.setDirection(DcMotor.Direction.REVERSE);
        howard.brDrive.setDirection(DcMotor.Direction.FORWARD);

        if (opModeIsActive()){

            newFRBLTarget = howard.flDrive.getCurrentPosition() + (int)(strafeInches * COUNTS_PER_INCH);
            newFLBRTarget = howard.frDrive.getCurrentPosition() + (int)(strafeInches * COUNTS_PER_INCH);
            howard.flDrive.setTargetPosition(newFRBLTarget);
            howard.frDrive.setTargetPosition(newFLBRTarget);

            newFRBLTarget = howard.blDrive.getCurrentPosition() + (int)(strafeInches * COUNTS_PER_INCH);
            newFLBRTarget = howard.brDrive.getCurrentPosition() + (int)(strafeInches * COUNTS_PER_INCH);
            howard.blDrive.setTargetPosition(newFRBLTarget);
            howard.brDrive.setTargetPosition(newFLBRTarget);

            // Turn On RUN_TO_POSITION
            howard.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            howard.flDrive.setPower(Math.abs(speed));
            howard.frDrive.setPower(Math.abs(speed));
            howard.blDrive.setPower(Math.abs(speed));
            howard.brDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (howard.flDrive.isBusy() && howard.frDrive.isBusy() &&
                            howard.blDrive.isBusy() && howard.brDrive.isBusy())){
                //LOGIC
            }
            howard.flDrive.setPower(0);
            howard.frDrive.setPower(0);
            howard.blDrive.setPower(0);
            howard.brDrive.setPower(0);

            howard.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            howard.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            howard.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            howard.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


    }

    public float senseColor(double timeoutS){

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            /*Color.RGBToHSV((int) (howard.colSensor.red() * SCALE_FACTOR),
                    (int) (howard.colSensor.green() * SCALE_FACTOR),
                    (int) (howard.colSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            //telemetry.addData("Distance (cm)",
            //        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", howard.colSensor.alpha());
            telemetry.addData("Red  ", howard.colSensor.red());
            telemetry.addData("Green", howard.colSensor.green());
            telemetry.addData("Blue ", howard.colSensor.blue());
            telemetry.addData("Hue", hsvValues[0]); */

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            telemetry.update();
        }
        return hsvValues[0];
    }
}
