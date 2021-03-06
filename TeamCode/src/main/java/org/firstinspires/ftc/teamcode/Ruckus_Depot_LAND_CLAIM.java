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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

@Autonomous(name="Depot LAND & CLAIM", group="D")
@Disabled
public class Ruckus_Depot_LAND_CLAIM extends LinearOpMode {

    /* Declare OpMode members. */
    Ruckus_HwMap howard   = new Ruckus_HwMap();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        howard.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
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
        /*howard.lWinch.setPower(howard.WINCH_POWER);
        howard.rWinch.setPower(howard.WINCH_POWER);
        sleep(howard.TIME_TO_EXTEND);
        howard.lWinch.setPower(0);
        howard.rWinch.setPower(0);*/
        sleep(500);
        //howard.latch.setPosition(howard.LATCH_OPEN);
        //dead-reckoning strafing to unhook
        howard.brDrive.setPower(0.3);
        howard.flDrive.setPower(0.3);
        howard.blDrive.setPower(-0.3);
        howard.frDrive.setPower(-0.3);
        sleep(1000);
        howard.brDrive.setPower(0);
        howard.flDrive.setPower(0);
        howard.blDrive.setPower(0);
        howard.frDrive.setPower(0);
        sleep(100);
        //retract arm
        /*howard.lWinch.setPower(-howard.WINCH_POWER);
        howard.rWinch.setPower(-howard.WINCH_POWER);
        sleep(howard.TIME_TO_RETRACT);
        howard.lWinch.setPower(0);
        howard.rWinch.setPower(0);*/

        //VUFORIA ORIENTATION

        //init gyro
        howard.gyroInit();
        while (!isStopRequested() && !howard.gyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        //leave the landing zone and drive towards wall
        gyroTurn(howard.TURN_SPEED, 45.0);
        gyroHold(howard.TURN_SPEED, 45.0, 0.5);
        gyroDrive(howard.DRIVE_SPEED, -45.0, 45.0);  //the first "-45.0" is inches not degrees

        // >> Starting in Depot pos: turn right
        //    Starting in Crater pos: turn left
        gyroTurn(howard.TURN_SPEED, -45.0);
        gyroHold(howard.TURN_SPEED, -45.0, 0.5);
        gyroDrive(howard.DRIVE_SPEED, -25.0, -45.0);

        //dump marker in depot

        //reverse to crater
        //gyroDrive(howard.DRIVE_SPEED, -70.0, 45.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
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

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newFrontLeftTarget;
        int     newFrontRightTarget;
        int     newBackLeftTarget;
        int     newBackRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            moveCounts = (int)(distance * howard.COUNTS_PER_INCH);
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = howard.flDrive.getCurrentPosition() + moveCounts;
            newFrontRightTarget = howard.frDrive.getCurrentPosition() + moveCounts;
            newBackLeftTarget = howard.blDrive.getCurrentPosition() + moveCounts;
            newBackRightTarget = howard.brDrive.getCurrentPosition() + moveCounts;
            howard.flDrive.setTargetPosition(newFrontLeftTarget);
            howard.frDrive.setTargetPosition(newFrontRightTarget);
            howard.blDrive.setTargetPosition(newBackLeftTarget);
            howard.brDrive.setTargetPosition(newBackRightTarget);

            howard.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            howard.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            howard.flDrive.setPower(speed);
            howard.frDrive.setPower(speed);
            howard.blDrive.setPower(speed);
            howard.brDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (howard.flDrive.isBusy() && howard.frDrive.isBusy() && howard.blDrive.isBusy() && howard.brDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, howard.P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                howard.flDrive.setPower(leftSpeed);
                howard.frDrive.setPower(rightSpeed);
                howard.blDrive.setPower(leftSpeed);
                howard.brDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d:%7d:%7d",      newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",      howard.flDrive.getCurrentPosition(),
                        howard.frDrive.getCurrentPosition(),
                        howard.blDrive.getCurrentPosition(),
                        howard.brDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
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
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, howard.P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, howard.P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        howard.flDrive.setPower(0);
        howard.frDrive.setPower(0);
        howard.blDrive.setPower(0);
        howard.brDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= howard.HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }


        // Send desired speeds to motors.
        howard.flDrive.setPower(leftSpeed);
        howard.frDrive.setPower(rightSpeed);
        howard.blDrive.setPower(leftSpeed);
        howard.brDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - howard.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
