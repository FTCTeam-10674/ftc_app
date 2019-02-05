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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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

@Autonomous(name="Crater LANDw/E & CLAIM", group="C")
//@Disabled
public class Ruckus_Crater_LANDwE_CLAIM extends LinearOpMode {


    /* Declare OpMode members. */
    Ruckus_HwMap howard   = new Ruckus_HwMap();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Init stuff
         */

        howard.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          howard.flDrive.getCurrentPosition(),
                          howard.frDrive.getCurrentPosition(),
                          howard.blDrive.getCurrentPosition(),
                          howard.brDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //extend LAM until on wheels touch ground (encoder)
        encoderLamb(howard.LAMB_POWER, 7.375, 10);


        //strafe to unhook latch (dead reck)
        howard.frDrive.setPower(-0.5);
        howard.flDrive.setPower(0.5);
        howard.brDrive.setPower(0.5);
        howard.blDrive.setPower(-0.5);
        sleep(400);
        howard.frDrive.setPower(0);
        howard.flDrive.setPower(0);
        howard.brDrive.setPower(0);
        howard.blDrive.setPower(0);

        sleep(500);

        //drive to de-align
        howard.frDrive.setPower(0.5);
        howard.flDrive.setPower(0.5);
        howard.brDrive.setPower(0.5);
        howard.blDrive.setPower(0.5);
        sleep(250);
        howard.frDrive.setPower(0);
        howard.flDrive.setPower(0);
        howard.brDrive.setPower(0);
        howard.blDrive.setPower(0);

        sleep(500);

        //strafe back to center
        howard.frDrive.setPower(0.5);
        howard.flDrive.setPower(-0.5);
        howard.brDrive.setPower(-0.5);
        howard.blDrive.setPower(0.5);
        sleep(400);
        howard.frDrive.setPower(0);
        howard.flDrive.setPower(0);
        howard.brDrive.setPower(0);
        howard.blDrive.setPower(0);

        sleep(250);


        howard.flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        howard.gyroInit();
        while (!isStopRequested() && !howard.gyro.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        /*
        gyroTurn(howard.TURN_SPEED, 180.0);
        gyroHold(howard.TURN_SPEED, 180.0, 0.25);
        gyroDrive(howard.DRIVE_SPEED, 4.0, 180.0);

        sleep(500);

        //activate TensorFlow
        if (howard.tfod != null) {
            howard.tfod.activate();
        }

        sleep(1000);
        runtime.reset();
        int goldPos = identify(1);

        //drive to gold mineral based on value of goldPos (facing backwards)
        if (goldPos == 0){

            gyroTurn(howard.TURN_SPEED, 180+26.5);
            gyroHold(howard.TURN_SPEED, 180+26.5, 0.25);
            gyroDrive(howard.DRIVE_SPEED, -30.0, 180+26.5);
            gyroDrive(howard.DRIVE_SPEED, 30.0, 180+26.5);

        } else if (goldPos == 1){

            gyroDrive(howard.DRIVE_SPEED, -30, 180.0); //guess
            gyroDrive(howard.DRIVE_SPEED, 30, 180.0);

        } else {

            gyroTurn(howard.TURN_SPEED, 180-26.5);
            gyroHold(howard.TURN_SPEED, 180-26.5, 0.25);
            gyroDrive(howard.DRIVE_SPEED, -30.0, 180-26.5);
            gyroDrive(howard.DRIVE_SPEED, 30.0, 180-26.5);

        }
        */

        //turn front to face depot
        gyroTurn(howard.TURN_SPEED, 80.0);
        //gyroHold();
        gyroDrive(howard.TURN_SPEED, 60, 80.0);
        gyroTurn(howard.TURN_SPEED, 135.0);
        //gyroHold();
        gyroDrive(howard.DRIVE_SPEED, 12.0, 135.0);

        //dump marker in depot
        howard.lWrist.setPosition(howard.WRIST_OUT + 0.5);
        howard.rWrist.setPosition(-howard.WRIST_OUT + 0.5);

        //back up
        gyroDrive(howard.DRIVE_SPEED, -6.0, 135.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    //METHODS

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
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = howard.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "cam");

        //  Instantiate the Vuforia engine
        howard.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        howard.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, howard.vuforia);
        howard.tfod.loadModelFromAsset(howard.TFOD_MODEL_ASSET, howard.LABEL_GOLD_MINERAL, howard.LABEL_SILVER_MINERAL);
    }

    private int identify(int timeoutS){
        int g = 1; //0 = Left; 1 = Center; 2 = Right
        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            if (howard.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.

                List<Recognition> updatedRecognitions = howard.tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(howard.LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                g = 0;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                g = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                g = 1;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        return g;
    }
    public void encoderLamb(double speed,
                             double lambInches,
                             double timeoutS) {
        int newLambTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLambTarget = howard.lamb.getCurrentPosition() + (int)(lambInches * howard.LAMB_COUNTS_PER_INCH);
            howard.lamb.setTargetPosition(newLambTarget);

            // Turn On RUN_TO_POSITION
            howard.lamb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            howard.lamb.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (howard.lamb.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newLambTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        howard.lamb.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            howard.lamb.setPower(0);

            // Turn off RUN_TO_POSITION
            howard.lamb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
