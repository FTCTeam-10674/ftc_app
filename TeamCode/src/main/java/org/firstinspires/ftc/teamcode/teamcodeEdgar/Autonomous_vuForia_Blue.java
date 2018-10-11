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

package org.firstinspires.ftc.teamcode.teamcodeEdgar;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

@Autonomous(name="Auto Drive VuForia Blue", group="Worksish")
@Disabled
public class Autonomous_vuForia_Blue extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private DcMotor liftMotor;
    private Servo leftGrabber;
    private Servo rightGrabber;

    Servo elbowL;
    Servo wristL;
    Servo elbowR;
    Servo wristR;

    ColorSensor sensorColorL;

    float hsvResult;


    VuforiaLocalizer vuforia;


    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.5  ;    // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0  ;    // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1.0;
    static final double     TURN_SPEED              = 0.7;

    @Override
    public void runOpMode() {


        //robot.init(hardwareMap);
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

        sensorColorL = hardwareMap.get(ColorSensor.class, "sensor_color_l");



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elbowL.setPosition(0.0);
        wristL.setPosition(0.6);
        elbowR.setPosition(0.0);
        wristR.setPosition(0.2);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          frontLeftDrive.getCurrentPosition(),
                          frontRightDrive.getCurrentPosition(),
                          backLeftDrive.getCurrentPosition(),
                          backRightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //set position to neutral
        wristL.setPosition(0.4);
        wristR.setPosition(0.4);
        sleep(500);
        //Lower sensor arm
        elbowL.setPosition(0.53);
        sleep(500);
        //If the color is blue, knock the other one over
        hsvResult = senseColor(5);
        sleep(500);
        if (opModeIsActive() && hsvResult > 50 && hsvResult < 250) {
            wristL.setPosition(0.0);
            sleep(500);
            wristL.setPosition(0.4);
            elbowL.setPosition(0.0);
            sleep(500);
        }
        else {
            wristL.setPosition(1.0);
            sleep(500);
            wristL.setPosition(0.4);
            elbowL.setPosition(0.0);
            sleep(500);
        }



        leftGrabber.setPosition(0.5);
        rightGrabber.setPosition(0.5);
        sleep(500);
        liftMotor.setPower(-0.5);
        sleep(500);
        liftMotor.setPower(0.0);
        sleep(500);

        int image = readImage(2.0);
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  32,  32, 5.0);  // S1: Forward 36 Inches with 5 Sec timeout

        if (image == 1){

        }
        else if (image == 2) {
            encoderDrive(DRIVE_SPEED, 7, 7, 1.0);
        }

        else if (image == 3) {
            encoderDrive(DRIVE_SPEED, 14, 14, 2.0);
        }



        encoderDrive(TURN_SPEED,   17, -17, 4.0);  // S2: Turn Right 0 Inches with 0 Sec timeout
        encoderDrive(DRIVE_SPEED, 20, 20, 1.0);  // S3: Reverse 0 Inches with 0 Sec timeout
        leftGrabber.setPosition(1.0);
        rightGrabber.setPosition(0.0);
        sleep(500);

        encoderDrive(DRIVE_SPEED, -20,-20, 1.0);
        encoderDrive(TURN_SPEED, 30, -30, 4.0);
        encoderDrive(DRIVE_SPEED, -20,-20, 1.0);




        // pause for servos to move

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
    public float senseColor(double timeoutS) {

        float hsvValues[] = {0F, 0F, 0F};
        final double SCALE_FACTOR = 255;

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColorL.red() * SCALE_FACTOR),
                    (int) (sensorColorL.green() * SCALE_FACTOR),
                    (int) (sensorColorL.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            //telemetry.addData("Distance (cm)",
            //        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColorL.alpha());
            telemetry.addData("Red  ", sensorColorL.red());
            telemetry.addData("Green", sensorColorL.green());
            telemetry.addData("Blue ", sensorColorL.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            telemetry.update();
        }
        return hsvValues[0];
    }
    public int readImage(double timeoutS){

        // LEFT: 1, CENTER: 2, RIGHT: 3

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQoL1OT/////AAAAmYWLzC+XjEBmlXjNuz20D6Qdc6TJeVAx2ko5q1i4KwFKXKiVK3pQKuOyVYN2Jm71RtDB0US25Q0qlNmPFZkCzeji6pahjC9j/sA/1g0DrTRsD55qSkWOSyAq2P0E3H6ykeo+vT3pWMiyHDUn/P8sLNeav1dPaXWu8sI+P//jb+8HaPVteJ8CXpF06PseALoOjXNgt+17D+Q1+6hdwmPYKaB7cwAOzIL3IAkVdyP4rbJGYQWsDAYsQ4zgAwGyscXaNPOGzNR2PCRN00ukubvZFuYL+DLRgGLY1+c/Nf8rpgwxgoDVLQpIrTQr5J3cK1VpfOTXZxaQxPu6j0FAxAb7hf11A1w4A706Gjolp1G5Rezn";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");

        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        runtime.reset();

        while(opModeIsActive() && runtime.seconds() > timeoutS ){
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                return 1;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                return 2;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                return 3;
            }
        }

        return 0;

    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            frontLeftDrive.getCurrentPosition(),
                                            frontRightDrive.getCurrentPosition(),
                                            backLeftDrive.getCurrentPosition(),
                                            backRightDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


              sleep(250);   // optional pause after each move
        }
    }
}
