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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Ruckus_HwMap
{
    //Electronics
    public DcMotor  flDrive    = null;
    public DcMotor  frDrive    = null;
    public DcMotor  blDrive    = null;
    public DcMotor  brDrive    = null;
    public DcMotor  lWinch   = null;
    public DcMotor  rWinch   = null;
    public DcMotor  armSwing   = null;
    public Servo    latch      = null;
    public Servo    dumper     = null;
    public Servo    lGrabbo    = null;
    public Servo    rGrabbo    = null;

    //Gyro
    BNO055IMU gyro;
    OpenGLMatrix lastLocation = null;
    Orientation angles;
    Acceleration gravity;

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro.initialize(parameters);
    }


    //Encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // 1120 is for Neverrest motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.6;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //Color sensor
    float hsvResult;

    //Vuforia
    VuforiaLocalizer vuforia;

    //Constants
    public final static double LATCH_CLOSED  = 1;
    public final static double LATCH_OPEN    = 0;
    public final static long TIME_TO_EXTEND = 4 * 1000;
    public final static long TIME_TO_RETRACT = 7 * 1000;
    public final static double WINCH_POWER   = 0.4;
    //public final static double COLLECTOR_POWER = 1;
    public final static double UNDUMPED = 0;
    public final static double DUMPED = 1;
    public final static double WINCH_MIN = 0;
    public final static double WINCH_MAX = 224000; //these values
    public final static double SWING_MIN = -1120;  //are completely
    public final static double SWING_MAX = 224000; //arbitrary


    //local OpMode members
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Ruckus_HwMap(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        flDrive  = hwMap.get(DcMotor.class, "fldrive");
        frDrive = hwMap.get(DcMotor.class, "frdrive");
        blDrive  = hwMap.get(DcMotor.class, "bldrive");
        brDrive = hwMap.get(DcMotor.class, "brdrive");
        lWinch = hwMap.get(DcMotor.class, "lwinch");
        rWinch = hwMap.get(DcMotor.class, "rwinch");
        armSwing    = hwMap.get(DcMotor.class, "swing");
        lGrabbo = hwMap.get(Servo.class, "lgrab");
        rGrabbo = hwMap.get(Servo.class, "rgrab");
        dumper = hwMap.get(Servo.class, "dumper");
        latch = hwMap.get(Servo.class, "latch");
        gyro = hwMap.get(BNO055IMU.class, "imu_gyro");
        flDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        blDrive.setDirection(DcMotor.Direction.REVERSE);
        brDrive.setDirection(DcMotor.Direction.FORWARD);
        lWinch.setDirection(DcMotor.Direction.REVERSE);
        rWinch.setDirection(DcMotor.Direction.REVERSE);
        armSwing.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        flDrive.setPower (0);
        frDrive.setPower (0);
        blDrive.setPower (0);
        brDrive.setPower (0);
        lWinch.setPower (0);
        rWinch.setPower (0);
        armSwing.setPower (0);
        //latch.setPosition(LATCH_CLOSED);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //we only need one here, we don't both l and r
        armSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

 }

