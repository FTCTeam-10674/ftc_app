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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class HardwareHowardbot
{
    /* Public OpMode members. */
    public DcMotor  flDrive   = null;
    public DcMotor  frDrive  = null;
    public DcMotor  blDrive     = null;
    public DcMotor  brDrive    = null;
    public DcMotor  winch   = null;
    public DcMotor  mainArm   = null;
    public DcMotor  leftGrabber = null;
    public DcMotor  rightGrabber = null;
    public Servo    latch = null;
    public Servo    sensArm = null;
    public ColorSensor colSensor;
    int donothing;

    public final static double LATCH_HOME = 0;
    public final static double SENSARM_HOME = 0;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareHowardbot(){

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
        winch = hwMap.get(DcMotor.class, "winch");
        mainArm    = hwMap.get(DcMotor.class, "marm");
        leftGrabber = hwMap.get(DcMotor.class, "lgrabber");
        rightGrabber = hwMap.get(DcMotor.class, "rgrabber");
        latch = hwMap.get(Servo.class, "latch");
        sensArm = hwMap.get(Servo.class, "sensarm");
        colSensor = hwMap.get(ColorSensor.class, "colsens");
        flDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        frDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        blDrive.setDirection(DcMotor.Direction.FORWARD);
        brDrive.setDirection(DcMotor.Direction.REVERSE);
        winch.setDirection(DcMotor.Direction.FORWARD);
        mainArm.setDirection(DcMotor.Direction.FORWARD);
        leftGrabber.setDirection(DcMotor.Direction.FORWARD);
        rightGrabber.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        flDrive.setPower (0);
        frDrive.setPower (0);
        blDrive.setPower (0);
        brDrive.setPower (0);
        winch.setPower (0);
        mainArm.setPower (0);
        leftGrabber.setPower (0);
        rightGrabber.setPower (0);
        latch.setPosition(LATCH_HOME);
        sensArm.setPosition(SENSARM_HOME);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightGrabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ha ha

    }
 }
