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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="TeleOp Ruckus", group="Linear Opmode")
//@Disabled
public class Ruckus_TeleOp extends LinearOpMode {
    Ruckus_HwMap howard   = new Ruckus_HwMap();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        howard.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        howard.flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        howard.frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        howard.blDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        howard.brDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        howard.lWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        howard.armSwing.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        howard.lWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        howard.armSwing.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //temporarily WITHOUT for testing purposes

        double dumpPos = howard.UNDUMPED;
        double latchPos = howard.LATCH_OPEN;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //double leftGrabberPosition = 0.0;
        //double rightGrabberPosition = 1.0;

        //howard.lGrabbo.setPosition(leftGrabberPosition);
        //howard.rGrabbo.setPosition(rightGrabberPosition);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double flPower;
            double frPower;
            double blPower;
            double brPower;
            double winchPower;
            double winchPos = howard.lWinch.getCurrentPosition();
            double swingPower;
            double swingPos = howard.armSwing.getCurrentPosition();
            //double collectorPower;



            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            //Drive train controls
            //FIX DRIVE AND STRAFE: drive on y, strafe on x.
            double drive  = -gamepad1.left_stick_y;
            double rotate =  gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            flPower  = Range.clip(drive + rotate - strafe, -1.0, 1.0);
            frPower  = Range.clip(drive - rotate + strafe, -1.0, 1.0);
            blPower  = Range.clip(drive + rotate + strafe, -1.0, 1.0);
            brPower  = Range.clip(drive - rotate - strafe, -1.0, 1.0);

            //Arm controls
            /*if (winchPos <= howard.WINCH_MIN){
                winchPower = Range.clip(-gamepad2.left_stick_y, 0.0, 1.0);
            }
            else if (winchPos >= howard.WINCH_MAX){
                winchPower = Range.clip(-gamepad2.left_stick_y, -1.0, 0.0);
            }
            else {
                winchPower = Range.clip(-gamepad2.left_stick_y, 1.0, -1.0);
            }

            if (swingPos <= howard.SWING_MIN){
                swingPower = Range.clip(-gamepad2.right_stick_y, 0.0, 1.0);
            }
            else if (swingPos >= howard.SWING_MAX){
                swingPower = Range.clip(-gamepad2.right_stick_y, -1.0, 0.0);
            }
            else {
                swingPower = Range.clip(-gamepad2.left_stick_y, 1.0, -1.0);
            }*/

            /*if (gamepad2.right_bumper){
                leftGrabberPosition = 0.3;
                rightGrabberPosition = 0.7;
                //grabberOpen = true;
            }

            else if (gamepad2.left_bumper){
                leftGrabberPosition = 0.6;
                rightGrabberPosition = 0.4;
            }

            else if (gamepad2.left_trigger > 0.1){
                leftGrabberPosition = 0.4;
                rightGrabberPosition = 0.6;
            }

            else if (gamepad2.right_trigger > 0.1) {
                leftGrabberPosition = 0.0;
                rightGrabberPosition = 1.0;
            }
*/
            //Collector controls
           /* if(gamepad2.right_trigger > 0.75){
                collectorPower = howard.COLLECTOR_POWER;
            }
            else if(gamepad2.left_trigger > 0.75){
                collectorPower = -howard.COLLECTOR_POWER;
            }
            else{
                collectorPower = 0;
            }*/

           winchPower = -gamepad2.left_stick_y;
           swingPower = -gamepad2.right_stick_y;

            //Latch controls
            /*if(gamepad2.a) {
                latchPos = howard.LATCH_OPEN;
            }
            else if(gamepad2.b) {
                latchPos = howard.LATCH_CLOSED;
            }*/

            //Dumper controls
            if(gamepad1.x) {
                dumpPos = howard.DUMPED;
            }
            else if(gamepad1.y) {
                dumpPos = howard.UNDUMPED;
            }
            // Send calculations to motors/servos
            howard.flDrive.setPower(flPower);
            howard.frDrive.setPower(frPower);
            howard.blDrive.setPower(blPower);
            howard.brDrive.setPower(brPower);
            howard.lWinch.setPower(winchPower);
            howard.rWinch.setPower(winchPower);
            howard.armSwing.setPower(swingPower);
            howard.dumper.setPosition(dumpPos);
            howard.latch.setPosition(latchPos);
            //howard.lGrabbo.setPosition(leftGrabberPosition);
            //howard.rGrabbo.setPosition(rightGrabberPosition);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor Powers", "fl (%.2f), fr (%.2f), bl (%.2f), br (%.2f)",
                                               flPower,   frPower,   blPower,   brPower);
            telemetry.addData("Winch Position", "count (%7f)", winchPos);
            telemetry.addData("Servos", "latch (%.2f), dumper (%.2f)", latchPos, dumpPos);
            telemetry.update();
        }
    }
}
