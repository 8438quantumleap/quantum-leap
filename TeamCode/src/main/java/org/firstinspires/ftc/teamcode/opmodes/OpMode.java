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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="First Opmode", group="Iterative Opmode")
public class OpMode extends com.qualcomm.robotcore.eventloop.opmode.OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

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
        double frontLeftPower = 0;
        double backLeftPower = 0;
        double frontRightPower = 0;
        double backRightPower = 0;
        String direction = "stopped";
        /*Controller setup
        left stick + right bumper - move left and right
        left stick + right stick + right bumper - move diagonally
        left stick - left wheels
        right stick - right wheels
         */

        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightY = gamepad1.right_stick_y;
        double rightX  =  gamepad1.right_stick_x;
        boolean rightButtonDown = gamepad1.right_bumper;
        if(rightButtonDown){
            //strafe right and left
            if(rightY == 0) {
                frontRightPower = -leftX;
                frontLeftPower = leftX;
                backLeftPower = -leftX;
                backRightPower = leftX;
                if(leftX > 0){
                    direction = "strafing right";
                } else if (leftX < 0){
                    direction = "strafing left";
                }
                /*
                  Diagonal. Right stick will control forwards and backwards.
                  Left stick will will control left and right
                 */
            } else if (rightY > 0){ //Forwards
                if(leftX > 0){ //Right
                    frontLeftPower = ((leftX/2 + rightY/2)/2);
                    backRightPower = ((leftX/2 + rightY/2)/2);
                    direction = "diagonal front-right";
                } else if(leftX < 0){ //Left
                    frontRightPower = ((Math.abs(leftX/2) + rightY/2)/2);
                    backLeftPower = ((Math.abs(leftX/2) + rightY/2)/2);
                    direction = "diagonal front-left";
                }
            } else if (rightY < 0){ //Backwards
                if(leftX > 0){ //Right
                    frontLeftPower = -((leftX/2 + rightY/2)/2);
                    backRightPower = -((leftX/2 + rightY/2)/2);
                    direction = "diagonal back-right";
                } else if(leftX < 0){ //Left
                    frontRightPower = -((Math.abs(leftX/2) + rightY/2)/2);
                    backLeftPower = -((Math.abs(leftX/2) + rightY/2)/2);
                    direction = "diagonal back-left";
                }
            }
        } else { //Normal Tank Drive
            frontLeftPower = leftY;
            backLeftPower = leftY;
            frontRightPower = rightY;
            backRightPower = rightY;
            if (leftY > 0){
                if (rightY > 0){
                    direction = "forwards";
                } else if (rightY < 0){
                    direction = "pivot right";
                } else {
                    direction = "turn right";
                }
            } else if (leftY < 0){
                if(rightY < 0){
                    direction = "backwards";
                } else if (rightY > 0){
                    direction = "pivot left";
                }
            } else if(rightY > 0){
                direction = "turn left";
            }
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(gamepad1.right_trigger>1?frontLeftPower*2:frontLeftPower);
        leftBackDrive.setPower(gamepad1.right_trigger>1?backLeftPower*2:backLeftPower);
        rightFrontDrive.setPower(gamepad1.right_trigger>1?frontRightPower*2:frontRightPower);
        rightBackDrive.setPower(gamepad1.right_trigger>1?backRightPower*2:backRightPower);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f) | front right (%.2f), back left (%.2f)| back right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Direction", direction);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
