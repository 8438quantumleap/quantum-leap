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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.provider.Settings;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import static android.os.SystemClock.sleep;

@TeleOp(name="Main TeleOp", group="Iterative Opmode")
public class JacksonsTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor glypherArmTilt = null;
    private DcMotor glypherArmYax = null;
    private DcMotor glypherPinch = null;

    //temp
    //private Servo ts;

    //private Servo jewel = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        glypherArmTilt = hardwareMap.get(DcMotor.class, "tilt");
        glypherArmYax = hardwareMap.get(DcMotor.class, "up");
        glypherPinch = hardwareMap.get(DcMotor.class, "pinch");

        //temp
        //ts = hardwareMap.get(Servo.class, "ts");

        //jewel = hardwareMap.get(Servo.class, "jewel");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        glypherArmYax.setDirection(DcMotorSimple.Direction.REVERSE);
        //glypherPinch.setDirection(DcMotorSimple.Direction.REVERSE);

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
        //jewel.setPosition(.05);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        gamepadOneStuff();
        gamepadTwoStuff();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    private void gamepadOneStuff(){
        double leftPower = 0;
        double rightPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        double leftY = gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;

        double rightY = gamepad1.right_stick_y;
        if(gamepad1.right_bumper || gamepad1.right_trigger > 0) {
            if (rightY == 0) {
                rightPower = leftX;
                leftPower = -leftX;
                leftBackPower = leftX;
                rightBackPower = -leftX;
            }
        } else {
            leftPower = leftBackPower = gamepad1.left_stick_y;
            rightPower = rightBackPower = gamepad1.right_stick_y;
        }

        double powermult = .75;

        /*if(gamepad1.left_trigger > 0){
            powermult = .25;
        } else*/ if(gamepad1.left_bumper){
            powermult = 1;
        }

        leftDrive.setPower(leftPower*powermult);
        rightDrive.setPower(rightPower*powermult);
        leftBackDrive.setPower(leftBackPower*powermult);
        rightBackDrive.setPower(rightBackPower*powermult);

        //test
        /*if(gamepad1.a){
            ts.setPosition(90);
        }
        if(gamepad1.b){
            ts.setPosition(180);
        }
        if(gamepad1.x){
            ts.setPosition(0);
        }
        */
    }
// this is coding not minecraft
    private void gamepadTwoStuff(){
        double glypherPinchPower = 0;

        if(Math.abs(gamepad2.left_stick_x) > Math.abs(gamepad2.left_stick_y)){
            if(gamepad2.left_stick_x > 0){
                glypherPinchPower = .40;
            } else if(gamepad2.left_stick_x < 0){
                glypherPinchPower = -.40;
            }
        } else {
            if(gamepad2.left_trigger > 0) {
                //slow mode
                glypherArmYax.setPower(gamepad2.left_stick_y * .40);
            } else if(gamepad2.left_bumper){
                //fast mode
                glypherArmYax.setPower(gamepad2.left_stick_y * .60);
            } else {
                glypherArmYax.setPower(gamepad2.left_stick_y * .45);
            }
        }
        if(gamepad2.right_trigger > 0) {
            glypherPinch.setPower(gamepad2.right_trigger*.4);
        }

        if(gamepad2.right_bumper) {
            glypherPinch.setPower(-.25);
        }
        else glypherPinch.setPower(0);

        glypherArmTilt.setPower(gamepad2.right_stick_y*.25);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

