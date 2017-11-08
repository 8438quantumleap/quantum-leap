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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

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

@Autonomous(name="SouthEastAutonomous", group="Linear Opmode")
public class SouthEastAutonomous extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor Tilt = null;
    //This code is commented out because we do not have a compass
    //CompassSensor compass;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //Tilt = hardwareMap.get(DcMotor.class, "tilt");
        //This code is commented out because we do not have a compass
        //compass = hardwareMap.get(ModernRoboticsI2cCompassSensor.class, "compass");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        boolean active = true;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && active)
        {
            //Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //This code is commented out because we do not have a compass
            //double direction;
            //direction = compass.getDirection();
            //telemetry.addData("Direction", "Direction: " + direction);
            //telemetry.update();

            //Add Arm Code
            MoveBackward(24);
            MoveLeft(24);




            active = false;
        }

        leftPower    = 0 ;
        rightPower   = 0 ;
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);


    }
    public void MoveForward(double inches)
    {
        double Power = 0.25;
        long time = (long)(inches * 65);//(semi precise)
        frontLeftDrive.setPower(Power);
        frontRightDrive.setPower(Power);
        backLeftDrive.setPower(Power);
        backRightDrive.setPower(Power);
        telemetry.addData("Move Forward", "Inches (" + inches +"), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }
    public void MoveBackward(double inches)
    {
        double Power = 0.25;
        long time = (long)(inches * 65);//(semi precise)
        frontLeftDrive.setPower(Power * -1);
        frontRightDrive.setPower(Power * -1);
        backLeftDrive.setPower(Power * -1);
        backRightDrive.setPower(Power * -1);
        telemetry.addData("Move Backward", "Inches (" + inches +"), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }
    public void TurnLeft(double degrees)
    {
        double Power = 0.25;
        long time = (long)(degrees * 15.70);//(precise)
        frontLeftDrive.setPower(Power * -1);
        frontRightDrive.setPower(Power);
        backLeftDrive.setPower(Power * -1);
        backRightDrive.setPower(Power);
        telemetry.addData("Left Turn", "Degrees (" + degrees + "), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void TurnRight(double degrees)
    {
        double Power = 0.25;
        long time = (long)(degrees * 15.70);//(precise)
        frontLeftDrive.setPower(Power);
        frontRightDrive.setPower(Power * -1);
        backLeftDrive.setPower(Power);
        backRightDrive.setPower(Power * -1);
        telemetry.addData("Right Turn", "Degrees (" + degrees + "), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void MoveLeft(double inches)
    {
        double LeftPower = 0.45;
        double RightPower = 0.55;
        long time = (long)(inches * 100);//NOT THE REAL VALUE (imprecise)
        frontLeftDrive.setPower(LeftPower * -1);
        frontRightDrive.setPower(RightPower);
        backLeftDrive.setPower(LeftPower);
        backRightDrive.setPower(RightPower * -1);
        telemetry.addData("Move Left", "Inches (" + inches +"), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
    public void MoveRight(double inches)
    {
        double LeftPower = 0.45;
        double RightPower = 0.55;
        long time = (long) (inches * 100);//NOT THE REAL VALUE (imprecise)
        frontLeftDrive.setPower(LeftPower);
        frontRightDrive.setPower(RightPower * -1);
        backLeftDrive.setPower(LeftPower * -1);
        backRightDrive.setPower(RightPower);
        telemetry.addData("Move Right", "Inches (" + inches + "), Time (" + time + ")");
        telemetry.update();
        sleep(time);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
