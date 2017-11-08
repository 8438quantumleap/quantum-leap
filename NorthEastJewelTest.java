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

import android.app.Activity;
import android.graphics.Color;
import android.provider.Settings;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="NorthEastJewelTest(Red)", group="Linear Opmode")
public class NorthEastJewelTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor Tilt = null;

    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = (MAX_POS - MIN_POS) / 2;
    Servo   servo;


    /** The colorSensor field will contain a reference to our color sensor hardware object */
    NormalizedColorSensor colorSensor;
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot */
    View relativeLayout;

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
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        servo = hardwareMap.get(Servo.class, "jewel");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        boolean toggleLight = false;

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

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
            toggleLight = true;
            if (toggleLight) {
                if (colorSensor instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight)colorSensor;
                    light.enableLight(!light.isLightOn());
                }
            }

            //Arm Code
            MoveLeft(4);
            ColorSensorJewelArm();
            MoveRight(4);
            //Driving Code
            MoveBackward(24);
            MoveRight(12);
            TurnLeft(180);
            MoveForward(16);


            active = false;
        }

        leftPower    = 0 ;
        rightPower   = 0 ;
        frontLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);


    }
    public void MoveForward(double inches) {
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
    public void MoveBackward(double inches) {
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
    public void TurnLeft(double degrees) {
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
    public void TurnRight(double degrees) {
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
    public void MoveLeft(double inches) {
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
    public void MoveRight(double inches) {
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
    public void ColorSensorJewelArm() {
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;
        boolean detectRed = false;
        boolean detectBlue = false;

        servo.setPosition(MAX_POS);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        Color.colorToHSV(colors.toColor(), hsvValues);
        telemetry.addLine()
                .addData("H", "%.3f", hsvValues[0])
                .addData("S", "%.3f", hsvValues[1])
                .addData("V", "%.3f", hsvValues[2]);
        telemetry.addLine()
                .addData("a", "%.3f", colors.alpha)
                .addData("r", "%.3f", colors.red)
                .addData("g", "%.3f", colors.green)
                .addData("b", "%.3f", colors.blue);

        int color = colors.toColor();

        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;//divides relative to max
        colors.blue  /= max;
        color = colors.toColor();

        telemetry.addLine("normalized color:  ")
                .addData("a", "%02x", Color.alpha(color))
                .addData("r", "%02x", Color.red(color))
                .addData("g", "%02x", Color.green(color))
                .addData("b", "%02x", Color.blue(color));
        telemetry.update();

        if (colors.red == 1){
            detectRed = true;
        }
        else if (colors.blue == 1){
            detectBlue = true;
        }

        if (detectBlue){
            TurnLeft(30);
            TurnRight(30);

        }
        else if (detectRed){
            TurnRight(30);
            TurnLeft(30);

        }
        servo.setPosition(MIN_POS);

    }
}
