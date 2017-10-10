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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryGroup;
import org.firstinspires.ftc.teamcode.telemetry.TelemetryGroupManager;

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

@TeleOp(name="Debug: Controller Test", group="Debug")
@Disabled
public class ControllerTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private TelemetryGroupManager telemetryGroupManager;
    private TelemetryGroup controller1 = new TelemetryGroup("Controller One");
    private TelemetryGroup controller1Buttons = new TelemetryGroup("Buttons");
    private TelemetryGroup controller1Joysticks = new TelemetryGroup("Joysticks");
    private TelemetryGroup controller2 = new TelemetryGroup("Controller Two");
    private TelemetryGroup controller2Buttons = new TelemetryGroup("Buttons");
    private TelemetryGroup controller2Joysticks = new TelemetryGroup("Joysticks");

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetryGroupManager = new TelemetryGroupManager(telemetry);
        controller1.addSubgroup(controller1Buttons);
        controller1.addSubgroup(controller1Joysticks);
        controller1.addSubgroup(controller2Buttons);
        controller1.addSubgroup(controller2Joysticks);
        telemetryGroupManager.addTelemetryGroup(controller1);
        telemetryGroupManager.addTelemetryGroup(controller2);
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
        controller1Buttons.setData("A", String.valueOf(gamepad1.a));
        controller1Buttons.setData("B", String.valueOf(gamepad1.b));
        controller1Buttons.setData("X", String.valueOf(gamepad1.x));
        controller1Buttons.setData("Y", String.valueOf(gamepad1.y));

        controller2Buttons.setData("A", String.valueOf(gamepad1.a));
        controller2Buttons.setData("B", String.valueOf(gamepad1.b));
        controller2Buttons.setData("X", String.valueOf(gamepad1.x));
        controller2Buttons.setData("Y", String.valueOf(gamepad1.y));

        controller1Joysticks.setData("Right X", String.valueOf(gamepad1.right_stick_x));
        controller1Joysticks.setData("Right Y", String.valueOf(gamepad1.right_stick_y));
        controller1Joysticks.setData("Left X", String.valueOf(gamepad1.left_stick_x));
        controller1Joysticks.setData("Left Y", String.valueOf(gamepad1.left_stick_y));

        controller2Joysticks.setData("Right X", String.valueOf(gamepad2.right_stick_x));
        controller2Joysticks.setData("Right Y", String.valueOf(gamepad2.right_stick_y));
        controller2Joysticks.setData("Left X", String.valueOf(gamepad2.left_stick_x));
        controller2Joysticks.setData("Left Y", String.valueOf(gamepad2.left_stick_y));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

