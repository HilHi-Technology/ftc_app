/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes    contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FPSTeleOp", group="Iterative Opmode")
public class FPSTeleOp extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor armMotor = null;
    public Servo leftGrab = null;
    public Servo rightGrab = null;
    public Servo golf = null;
    private final float turnMultiplier = 0.5f;
    boolean toggleGrab = false;
    boolean doToggleGrab = true;
    boolean toggleGolf = false;
    boolean doToggleGolf = true;

    /* Code to run ONCE when the driver hits INIT */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");
        armMotor = hardwareMap.dcMotor.get("arm");
        leftGrab = hardwareMap.servo.get("lg");
        rightGrab = hardwareMap.servo.get("rg");
        golf = hardwareMap.servo.get("golf");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftGrab.setDirection(Servo.Direction.REVERSE);
    }

    /* Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY */
    @Override
    public void init_loop() {}

    /* Code to run ONCE when the driver hits PLAY */
    @Override
    public void start() {
        runtime.reset();
    }

    /* Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP */
    @Override
    public void loop() {
        //Change status to running for debug
        telemetry.addData("Status", "Running: " + runtime.toString());

        //Get raw joystick input
        float forwardStick = gamepad1.left_stick_y;
        float turnStick = -gamepad1.right_stick_x;

        telemetry.addData("ForwardStick", forwardStick);
        telemetry.addData("TurnStick", turnStick);


        //Create deadzone
        float forwardPower = forwardStick;
        float turnPower = turnStick * turnMultiplier;

        //Apply turn to forward power (individual for each motor),
        float leftPower = forwardPower != 0f ? forwardPower - turnPower : -turnPower;
        float rightPower = forwardPower != 0f ? forwardPower + turnPower : turnPower;

        float amountOverMax = 0f;
        //Adjust motor values down if they're too high
        if (leftPower > 1f || rightPower > 1f) {
            amountOverMax = leftPower > 1f ? leftPower - 1f : rightPower - 1f;
        } else if (leftPower < -1f || rightPower < -1f) {
            amountOverMax = leftPower < -1f ? leftPower + 1f : rightPower + 1f;
        }

        leftPower -= amountOverMax;
        rightPower -= amountOverMax;

        leftMotor.setPower(leftPower);
        telemetry.addData("LeftPower", leftPower);
        rightMotor.setPower(rightPower);
        telemetry.addData("RightPower", rightPower);

        telemetry.addData("ForwardPower", forwardPower);
        telemetry.addData("TurnPower", turnPower);


        if (gamepad1.left_trigger > 0) {
            armMotor.setPower(1);
        }
        else if (gamepad1.right_trigger > 0) {
            armMotor.setPower(-1);
        }
        else {
            armMotor.setPower(0);
        }

        if (!toggleGrab && doToggleGrab && gamepad1.a) {
            leftGrab.setPosition(0.05);
            rightGrab.setPosition(0.05);
            toggleGrab = true;
            doToggleGrab = false;
        }
        else if (toggleGrab && doToggleGrab && gamepad1.a) {
            leftGrab.setPosition(1);
            rightGrab.setPosition(1);
            toggleGrab = false;
            doToggleGrab = false;
        }
        else if (!gamepad1.a) {
            doToggleGrab = true;
        }

        /*
        if (!toggleGolf && doToggleGolf && gamepad1.b) {
            golf.setPosition(1);
            toggleGolf = true;
            doToggleGolf = false;
        }
        else if (toggleGolf && doToggleGolf && gamepad1.b) {
            golf.setPosition(0.55);
            toggleGolf = false;
            doToggleGolf = false;
        }
        else if (!gamepad1.b) {
            doToggleGolf = true;
        }
        */

    }
}