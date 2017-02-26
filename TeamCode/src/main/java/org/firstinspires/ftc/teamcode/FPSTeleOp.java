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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

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

@TeleOp(name="FPSTeleOp", group="Iterative Opmode")  // @Thingy?(...) is the other common choice
public class FPSTeleOp extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor sweep = null;
    public DcMotor spin1 = null;
    public DcMotor spin2 = null;
    public DcMotor arm = null;
    public Servo pushLeft = null;
    public Servo pushRight = null;
    public Servo liftLock;
    public ColorSensor colorSensor;

    private final float turnMultiplier = 0.5f;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");
        sweep = hardwareMap.dcMotor.get("sweep");
        spin1 = hardwareMap.dcMotor.get("spin1");
        spin2 = hardwareMap.dcMotor.get("spin2");
        pushLeft = hardwareMap.servo.get("pushLeft");
        pushRight = hardwareMap.servo.get("pushRight");
        arm = hardwareMap.dcMotor.get("arm");
        liftLock = hardwareMap.servo.get("lock");

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        spin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");
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
        //Change status to running for debug
        telemetry.addData("Status", "Running: " + runtime.toString());

        //Get raw joystick input
        float forwardStick = -gamepad1.left_stick_y;
        float turnStick = -gamepad1.right_stick_x;
        boolean shootOut = gamepad1.right_bumper;
        float shootOutFast = gamepad1.right_trigger;
        boolean sweepIn = gamepad1.left_bumper;
        float sweepOut = gamepad1.left_trigger;

        float newLeftEncoder = 0;
        float oldLeftEncoder = 0;
        float newRightEncoder = 0;
        float oldRightEncoder = 0;
        float RPM = 0;
        float currentPower = 0.5f;

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

        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
        // leftMotor.setPower(-gamepad1.left_stick_y);
        // rightMotor.setPower(-gamepad1.right_stick_y);

        if (sweepIn) {
            telemetry.addData("SweepIn", sweepIn);
            telemetry.update();
            sweep.setPower(1);
        } else if (sweepOut > 0) {
            sweep.setPower(-1);
        } else {
            sweep.setPower(0);
        }

        if (shootOut) {
            //spin1.setPower(0.65);
            //spin2.setPower(0.65);
            telemetry.addData("Shootout", shootOut);
            telemetry.update();
            spin1.setPower(currentPower);
            spin2.setPower(currentPower);

            runtime.reset();
            while (runtime.seconds() < 1) {
            }

            while (RPM < 650) {
                newLeftEncoder = spin1.getCurrentPosition();
                newRightEncoder = spin2.getCurrentPosition();

                spin1.setPower(currentPower);
                spin2.setPower(currentPower);

                runtime.reset();
                while (runtime.seconds() < 0.1) {
                }

                currentPower += 0.015f;
                RPM = (((newLeftEncoder - oldLeftEncoder) + (newRightEncoder - oldRightEncoder)) / 2) / (int)runtime.seconds();

                oldLeftEncoder = newLeftEncoder;
                oldRightEncoder = newRightEncoder;
            }

            telemetry.addData("Sweeping", sweep);
            telemetry.update();

            sweep.setPower(1);

            runtime.reset();
            while (runtime.seconds() < 5) {
            }

            sweep.setPower(0);
            spin1.setPower(0);
            spin2.setPower(0);

        } else {
            //spin1.setPower(spin1.getPower() * 0.98);
            //spin2.setPower(spin2.getPower() * 0.98);
        }

        if (shootOutFast > 0) {
            //spin1.setPower(0.9);
            //spin2.setPower(0.9);
            telemetry.addData("Shootout", shootOut);
            telemetry.update();
            spin1.setPower(0.5);
            spin2.setPower(0.5);

            runtime.reset();
            while (runtime.seconds() < 1) {
            }

            while (RPM < 900) {
                runtime.reset();

                spin1.setPower(currentPower);
                spin2.setPower(currentPower);

                runtime.reset();
                while (runtime.seconds() < 0.1) {
                }

                currentPower = currentPower + 0.015f;
                RPM = (((newLeftEncoder - oldLeftEncoder) - (newRightEncoder - oldRightEncoder)) / 2) / (int)runtime.seconds();
            }

            sweep.setPower(1);

            runtime.reset();
            while (runtime.seconds() < 5) {
            }

            sweep.setPower(0);
            spin1.setPower(0);
            spin2.setPower(0);


        } else {
            //spin1.setPower(spin1.getPower() * 0.98);
            //spin2.setPower(spin2.getPower() * 0.98);
        }

        if (gamepad1.x) {
            pushLeft.setPosition(0);
        } else if (gamepad1.b) {
            pushLeft.setPosition(1);
        } else {
            pushLeft.setPosition(0.5);
        }

        if (gamepad1.dpad_left) {
            pushRight.setPosition(0);
        } else if (gamepad1.dpad_right) {
            pushRight.setPosition(1);
        } else {
            pushRight.setPosition(0.5);
        }

        if (gamepad1.y) {
            arm.setPower(1);
        } else if (gamepad1.a) {
            arm.setPower(-1);
        } else {
            arm.setPower(0);
        }

        if (gamepad1.dpad_up) {
            liftLock.setPosition(0);
        } else {
            liftLock.setPosition(1);
        }

    }
}