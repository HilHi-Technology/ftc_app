package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jonathan on 9/28/2015.
 */
public class FPSTeleOp extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    public FPSTeleOp() {

    }

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        float joy1Y = gamepad1.left_stick_y;
        float joy1X = gamepad1.left_stick_x;
        float joy2X = gamepad1.right_stick_x;
        float joy2Y = gamepad1.right_stick_y;

        float motorPower = joy1Y * 100; //Create variables for storing the motor power and the curve to be applied to the motors.
        float motorCurve = joy2X * 50;
        int joy1Up = 2; //Create two psuedo-boolean values, which will be either 0, 1, or 2, representing false, true, or undefined.
        int joy2Left = 2;

        if (joy1Y < 0.1 && joy1Y > -0.1 && joy1X > -0.1 && joy1X < 0.1) { //If the left, or the power joystick is in the deadzone...
            motorPower = 0; //Set the motors' power to be 0.
        } else if (joy1Y > 0) { //If the power joystick is forward...
            joy1Up = 1; //Set joy1Up to 1, showing that the left joystick is up.
        } else { //If the joystick is not within the deadzone, or being pushed forward, it must be being pulled backwards, in which case...
            joy1Up = 0; //Set joy1Up to 0, showing that the left joystick is down.
        }

        if (joy2Y < 0.1 && joy2Y > -0.1 && joy2X > -0.1 && joy2X < 0.1) { //If the right, or the steering joystick is within the deadzone...
            motorCurve = 0; //Set the motors' curve to be 0
        } else if (joy2X > 0) { //If the control joystick is not in the deadzone, and it's to the right...
            joy2Left = 0; //Set joy2Left to 0, signifying that the right joystick is right.
        } else if (joy2X < 0) { //If the control joystick is not in the deadzone, nor to the right, it must be to the left, in which case...
            joy2Left = 1; //Set joy2Left to 1, signifying that the right joystick is to the left.
        }

        float motorPowerD = 0; //Create variables to store the final values
        float motorPowerE = 0;

        if (joy1Up == 1 && joy2Left == 1) {
            motorPowerD = motorPower - motorCurve;
            motorPowerE = motorPower + motorCurve;
        } else if (joy1Up == 1 && joy2Left == 0) {
            motorPowerD = motorPower - motorCurve;
            motorPowerE = motorPower + motorCurve;
        } else if (joy1Up == 0 && joy2Left == 1) {
            motorPowerD = motorPower - motorCurve;
            motorPowerE = motorPower + motorCurve;
        } else if (joy1Up == 0 && joy2Left == 0) {
            motorPowerD = motorPower - motorCurve;
            motorPowerE = motorPower + motorCurve;
        } else if (joy1Up == 2) {
            motorPowerD = -motorCurve;
            motorPowerE = motorCurve;
        } else if (joy2Left == 2) {
            motorPowerD = motorPower;
            motorPowerE = motorPower;
        } else if (joy1Up == 2 && joy2Left == 2) {
            motorPowerD = 0;
            motorPowerE = 0;
        }

        if (motorPowerD > 100) {
            float amountToSubtract = motorPowerD - 100;
            motorPowerD = motorPowerD - amountToSubtract;
            motorPowerE = motorPowerE - amountToSubtract;
        }
        if (motorPowerE > 100) {
            float amountToSubtract = motorPowerE - 100;
            motorPowerD = motorPowerD - amountToSubtract;
            motorPowerE = motorPowerE - amountToSubtract;
        }

        if (motorPowerD < -100) {
            float amountToAdd = motorPowerD + 100;
            motorPowerD = motorPowerD - amountToAdd;
            motorPowerE = motorPowerE - amountToAdd;
        }
        if (motorPowerE < -100) {
            float amountToAdd = motorPowerE + 100;
            motorPowerD = motorPowerD - amountToAdd;
            motorPowerE = motorPowerE - amountToAdd;
        }

        float motorPowerDFloat = motorPowerD / 100f;
        float motorPowerEFloat = motorPowerE / 100f;

        leftMotor.setPower(motorPowerDFloat);
        rightMotor.setPower(motorPowerEFloat);
    }
}
