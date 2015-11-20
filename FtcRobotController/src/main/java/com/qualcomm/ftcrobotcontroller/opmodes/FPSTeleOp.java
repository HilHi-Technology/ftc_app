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
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
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
        boolean joy1deadzone = false; //Create two psuedo-boolean values, which will be either 0, 1, or 2, representing false, true, or undefined.
        boolean joy2deadzone = false;

        if (joy1Y < 0.1 && joy1Y > -0.1 && joy1X > -0.1 && joy1X < 0.1) { //If the left, or the power joystick is in the deadzone...
            motorPower = 0; //Set the motors' power to be 0.
            joy1deadzone = true;
        }

        if (joy2Y < 0.1 && joy2Y > -0.1 && joy2X > -0.1 && joy2X < 0.1) { //If the right, or the steering joystick is within the deadzone...
            motorCurve = 0; //Set the motors' curve to be 0
            joy2deadzone = true;
        }

        float leftMotorPower = 0; //Create variables to store the final values
        float rightMotorPower = 0;

        if (!joy1deadzone && !joy2deadzone) {
            leftMotorPower = motorPower + motorCurve;
            rightMotorPower = motorPower - motorCurve;
        } else if (joy1deadzone && joy2deadzone) {
            leftMotorPower = 0;
            rightMotorPower = 0;
        } else if (joy1deadzone) {
            leftMotorPower = motorCurve;
            rightMotorPower = -motorCurve;
        } else if (joy2deadzone) {
            leftMotorPower = motorPower;
            rightMotorPower = motorPower;
        }

        if (leftMotorPower > 100) {
            float amountToSubtract = leftMotorPower - 100;
            leftMotorPower = leftMotorPower - amountToSubtract;
            rightMotorPower = rightMotorPower - amountToSubtract;
        }
        if (rightMotorPower > 100) {
            float amountToSubtract = rightMotorPower - 100;
            leftMotorPower = leftMotorPower - amountToSubtract;
            rightMotorPower = rightMotorPower - amountToSubtract;
        }

        if (leftMotorPower < -100) {
            float amountToAdd = leftMotorPower + 100;
            leftMotorPower = leftMotorPower - amountToAdd;
            rightMotorPower = rightMotorPower - amountToAdd;
        }
        if (rightMotorPower < -100) {
            float amountToAdd = rightMotorPower + 100;
            leftMotorPower = leftMotorPower - amountToAdd;
            rightMotorPower = rightMotorPower - amountToAdd;
        }

        float leftMotorPowerFloat = leftMotorPower / 100f;
        float rightMotorPowerFloat = rightMotorPower / 100f;

        leftMotor.setPower(leftMotorPowerFloat);
        rightMotor.setPower(rightMotorPowerFloat);
    }
}
