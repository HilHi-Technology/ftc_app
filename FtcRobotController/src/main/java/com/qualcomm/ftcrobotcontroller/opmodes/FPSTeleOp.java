package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jonathan on 9/28/2015.
 */
public class FPSTeleOp extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor hook1Motor;
    DcMotor hook2Motor;
    /*DcMotor armAngleMotor;
    DcMotor armExtendMotor;*/

    Servo leftPusher;
    Servo rightPusher;

    Servo pulley;

    Servo dumper;

    Servo leftZipline;
    Servo rightZipline;

    private boolean hookDown = false;

    private boolean button1 = false;

    private boolean dumpDown = false;

    private boolean button2 = false;

    private boolean rightZipDown = false;

    private boolean button3 = false;

    private boolean leftZipDown = false;

    private boolean button4 = false;

    public FPSTeleOp() {

    }

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        hook1Motor = hardwareMap.dcMotor.get("hook1");
        hook2Motor = hardwareMap.dcMotor.get("hook2");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftPusher = hardwareMap.servo.get("leftPusher");
        rightPusher = hardwareMap.servo.get("rightPusher");

        dumper = hardwareMap.servo.get("climberDump");

        pulley = hardwareMap.servo.get("pulley");

        leftZipline = hardwareMap.servo.get("leftZipline");
        rightZipline = hardwareMap.servo.get("rightZipline");
    }

    @Override
    public void loop() {
        float joy1Y = gamepad1.left_stick_y;
        float joy1X = gamepad1.left_stick_x;
        float joy2X = gamepad1.right_stick_x;
        float joy2Y = gamepad1.right_stick_y;
        float rightTrigger = gamepad2.right_trigger;
        float leftTrigger = gamepad2.left_trigger;

        float motorPower = joy1Y * 100; //Create variables for storing the motor power and the curve to be applied to the motors.
        float motorCurve = joy2X * 100;
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
            motorCurve *= 0.5;
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


        if (gamepad1.a) {
            if (button1 == false) {
                hookDown = !hookDown;
                button1 = true;
            }
        } else {
            button1 = false;
        }

        if (hookDown) {
            leftPusher.setPosition(0);
            rightPusher.setPosition(1);
        } else {
            leftPusher.setPosition(1);
            rightPusher.setPosition(0);
        }

        if (gamepad1.b) {
            if (button2 == false) {
                dumpDown = !dumpDown;
                button2 = true;
            }
        } else {
            button2 = false;
        }

        if (dumpDown) {
            dumper.setPosition(1);
        } else {
            dumper.setPosition(0);
        }


        if (gamepad1.x) {
            pulley.setPosition(1);
        } else if (gamepad1.y) {
            pulley.setPosition(0);
        } else {
            pulley.setPosition(0.5);
        }

        if (gamepad1.right_bumper) {
            hook1Motor.setPower(1);
            hook2Motor.setPower(1);
        } else if (gamepad1.left_bumper) {
            hook1Motor.setPower(-1);
            hook2Motor.setPower(-1);
        } else {
            hook1Motor.setPower(0);
            hook2Motor.setPower(0);
        }

        if (rightTrigger > 0.2) {
            if (button3 == false) {
                rightZipDown = !rightZipDown;
                button3 = true;
            }
        } else {
            button3 = false;
        }

        if (rightZipDown) {
            rightZipline.setPosition(1);
        } else {
            rightZipline.setPosition(0);
        }

        if (leftTrigger > 0.2) {
            if (button4 == false) {
                leftZipDown = !leftZipDown;
                button4 = true;
            }
        } else {
            button4 = false;
        }

        if (leftZipDown) {
            leftZipline.setPosition(0);
        } else {
            leftZipline.setPosition(1);
        }
    }
}
