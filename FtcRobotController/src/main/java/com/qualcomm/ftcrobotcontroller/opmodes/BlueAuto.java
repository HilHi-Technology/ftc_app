package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class BlueAuto extends LinearOpMode {

    DcMotor left;
    DcMotor right;

    Servo climberDump;
    Servo leftPusher;
    Servo rightPusher;
    Servo pulley;
    Servo rightZipline;

    ColorSensor colorSensor;

    public BlueAuto() {
    }

    @Override
    public void runOpMode() {

        mapHardware();

        left.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        right.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        newWaitForStart();

        rightPusher.setPosition(1);
        leftPusher.setPosition(0);
        climberDump.setPosition(0);
        pulley.setPosition(0.5);
        rightZipline.setPosition(1);

        pause(8000);
   
        goForward(6000, 0.3);

        pause(1000);

        goBackward(225, 0.3);

        pause(1000);

        turnLeft(1235, 0.3);

        pause(1000);

        goForward(3400, 0.3);

        pause(1000);

        climberDump.setPosition(0.88);

        pause(1000);

        climberDump.setPosition(0);

        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());

        if(colorSensor.blue() > 1) {

            goForward(600, 0.1);

        }
        else {
            goBackward(1500, 0.3);

            pause(1000);

            turnRight(300, 0.3);

            pause(1000);

            goForward(1100, 0.3);

            pause(1000);

            turnLeft(325, 0.3);

            pause(1000);

            goForward(1000, 0.1);
        }
    }



    private void pause(int milliseconds) {
        try {
            sleep(milliseconds);
        } catch (Exception e) {
        }
    }


    private void goForward (int distance, double power) {
        right.setTargetPosition(distance);
        left.setTargetPosition(distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void goBackward(int distance, double power) {
        right.setTargetPosition(-distance);
        left.setTargetPosition(-distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void turnRight(int distance, double power) {
        right.setTargetPosition(-distance);
        left.setTargetPosition(distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void turnLeft(int distance, double power) {
        right.setTargetPosition(distance);
        left.setTargetPosition(-distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void newWaitForStart() {
        try {
            waitForStart();
        } catch (Exception e) {
        }
    }


    private void mapHardware() {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");
        right.setDirection(DcMotor.Direction.REVERSE);

        pulley = hardwareMap.servo.get("pulley");
        climberDump = hardwareMap.servo.get("climberDump");

        rightPusher = hardwareMap.servo.get("rightPusher");
        leftPusher = hardwareMap.servo.get("leftPusher");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        rightZipline = hardwareMap.servo.get("rightZipline");
    }
}
