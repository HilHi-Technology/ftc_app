package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class RedAuto extends LinearOpMode {

    DcMotor left;
    DcMotor right;

    Servo climberDump;
    Servo leftPusher;
    Servo rightPusher;
    Servo pulley;
    Servo rightZipline;

    ColorSensor colorSensor;

    public RedAuto() {
    }

    private void pause(int milliseconds) {
        try {
            sleep(milliseconds);
        } catch (Exception e) {

        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        mapHardware();

        left.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        right.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        newWaitForStart();

        colorSensor.enableLed(false);
        leftPusher.setPosition(0);
        rightPusher.setPosition(1);
        climberDump.setPosition(0);
        pulley.setPosition(0.5);
        rightZipline.setPosition(1);

        pause(8000);

        goForward(6000, 0.3);

        pause(1000);

        goBackward(225, 0.3);

        pause(1000);

        turnRight(1235, 0.3);

        pause(1000);

        goForward(3400, 0.3);

        pause(1000);

        climberDump.setPosition(0.88);

        pause(1000);

        climberDump.setPosition(0);

        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());

        if(colorSensor.blue() > 1) {
            goBackward(1800, 0.3);

            pause(1000);

            turnLeft(300, 0.3);

            pause(1000);

            goForward(1100, 0.3);

            pause(1000);

            turnRight(325, 0.3);

            pause(1000);

            goForward(1000, 0.1);
        }
        else {

            goForward(600, 0.1);

        }
    }


    private void goForward (int distance, double power) throws InterruptedException {
        right.setTargetPosition(distance);
        left.setTargetPosition(distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
            waitOneFullHardwareCycle();
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void goBackward(int distance, double power) throws InterruptedException {
        right.setTargetPosition(-distance);
        left.setTargetPosition(-distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
            waitOneFullHardwareCycle();
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void turnRight(int distance, double power) throws InterruptedException {
        right.setTargetPosition(-distance);
        left.setTargetPosition(distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
            waitOneFullHardwareCycle();
        }

        left.setPower(0);
        right.setPower(0);

        left.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        right.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }


    private void turnLeft(int distance, double power) throws InterruptedException {
        right.setTargetPosition(distance);
        left.setTargetPosition(-distance);

        left.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);

        while (Math.abs(right.getCurrentPosition()) < distance) {
            waitOneFullHardwareCycle();
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

        leftPusher = hardwareMap.servo.get("leftPusher");
        rightPusher = hardwareMap.servo.get("rightPusher");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        rightZipline = hardwareMap.servo.get("rightZipline");
    }
}
