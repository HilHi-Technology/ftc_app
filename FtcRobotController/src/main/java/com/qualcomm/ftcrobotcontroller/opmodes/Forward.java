package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.ExecutionException;


//Created by Jonathan on 9/28/2015

public class Forward extends LinearOpMode {

    DcMotor left;
    DcMotor right;

    public Forward() {

    }

    @Override
    public void runOpMode(){

        mapHardwareLeft();
        mapHardwareRight();

        newWaitForStart();

        goForward(1000);


    }

    private void goForward(int milliseconds) {
        left.setPower(1);
        right.setPower(1);

        try {
            sleep(milliseconds);
        } catch (Exception e) {

        }
        left.setPower(0);
        right.setPower(0);
    }

    private void goBackward(int milliseconds) {
        left.setPower(-1);
        right.setPower(-1);

        try {
            sleep(milliseconds);
        } catch (Exception e) {

        }
        left.setPower(0);
        right.setPower(0);
    }

    private void turnRight(int milliseconds) {
        left.setPower(1);
        right.setPower(-1);

        try {
            sleep(milliseconds);
        } catch (Exception e) {

        }
        left.setPower(0);
        right.setPower(0);
    }

    private void turnLeft(int milliseconds) {
        left.setPower(-1);
        right.setPower(1);

        try {
            sleep(milliseconds);
        } catch (Exception e) {

        }
        left.setPower(0);
        right.setPower(0);
    }

    private void newWaitForStart() {
        try {
            waitForStart();
        } catch (Exception e) {

        }
    }

    private void mapHardwareLeft() {
        left = hardwareMap.dcMotor.get("left");
    }

    private void mapHardwareRight() {
        right = hardwareMap.dcMotor.get("right");
    }
}