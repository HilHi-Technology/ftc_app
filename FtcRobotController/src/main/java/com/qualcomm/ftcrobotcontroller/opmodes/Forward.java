package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorController;

import com.qualcomm.robotcore.util.Range;


import java.util.concurrent.ExecutionException;



//Created by Jonathan on 9/28/2015


public class Forward extends LinearOpMode {

    

DcMotor left;
    
DcMotor right;

 

Servo climberDump   


private int turn90 = 1230;

   

public Forward() {

    }

    

@Override
    
public void runOpMode() {

        

mapHardware();

        

left.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        
right.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        

newWaitForStart();

      

climberDump.setPosition(1);

goForward(5355, 0.4);

        

pause(2000);

        

turnRight(1095, 0.3);

        

pause(2000);

        

goForward(3300, 0.3);

        

pause(2000);

        

goForward(450, 0.1);

climberDump.setPosition(0);
    
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

        telemetry.addData("2", String.format("%d", right.getTargetPosition()));

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

        telemetry.addData("2", String.format("%d", right.getTargetPosition()));

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

        telemetry.addData("2", String.format("%d", right.getTargetPosition()));

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

        telemetry.addData("2", String.format("%d", right.getTargetPosition()));

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
climberDump = hardwareMap.servo.get("climberDump");
    
}
}

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