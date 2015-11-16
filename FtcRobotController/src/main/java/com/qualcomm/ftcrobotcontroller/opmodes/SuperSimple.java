package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jonathan on 9/28/2015.
 */
public class SuperSimple extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    public SuperSimple() {

    }

    @Override
    public void init()
    {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.left_stick_y);
    }
}
