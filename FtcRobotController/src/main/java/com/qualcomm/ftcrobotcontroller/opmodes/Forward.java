package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//Created by Jonathan on 9/28/2015

public class Forward extends OpMode {

    DcMotor forward;

    public Forward() {

    }

    @Override
    public void init() {
        forward = hardwareMap.dcMotor.get("forward");
    }

    @Override
    public void loop() {
        forward.setPower(1);
    }
}