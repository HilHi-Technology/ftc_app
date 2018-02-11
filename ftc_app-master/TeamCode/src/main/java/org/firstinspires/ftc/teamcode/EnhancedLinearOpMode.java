
package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class EnhancedLinearOpMode extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public ColorSensor color;
    public Servo golf;

    public int redCheck = 1;
    public int blueCheck = 1;

    @Override
    public void runOpMode() {
        runtime.reset();

        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");

        golf = hardwareMap.servo.get("golf");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        color = hardwareMap.colorSensor.get("color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        color.enableLed(true);
        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Init");
        telemetry.update();


        while (!opModeIsActive()) {
            telemetry.addData("Time Since Init", runtime.seconds());
            telemetry.update();
        }
        startOpMode();
    }

    public abstract void startOpMode();

    }
