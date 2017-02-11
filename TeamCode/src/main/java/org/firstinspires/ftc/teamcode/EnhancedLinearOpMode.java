package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Lego on 2/3/2017.
 */

public abstract class EnhancedLinearOpMode extends LinearOpMode {

    public final double MIN_POWER_STRAIGHT = 0.25;
    public final double MIN_POWER_TURN = 0.4;
    public final int TICKS_START_SLOWDOWN = 500;

    public ElapsedTime runtime = new ElapsedTime();
    public AHRS navx_device;

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public Servo pusher = null;
    public ColorSensor colorSensor;

    public VuforiaLocalizer vuforia;

    private static final String tag = "Autonomous";

    @Override
    public void runOpMode() {
        runtime.reset();

        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");

        pusher = hardwareMap.servo.get("push");
        pusher.setPosition(0.5);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"), 0, AHRS.DeviceDataType.kProcessedData);

        colorSensor = hardwareMap.colorSensor.get("color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        colorSensor.enableLed(false);
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Init");
        telemetry.update();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVpZvhj/////AAAAGS4AanX+yU5toBr4URhwzTlcjMcTgpcgbIoseTYKxBoYxNobI6A5VxrfyfBiSEpEyk0RA0ynCqNIQbnpYg20ufD+Fg1eHR6sB+6BalZhvf6vnigBboPowdl+7k64fnboXbpzez157m7B6Yiegz9uygCQJZiDMzwcyz753xOxnKPh4LGtTaY2ErXJn0e46tNinSqyF5O6PiHyooUQPxleWWbqZ9ygGXspfCy3AqivZfw6OJn5L2l6He3JX89Kxprpi/EMhTYT5NXXzneIKYwjNaf4L0UShuZI3DgzLIx3+2QVIRaAP1X5iuWwMQxrj5BMRnvyRyZrTuGZNOTdPKc28MWTtyyOjwMf9yyQFcjWQzXF";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        boolean hasZeroed = false;

        while (!opModeIsActive()) {
            telemetry.addData("Time Since Init", runtime.seconds());
            telemetry.update();
            if (runtime.seconds() > 25 && !hasZeroed) {
                hasZeroed = true;
                navx_device.zeroYaw();
            }
        }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        startOpMode();
    }

    public abstract void startOpMode();

    public void encoderDrive(double speed, double leftTicks, double rightTicks, double sleepTime) {
        if (opModeIsActive()) {

            leftTicks = -leftTicks;
            rightTicks = -rightTicks;

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);

            leftMotor.setTargetPosition((int) leftTicks);
            rightMotor.setTargetPosition((int) rightTicks);

            sleep(250);

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep(250);

            if (leftTicks == 0) {
                leftMotor.setPower(0);
                rightMotor.setPower(Math.abs(speed));
            } else if (rightTicks == 0) {
                leftMotor.setPower(Math.abs(speed));
                rightMotor.setPower(Math.abs(0));
            } else if (leftTicks > rightTicks) {
                leftMotor.setPower(Math.abs(speed));
                rightMotor.setPower(Math.abs((rightTicks / leftTicks) * speed));
            } else if (rightTicks > leftTicks) {
                leftMotor.setPower(Math.abs((leftTicks / rightTicks) * speed));
                rightMotor.setPower(Math.abs(speed));
            } else if (rightTicks == leftTicks) {
                rightMotor.setPower(speed);
                leftMotor.setPower(speed);
            }

            if (rightTicks == 0) {
                while (opModeIsActive() && (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(leftMotor.getTargetPosition()))) {
                    telemetry.addData("Path1", "Running to %7d :%7d", (int) leftTicks, (int) rightTicks);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (leftTicks == 0) {
                while (opModeIsActive() && (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(rightMotor.getTargetPosition()))) {
                    telemetry.addData("Path1", "Running to %7d :%7d", (int) leftTicks, (int) rightTicks);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                    telemetry.update();
                }
            } else {
                while (opModeIsActive() && (Math.abs(leftMotor.getCurrentPosition()) < Math.abs(leftMotor.getTargetPosition())) && (Math.abs(rightMotor.getCurrentPosition()) < Math.abs(rightMotor.getTargetPosition()))) {
                    telemetry.addData("Path1", "Running to %7d :%7d", (int) leftTicks, (int) rightTicks);
                    telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                    telemetry.update();
                    Log.i(tag, "Running to " + leftTicks + " :" + rightTicks);
                    Log.i(tag, "Running at " + leftMotor.getCurrentPosition() + " :" + rightMotor.getCurrentPosition());

                }
            }

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            sleep(250);

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            sleep((int) sleepTime);
        }
    }

    public void encoderYawStraight(double speed, double ticks, double adjustRate, double sleepTime) {
        if (opModeIsActive()) {

            float startYaw = navx_device.getYaw();
            if (speed > 0.8) {
                speed = 0.8;
            }
            int startPos = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

            rightMotor.setPower(speed);
            leftMotor.setPower(speed);

            int avgEncoder = 0;
            double slowdown = 0;

            while (opModeIsActive() && avgEncoder - startPos < ticks) {
                double percentCompleted = (avgEncoder - startPos - (ticks - TICKS_START_SLOWDOWN)) / TICKS_START_SLOWDOWN;
                if (percentCompleted >= 0) {
                    slowdown = percentCompleted * (speed - MIN_POWER_STRAIGHT);
                } else {
                    slowdown = 0;
                }
                float yawOffset = startYaw - navx_device.getYaw();
                leftMotor.setPower((speed - slowdown) + (yawOffset * adjustRate));
                rightMotor.setPower((speed - slowdown) - (yawOffset * adjustRate));

                avgEncoder = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

                telemetry.addData("Path1", "Running to %7d", avgEncoder - startPos);
                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "Running at speed %7s : %7s", leftMotor.getPower(), rightMotor.getPower());
                telemetry.update();
            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            sleep((int) sleepTime);
        }
    }

    public void navXTurn(float initialTarget, double maxPower, double turnSleepTime) {
        if (navx_device.isConnected()) {
            telemetry.addData("NavX is Connected?", "Yes");
            if (navx_device.isMagnetometerCalibrated()) {
                telemetry.addData("Magnometer is Calibrated?", "Yes");
                telemetry.addData("Zero Yaw?", "Yes");
                telemetry.update();
                sleep(500);

                float initialYaw = navx_device.getYaw();
                float totalDistance = initialYaw - initialTarget;

                float altInitialYaw = initialYaw == 0 ? 180 : (180 * Math.signum(initialYaw)) - initialYaw;
                float altInitialTarget = initialTarget == 0 ? 180 : (180 * Math.signum(initialTarget)) - initialTarget;
                float altTotalDistance = altInitialYaw - altInitialTarget;

                telemetry.addLine("totalDistance:altTotalDistance " + String.format("%.2f : %.2f",totalDistance, altTotalDistance));
                telemetry.update();

                boolean altTurn = false;
                float distanceRemaining = totalDistance;
                if (Math.abs(totalDistance) > Math.abs(altTotalDistance)) {
                    altTurn = true;
                    totalDistance = altTotalDistance;
                    distanceRemaining = altTotalDistance;
                }

                float initialSign = Math.signum(distanceRemaining);

                while (initialSign == Math.signum(distanceRemaining)) {
                    float powerRatio = ((totalDistance - (totalDistance - distanceRemaining)) / Math.abs(totalDistance)) * (float)maxPower;
                    if (altTurn) {
                        powerRatio = -powerRatio;
                    }
                    if (Math.abs(powerRatio) < MIN_POWER_TURN) {
                        leftMotor.setPower(-MIN_POWER_TURN * Math.signum(powerRatio));
                        rightMotor.setPower(MIN_POWER_TURN * Math.signum(powerRatio));
                    } else {
                        leftMotor.setPower(-powerRatio);
                        rightMotor.setPower(powerRatio);
                    }

                    telemetry.addLine("distanceRemaining:navx_getyaw:powerRatio " + String.format("%.2f : %.2f",distanceRemaining, navx_device.getYaw(), powerRatio));
                    telemetry.update();

                    if (altTurn) {
                        float altCurrentYaw = navx_device.getYaw() == 0 ? 180 : (180 * Math.signum(navx_device.getYaw())) - navx_device.getYaw();
                        distanceRemaining = altCurrentYaw - altInitialTarget;
                    } else {
                        distanceRemaining = navx_device.getYaw() - initialTarget;
                    }
                }

                leftMotor.setPower(0);
                rightMotor.setPower(0);
            } else {
                telemetry.addData("Magnometer is Calibrated?", "No");
                telemetry.update();
                sleep(500);
            }
        } else {
            telemetry.addData("NavX is Connected?", "No");
            telemetry.update();
            sleep(500);
        }
        sleep((int) turnSleepTime);
    }

    public void vuforiaMove(double firstSpeed, double secondSpeed, float position, int wait) {
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        beacons.activate();

        sleep(1000);

        VuforiaTrackable firstImage = null;

        leftMotor.setPower(firstSpeed);
        rightMotor.setPower(firstSpeed);

        telemetry.addData("Status", "Things");
        telemetry.update();

        while (opModeIsActive() && firstImage == null) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacon.getName() + "-Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beacon.getName() + "-Degrees", degreesToTurn);
                    firstImage = beacon;
                }
            }
            telemetry.update();
        }
        int reverseDirection = 1;
        if (secondSpeed < 0) {
            reverseDirection = -1;
        }
        while (reverseDirection * ((VuforiaTrackableDefaultListener) firstImage.getListener()).getPose().getTranslation().get(1) > reverseDirection * position) {
            leftMotor.setPower(secondSpeed);
            rightMotor.setPower(secondSpeed);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        beacons.deactivate();

        sleep(wait);
    }

    public void beaconPress(int redCheck, int blueCheck, int pressAmount, int distanceRedBlue) {
        pusher.setPosition(0);
        while (colorSensor.red() < redCheck && colorSensor.blue() < blueCheck) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
        pusher.setPosition(0.5);
        if (colorSensor.red() >= redCheck) {
            pusher.setPosition(0);
            sleep(pressAmount);
        } else if (colorSensor.blue() >= blueCheck) {
            vuforiaMove(0.1, 0.1, distanceRedBlue, 1000);
            pusher.setPosition(0);
            sleep(pressAmount);
        }
        pusher.setPosition(0.5);
    }

}
