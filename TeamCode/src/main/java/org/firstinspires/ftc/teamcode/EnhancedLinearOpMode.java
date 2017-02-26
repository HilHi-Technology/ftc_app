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

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Lego on 2/3/2017.
 */

public abstract class EnhancedLinearOpMode extends LinearOpMode {

    public final float MIN_POWER_STRAIGHT = 0.25f;
    public final float MIN_POWER_TURN = 0.35f;
    public final int STRAIGHT_START_SLOWDOWN = 500;
    public final float TURN_START_SLOWDOWN = 60f;

    public ElapsedTime runtime = new ElapsedTime();
    public AHRS navx_device;

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor sweep = null;
    public DcMotor spin1 = null;
    public DcMotor spin2 = null;
    public DcMotor arm = null;
    public Servo pushLeft = null;
    public Servo pushRight = null;
    public ColorSensor colorSensor;
    public Servo liftLock;

    public VuforiaLocalizer vuforia;

    private static final String tag = "Autonomous";

    @Override
    public void runOpMode() {
        runtime.reset();

        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");
        sweep = hardwareMap.dcMotor.get("sweep");
        spin1 = hardwareMap.dcMotor.get("spin1");
        spin2 = hardwareMap.dcMotor.get("spin2");
        pushLeft = hardwareMap.servo.get("pushLeft");
        pushRight = hardwareMap.servo.get("pushRight");
        arm = hardwareMap.dcMotor.get("arm");
        liftLock = hardwareMap.servo.get("lock");

        pushLeft.setPosition(0.5);
        pushRight.setPosition(0.5);
        liftLock.setPosition(0);

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
        startOpMode();
    }

    public abstract void startOpMode();

    /*
    This algorithm controls the trajectory of the robot by checking the yaw from the navx and doing a
    proportional correction to the motor power in order to return the robot to a straight line when it deviates.
    The further, the robot is from that line, the larger the power correction.
    Min and Max motor speeds are enforced for optimal efficiency.
     */

    public void encoderYawStraight(float speed, float ticks, float adjustRate, float sleepTime) {
        if (opModeIsActive()) {

            float startYaw = navx_device.getYaw();
            if (speed > 0.8) {
                speed = 0.8f;
            }
            int startPos = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

            rightMotor.setPower(speed);
            leftMotor.setPower(speed);

            int avgEncoder = 0;
            float slowdown = 0;

            while (opModeIsActive() && Math.abs(avgEncoder - startPos) < Math.abs(ticks)) {
                float percentCompleted = (avgEncoder - startPos - (ticks - (STRAIGHT_START_SLOWDOWN * Math.signum(ticks)))) / STRAIGHT_START_SLOWDOWN;
                if (Math.abs(avgEncoder - startPos) > Math.abs(ticks) - STRAIGHT_START_SLOWDOWN) {
                    slowdown = percentCompleted * (Math.abs(speed) - MIN_POWER_STRAIGHT);
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

    /*
    The NavX sensor module that contains a gyroscope, compass, and accelerometer is used in this
    algorithm to accurately turn relative to a set zero yaw at the beginning of both autonomous programs.
    Careful calculation is performed to find the amount to turn using a comparison algorithm taking turn
    direction into account with signum. Turning power is calculated proportionally using a Min and Max
    power for efficiency and tuning the power between those bounds based on remaining turn amount.
    */

    public void navXTurn(float initialTarget, float maxPower, float turnSleepTime) {
        if (navx_device.isConnected()) {
            telemetry.addData("NavX is Connected?", "Yes");
            if (navx_device.isMagnetometerCalibrated()) {
                telemetry.addData("Magnometer is Calibrated?", "Yes");
                telemetry.addData("Zero Yaw?", "Yes");
                telemetry.update();
                sleep(500);

                float initialYaw = navx_device.getYaw();
                float totalDistance = initialYaw - initialTarget; //30

                /*float altInitialYaw = initialYaw == 0 ? 180 : (180 - Math.abs(initialYaw)) * Math.signum(initialYaw); //-120
                float altInitialTarget = initialTarget == 0 ? 180 : (180 - Math.abs(initialTarget)) * Math.signum(initialTarget); //-90
                float altTotalDistance = altInitialYaw - altInitialTarget; //-30

                telemetry.addLine("totalDistance:altTotalDistance " + String.format("%.2f : %.2f",totalDistance, altTotalDistance));
                telemetry.update();

                boolean altTurn = false;*/
                float distanceRemaining = totalDistance;
                /*if (Math.abs(totalDistance) > Math.abs(altTotalDistance)) {
                    altTurn = true;
                    totalDistance = altTotalDistance;
                    distanceRemaining = altTotalDistance;
                }*/

                float initialSign = Math.signum(distanceRemaining);
                //float altTurnSwitchDirection = 1;
                float slowdown = 0;
                float percentComplete = 0;
                float lastYaw = navx_device.getYaw();
                float currentYaw = lastYaw;

                /*if (altTurn) {
                    altTurnSwitchDirection = -1;
                }*/

                while (initialSign == Math.signum(distanceRemaining)) {
                    percentComplete = 1 - (Math.abs(distanceRemaining) / TURN_START_SLOWDOWN);
                    if (Math.abs(distanceRemaining) < TURN_START_SLOWDOWN) {
                        slowdown = percentComplete * (maxPower - MIN_POWER_TURN);
                    } else {
                        slowdown = 0;
                    }

                    leftMotor.setPower(/*altTurnSwitchDirection * */(-maxPower - slowdown) * Math.signum(distanceRemaining));
                    rightMotor.setPower(/*altTurnSwitchDirection * */(maxPower - slowdown) * Math.signum(distanceRemaining));

                    /*if (altTurn) {
                    float altCurrentYaw = navx_device.getYaw() == 0 ? 180 : (180 * Math.signum(navx_device.getYaw())) - navx_device.getYaw();
                    distanceRemaining = altCurrentYaw - altInitialTarget;
                } else { */

                        currentYaw = navx_device.getYaw();
                        distanceRemaining = (currentYaw + (currentYaw - lastYaw)) - initialTarget;
                        telemetry.addData("delta", currentYaw-lastYaw);
                        telemetry.update();
                        lastYaw = currentYaw;
                    //}
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

    /*
    The final algorithm we use is vuforiaMove. The name is less
    self-descriptive than the other functions but vuforiaMove drives straight
    until the phone’s camera detects the image beneath the beacon.
    The detection algorithm used is based off of Vuforia code.
     */

    public void vuforiaMove(float firstSpeed, float secondSpeed, float position, int imageNumber, int wait) {
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        beacons.activate();

        sleep(1000);

        leftMotor.setPower(firstSpeed);
        rightMotor.setPower(firstSpeed);

        telemetry.addData("Status", "Things");
        telemetry.update();

        List<Integer> foundImages = new ArrayList<Integer>();

        while (opModeIsActive() && foundImages.size() < imageNumber) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();
                if (pose != null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beacon.getName() + "-Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beacon.getName() + "-Degrees", degreesToTurn);
                    int beaconIndex = beacons.indexOf(beacon);
                    if (!foundImages.contains(beaconIndex)) {
                        foundImages.add(beaconIndex);
                    }
                }
            }
            telemetry.update();
        }
        int reverseDirection = 1;
        if (secondSpeed < 0) {
            reverseDirection = -1;
        }
        while (opModeIsActive() && reverseDirection * ((VuforiaTrackableDefaultListener) beacons.get(foundImages.get(imageNumber - 1)).getListener()).getPose().getTranslation().get(1) > reverseDirection * position) {
            leftMotor.setPower(secondSpeed);
            rightMotor.setPower(secondSpeed);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        beacons.deactivate();

        sleep(wait);
    }

    public void beaconPressLeft(int redCheck, int blueCheck, int pressAmount, int distanceRedBlue) {
        pushLeft.setPosition(0);
        while (opModeIsActive() && colorSensor.red() < redCheck && colorSensor.blue() < blueCheck) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
        pushLeft.setPosition(0.5);
        if (colorSensor.red() >= redCheck) {
            pushLeft.setPosition(0);
            sleep(pressAmount);
        } else if (colorSensor.blue() >= blueCheck) {
            vuforiaMove(0.1f, 0.1f, distanceRedBlue, 1, 1000);
            pushLeft.setPosition(0);
            sleep(pressAmount);
        }
        pushLeft.setPosition(0.5);
    }

    public void beaconPressRight(int redCheck, int blueCheck, int pressAmount, int distanceRedBlue) {
        pushRight.setPosition(0);
        while (opModeIsActive() && colorSensor.red() < redCheck && colorSensor.blue() < blueCheck) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
        pushRight.setPosition(0.5);
        if (colorSensor.red() >= redCheck) {
            pushRight.setPosition(0);
            sleep(pressAmount);
        } else if (colorSensor.blue() >= blueCheck) {
            vuforiaMove(0.1f, 0.1f, distanceRedBlue, 1, 1000);
            pushRight.setPosition(0);
            sleep(pressAmount);
        }
        pushRight.setPosition(0.5);
    }

    /*
    The ballShootAuto algorithm is our latest addition, it was added to provide control
    to the problem of inconsistent ball shoots that we made based on power alone.
    This algorithm spins up the flywheels at a slightly low best guess power, then
    tunes the power based on computed RPM compared to our target ball throwing RPM.
    This ensures that balls are thrown at the target RPM regardless of battery charge.
    */

    public void ballShootAuto(int spinUp, float constantIncrease, int targetRPM) {
        float newLeftEncoder = 0;
        float oldLeftEncoder = 0;
        float newRightEncoder = 0;
        float oldRightEncoder = 0;
        float RPM = 0;
        float currentPower = 0.5f;

        spin2.setPower(0.5);
        spin1.setPower(0.5);
        sleep(spinUp);

        while (opModeIsActive() && RPM < targetRPM) {

            newLeftEncoder = spin1.getCurrentPosition();
            newRightEncoder = spin2.getCurrentPosition();

            runtime.reset();

            spin2.setPower(currentPower);
            spin1.setPower(currentPower);

            sleep(100);

            currentPower = currentPower + constantIncrease;
            RPM = (((newLeftEncoder - oldLeftEncoder) + (newRightEncoder - oldRightEncoder)) / 2) / (int)runtime.seconds();

            oldLeftEncoder = newLeftEncoder;
            oldRightEncoder = newRightEncoder;
        }

        sweep.setPower(1);
        sleep(3000);
        while(spin1.getPower() > 1) {
            sweep.setPower(sweep.getPower() * 0.98);
            spin1.setPower(spin1.getPower() * 0.98);
            spin2.setPower(spin2.getPower() * 0.98);
        }
        sweep.setPower(0);
        spin2.setPower(0);
        spin1.setPower(0);

    }

}
