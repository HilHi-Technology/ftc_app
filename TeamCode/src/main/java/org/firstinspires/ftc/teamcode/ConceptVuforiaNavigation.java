/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.vuforia.HINT;
import com.vuforia.Vuforia;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.kauailabs.navx.ftc.AHRS;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="Concept: Vuforia Navigation", group ="Concept")
public class ConceptVuforiaNavigation extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AHRS navx_device;

    private final double MIN_POWER = 0.4;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"), 0, AHRS.DeviceDataType.kProcessedData);

        colorSensor = hardwareMap.colorSensor.get("color");
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);
        colorSensor.enableLed(false);
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(250);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Init");
        telemetry.update();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();

        navx_device.zeroYaw();

        encoderDrive(0.4, 300, 300, 1000);
        navXTurn(-40, 0.5, 1000);
        encoderDrive(0.4, 4100, 4100, 1000);
        sleep(10000);
        navXOneTurn(0, 0.5, 1000);
        vuforiaMove(0.2, 0.2, 0, 1000);

        while (true) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public void encoderDrive(double speed, double leftTicks, double rightTicks, double sleepTime) {
        if (opModeIsActive()) {

            leftTicks = -leftTicks;
            rightTicks = -rightTicks;

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(250);

            leftMotor.setTargetPosition((int) leftTicks);
            rightMotor.setTargetPosition((int) rightTicks);

            sleep(250);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
                }
            }

            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            sleep((int) sleepTime);
        }
    }

    public void navXTurn(float initialTarget, double maxPower, double turnSleepTime) {
        //Disable encoders, so navX can control without influence
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Sleep because encoders need it after mode change
        sleep(250);
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
                    if (Math.abs(powerRatio) < MIN_POWER) {
                        leftMotor.setPower(MIN_POWER * Math.signum(powerRatio));
                        rightMotor.setPower(-MIN_POWER * Math.signum(powerRatio));
                    } else {
                        leftMotor.setPower(powerRatio);
                        rightMotor.setPower(-powerRatio);
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

    public void navXOneTurn(float initialTarget, double maxPower, double turnSleepTime) {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(250);
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
                    telemetry.addLine("power " + String.format("%.2f", powerRatio * (powerRatio < 0f ? 0f : 1f)));
                    telemetry.update();
                    if (Math.abs(powerRatio) < MIN_POWER) {
                        leftMotor.setPower(MIN_POWER * Math.signum(powerRatio) * (powerRatio < 0f ? 0f : 1f));
                        rightMotor.setPower(-MIN_POWER * Math.signum(powerRatio) * (powerRatio > 0f ? 0f : 1f));
                    } else {
                        leftMotor.setPower(powerRatio * (powerRatio < 0f ? 0f : 1f));
                        rightMotor.setPower(-powerRatio * (powerRatio > 0f ? 0f : 1f));
                    }

                    //telemetry.addLine("distanceRemaining:navx_getyaw:powerRatio " + String.format("%.2f : %.2f",distanceRemaining, navx_device.getYaw(), powerRatio));
                    //telemetry.update();

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

        firstSpeed = -firstSpeed;
        secondSpeed = -secondSpeed;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AVpZvhj/////AAAAGS4AanX+yU5toBr4URhwzTlcjMcTgpcgbIoseTYKxBoYxNobI6A5VxrfyfBiSEpEyk0RA0ynCqNIQbnpYg20ufD+Fg1eHR6sB+6BalZhvf6vnigBboPowdl+7k64fnboXbpzez157m7B6Yiegz9uygCQJZiDMzwcyz753xOxnKPh4LGtTaY2ErXJn0e46tNinSqyF5O6PiHyooUQPxleWWbqZ9ygGXspfCy3AqivZfw6OJn5L2l6He3JX89Kxprpi/EMhTYT5NXXzneIKYwjNaf4L0UShuZI3DgzLIx3+2QVIRaAP1X5iuWwMQxrj5BMRnvyRyZrTuGZNOTdPKc28MWTtyyOjwMf9yyQFcjWQzXF";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        beacons.activate();

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
        while (((VuforiaTrackableDefaultListener) firstImage.getListener()).getPose().getTranslation().get(1) > position) {
            leftMotor.setPower(secondSpeed);
            rightMotor.setPower(secondSpeed);
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        sleep(wait);
    }
}
