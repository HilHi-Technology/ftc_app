package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
public class MRRGBExample extends LinearOpMode {
  ColorSensor colorSensor;
  @Override
  public void runOpMode() throws InterruptedException {
    hardwareMap.logDevices();
    colorSensor = hardwareMap.colorSensor.get("colorSensor");
    colorSensor.enableLed(false);
    waitForStart();
    while (opModeIsActive()) {
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Blue ", colorSensor.blue());
    }
  }
}
