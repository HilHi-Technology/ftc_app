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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Prototype Auto", group ="Autonomous")
public class PrototypeAuto extends EnhancedLinearOpMode {

    @Override
    public void startOpMode() {

        golf.setPosition(0.75);
        sleep(1000);
        golf.setPosition(0.60);
        sleep(1000);
        golf.setPosition(0.50);
        sleep(2000);

        while (opModeIsActive() && (color.red() < redCheck) && (color.blue() < blueCheck)) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }

        if (color.red() >= redCheck) {
            rightMotor.setPower(0.3);
            leftMotor.setPower(-0.3);
            sleep(300);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            sleep(1000);
            golf.setPosition(1);
            sleep(1000);

            rightMotor.setPower(-0.3);
            leftMotor.setPower(0.3);
            sleep(300);
            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(300);
        }
        else if (color.blue() >= blueCheck) {
            rightMotor.setPower(-0.3);
            leftMotor.setPower(0.3);
            sleep(300);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            sleep(1000);
            golf.setPosition(1);
            sleep(1000);

            rightMotor.setPower(0.3);
            leftMotor.setPower(-0.3);
            sleep(300);
            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(300);
        }

        // Blue far

        leftMotor.setPower(-0.3);
        rightMotor.setPower(0.3);
        sleep(200);
        leftMotor.setPower(0);
        rightMotor.setPower(0);


        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(5000);
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        sleep(300);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}