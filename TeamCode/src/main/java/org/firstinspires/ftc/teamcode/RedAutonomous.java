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

@Autonomous(name="Red Autonomous", group ="Autonomous")
public class RedAutonomous extends EnhancedLinearOpMode {

    @Override
    public void startOpMode() {

        encoderDrive(0.2, 300, 300, 1000);
        navXTurn(-40, 0.3, 1000);
        encoderDrive(0.2, 4750, 4750, 1000);
        navXTurn(-5, 0.3, 1000);
        vuforiaMove(0.1, 0.1, 100, 1000);

        pusher.setPosition(0);
        while (colorSensor.red() < 2 && colorSensor.blue() < 2) {
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Blue", colorSensor.blue());
            telemetry.update();
        }
        pusher.setPosition(0.5);

        if (colorSensor.red() >= 2) {
            pusher.setPosition(0);
            sleep(5000);
        } else if (colorSensor.blue() >= 2) {
            vuforiaMove(0.1, 0.1, -25, 1000);
            pusher.setPosition(0);
            sleep(5000);
        }

        pusher.setPosition(0.5);
    }
}
