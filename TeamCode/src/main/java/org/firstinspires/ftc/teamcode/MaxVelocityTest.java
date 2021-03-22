package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MaxVelocityTest", group = "")
public class MaxVelocityTest extends CommonOpMode {
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        initHardware2020();

        waitForStart();

        while (opModeIsActive()) {
            currentVelocity = ringLauncherMotor.getVelocity();
            ringLauncherMotor.setPower(1);

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.update();
        }

    }
}