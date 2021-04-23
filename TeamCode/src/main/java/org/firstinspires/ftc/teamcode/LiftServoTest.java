package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "LiftServoTest", group = "")
public class LiftServoTest extends CommonOpMode {

    @Override
    public void runOpMode() {
        initHardware2020();

        waitForStart();

        while (opModeIsActive()) {
            liftAngleServo.setPosition(0);
            telemetry.addData("Position", liftAngleServo.getPosition());
            telemetry.update();
        }

    }
}