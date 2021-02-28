package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {
        initHardware();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            ringIntake();
            ringLauncherPrototypeMotor();
            ringLauncherPrototypeTrigger();
            getGeneralTelemetry();
        }

    }
}
