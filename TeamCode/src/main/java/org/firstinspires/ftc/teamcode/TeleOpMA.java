package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {
        initHardware2020();
        //initPID();

        waitForStart();

        //setupPIDParameters();

        while (opModeIsActive()) {
            drive();
            ringIntake();
            liftControl();
            ringLauncher();
            ringPush();
            getGeneralTelemetry();
        }

    }
}
