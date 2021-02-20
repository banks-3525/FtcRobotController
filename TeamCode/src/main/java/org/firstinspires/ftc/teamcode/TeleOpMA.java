package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {

        initTestHardware2020();


        waitForStart();

        while (opModeIsActive()) {
            ringIntake();
            getGeneralTelemetry();
        }

    }
}
