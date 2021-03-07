package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {
        initHardware2020();
        initPID();

        driveChooser();

        waitForStart();

        setupPIDParameters();

        while (opModeIsActive()) {
            if (drive == FIELD) {
                fieldCentricDrive();
            } else {
                robotCentricDrive();
            }

            ringIntake();
            liftControl();
            ringLauncher();
            ringPush();
            resetAlignment();
            //wobbleArm();
            taunt();
            getGeneralTelemetry();
        }

    }
}
