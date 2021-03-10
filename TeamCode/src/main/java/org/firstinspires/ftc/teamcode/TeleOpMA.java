package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            long startTime = System.currentTimeMillis();

            if (drive == FIELD) {
                fieldCentricDrive();
            } else {
                robotCentricDrive();
            }

            ringIntake();
            setSpeed();
            //liftControl();
            ringLauncher();
            rpmTest();
            ringPush();
            resetAlignment();
            //wobbleArm();
            taunt();
            getGeneralTelemetry();
        }

    }
}
