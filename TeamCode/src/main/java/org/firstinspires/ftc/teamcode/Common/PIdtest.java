package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommonOpMode;

@Autonomous(name = "PIDtest", group = "")
public class PIdtest extends CommonOpMode {

    @Override
    public void runOpMode() {
        initHardware2020();
        initPID();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        setupPIDParameters();

        if (opModeIsActive()) {
            driveStraightForward(200);
        }


    }
}