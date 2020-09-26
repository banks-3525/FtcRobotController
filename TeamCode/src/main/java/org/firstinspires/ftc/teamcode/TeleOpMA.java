package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpMA", group = "")
public class TeleOpMA extends CommonOpMode {

    @Override
    public void runOpMode() {

        initHardware();
        armsResetAndRun();

        waitForStart();

        while (opModeIsActive()) {
            drive();
            suction();
            setSpeed();
            arms();
            //capstoneControl();
            capstoneGrabber();
            foundationGrabberOnePress();
            lineUpBlock();
            incrementDown();
            //incrementUp();
            getGeneralTelemetry();
            //pushOutBackwards();
            //actuatorControl();
            leftactuator.setPosition(0.3);
            rightactuator.setPosition(0.4);
            //actuators(0.7, -1);
            //sleep(20);
        }

    }
}
