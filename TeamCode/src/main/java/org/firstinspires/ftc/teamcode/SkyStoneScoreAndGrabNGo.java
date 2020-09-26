package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "SkyStoneScoreAndGrabNGo", group = "")

public class SkyStoneScoreAndGrabNGo extends CommonOpMode {
    static final int SERVO_WAIT_TIME = 300;

    @Override
    public void runOpMode() {
        initHardware();

        allianceChooser();

        initPID();

        resetDriveWithoutEncoder();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (alliance == BLUE) {
                if (position == LEFT) {

                    leftactuator.setPosition(0.3);
                    rightactuator.setPosition(0.4);
                    strafeLeft(220);
                    sleep(100);
                    blueAutoDriveForwardAndStoneCheck();
                    stopDriveMotors();
                    sleep(50);
                    stopStoneDisposal();
                    pidPower = .6;
                    sleep(100);
                    stoneDisposal();
                    sleep(700);
                    stopStoneDisposal();
                    strafeRight(110);
                    sleep(100);
                    driveStraightBackward(300);
                    sleep(100);
                    driveBackwardsSlowlyToFoundation();
                    stopDriveMotors();
                    autoSuckIn();
                    sleep(2000);
                    stopStoneDisposal();
                    driveStraightForward(140);

                    break;

                }
                if (position == RIGHT) {

                    leftactuator.setPosition(0.3);
                    rightactuator.setPosition(0.4);
                    strafeLeft(220);
                    sleep(100);
                    blueAutoDriveForwardAndStoneCheck();
                    stopDriveMotors();
                    sleep(50);
                    stopStoneDisposal();
                    pidPower = .6;
                    sleep(100);
                    stoneDisposal();
                    sleep(700);
                    stopStoneDisposal();
                    strafeRight(110);
                    sleep(100);
                    driveStraightBackward(300);
                    sleep(100);
                    driveBackwardsSlowlyToFoundation();
                    stopDriveMotors();
                    autoSuckIn();
                    sleep(2000);
                    stopStoneDisposal();
                    driveStraightForward(140);

                    break;

                }

            }
            if (alliance == RED) {
                if (position == RIGHT) {

                    leftactuator.setPosition(0.3);
                    rightactuator.setPosition(0.4);
                    strafeRight(220);
                    sleep(100);
                    redAutoDriveForwardAndStoneCheck();
                    stopDriveMotors();
                    sleep(50);
                    stopStoneDisposal();
                    pidPower = .6;
                    sleep(100);
                    stoneDisposal();
                    sleep(700);
                    stopStoneDisposal();
                    strafeLeft(80);
                    sleep(100);
                    driveStraightBackward(300);
                    sleep(100);
                    driveBackwardsSlowlyToFoundation();
                    stopDriveMotors();
                    autoSuckIn();
                    sleep(2000);
                    stopStoneDisposal();
                    driveStraightForward(140);

                    break;
                }
                if (position == LEFT) {

                    leftactuator.setPosition(0.3);
                    rightactuator.setPosition(0.4);
                    strafeRight(220);
                    sleep(100);
                    redAutoDriveForwardAndStoneCheck();
                    stopDriveMotors();
                    sleep(50);
                    stopStoneDisposal();
                    pidPower = .6;
                    sleep(100);
                    stoneDisposal();
                    sleep(700);
                    stopStoneDisposal();
                    strafeLeft(80);
                    sleep(100);
                    driveStraightBackward(300);
                    sleep(100);
                    driveBackwardsSlowlyToFoundation();
                    stopDriveMotors();
                    autoSuckIn();
                    sleep(2000);
                    stopStoneDisposal();
                    driveStraightForward(140);

                    break;
                }
            }


        }
    }
}



