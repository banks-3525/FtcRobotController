package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "GrabNGoBBIT", group = "")
public class GrabNGoBBIT extends CommonOpMode {

    static final int SERVO_WAIT_TIME = 1000;

    @Override
    public void runOpMode() {
        initHardware();

        allianceChooser();

        waitForStart();

        while (opModeIsActive()) {

            // distance (in cm) to ticks = ((d/31.4)=r*1120)
            if (alliance == BLUE) {
                if (position == LEFT) {
                    //DriveX(-160, 1);
                    driveAuto(0, -1, 0, 1, -160);

                    //DriveY(155, 1);
                    driveAuto(-1, 0, 0, 1, -155);
                    //Grabber.setPosition(0);
                    sleep(SERVO_WAIT_TIME);


                    //DriveY(-169, 1);
                    driveAuto(-1, 0, 0, 1, -169);
                    //distanceDrive(160);
                    //Grabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    driveAuto(0,1,0,1,240);

                    break;
                }
                if (position == RIGHT) {
                    //DriveX(-160, 1);
                    driveAuto(0, -1, 0, 1, -160);

                    //DriveY(155, 1);
                    driveAuto(-1, 0, 0, 1, -155);
                    //Grabber.setPosition(0);
                    sleep(SERVO_WAIT_TIME);
                    //idle();

                    //DriveY(-169, 1);
                    driveAuto(-1, 0, 0, 1, -169);
                    //distanceDrive(160);
                    //Grabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);
                    //idle();

                    driveAuto(0,1,0,1,240);

                    break;
                }

            }
            if (alliance == RED) {
                if (position == LEFT) {
                    // DriveX(160, 1);
                    driveAuto(0, 1, 0, 1, 160);

                    //DriveY(155, 1);
                    driveAuto(-1, 0, 0, 1, -155);
                    //Grabber.setPosition(0);
                    sleep(SERVO_WAIT_TIME);
                    //idle();

                    //DriveY(-170, 1);
                    driveAuto(-1, 0, 0, 1, -169);
                    //distanceDrive(160);
                    //Grabber.setPosition(1);
                    sleep(CYCLE_MS);
                    //idle();

                    break;
                }
                if (position == RIGHT) {
                    //DriveX(160, 1);
                    driveAuto(0, 1, 0, 1, 160);

                    //DriveY(155, 1);
                    driveAuto(-1, 0, 0, 1, -155);
                    //Grabber.setPosition(0);
                    sleep(SERVO_WAIT_TIME);
                    //idle();

                    driveAuto(1, 0, 0, 1, 169);
                    //distanceDrive(160);
                    //Grabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);
                    //idle();

                    break;
                }
            }
        }
    }
}
