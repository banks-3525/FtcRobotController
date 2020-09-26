package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@Autonomous(name = "GrabNGo", group = "")
public class GrabNGo extends CommonOpMode {
    static final int SERVO_WAIT_TIME = 300;

    @Override
    public void runOpMode() {
        initHardware();

        allianceChooser();

        initPID();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();

        setupPIDParameters();

        //rightTurn(.5);
        //driveStraightBackward(200);
        //sleep(1000);
        //driveStraightForward(200);
        //leftTurn(.5);
        //leftTurn(90);
        //rotate(90, 1);
       /* strafeRight(100);
        sleep(250);
        strafeLeft(100);
        sleep(250);
        driveStraightBackward(100);
        sleep(250);
        driveStraightForward(100);
        sleep(250);
        rotate(-90, 0.5);*/


        // distance (in cm) to ticks = ((d/31.4)=r*1120)
            if (alliance == BLUE) {
                if (position == LEFT) {
                    //driveAuto(0, 1, 0, 1, 60);
                    //waitForGamePadA();
                    strafeRight(139);

                    //driveAuto(1, 0, 0, 1, 75);
                    //waitForGamePadA();
                    pidPower = .3;
                    driveStraightBackward(180);
                    sleep(SERVO_WAIT_TIME);
                    leftFoundationGrabber.setPosition(0); // puts foundation grabbers down
                    rightFoundationGrabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    //driveAuto(-1, 0, 0, 1, -80);
                    //waitForGamePadA();
                    pidPower = .6;
                    driveStraightForward(160);
                    sleep(SERVO_WAIT_TIME);
                    //rightTurn(.3, 2202);
                    //sleep(SERVO_WAIT_TIME);
                    //strafeRight(10);
                    //driveStraightBackward(20);
                    //sleep(SERVO_WAIT_TIME);
                    //strafeRight(42);
                    //sleep(SERVO_WAIT_TIME);

                    //waitForGamePadA();
                    leftFoundationGrabber.setPosition(.6); // puts foundation grabbers up
                    rightFoundationGrabber.setPosition(.4);
                    sleep(SERVO_WAIT_TIME);

                    //waitForGamePadA();
                    strafeLeft(145);

                   // waitForGamePadA();
                    driveStraightBackward(130);

                    strafeLeft(80);

                }
                if (position == RIGHT) {

                   // driveAuto(0, 1, 0, 1, 60);
                    strafeRight(139);

                    //driveAuto(1, 0, 0, .25, 75);
                    pidPower = .3;
                    driveStraightBackward(180);
                    sleep(SERVO_WAIT_TIME);
                    leftFoundationGrabber.setPosition(0); // puts foundation grabbers down
                    rightFoundationGrabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    //driveAuto(-1, 0, 0, .25, -80);
                    pidPower = .6;
                    driveStraightForward(150);
                    sleep(SERVO_WAIT_TIME);

                   // waitForGamePadA();
                    leftFoundationGrabber.setPosition(.6); // puts foundation grabbers up
                    rightFoundationGrabber.setPosition(.4);
                    sleep(SERVO_WAIT_TIME);

                    //waitForGamePadA();
                    strafeLeft(240);


                }

            }
            if (alliance == RED) {
                if (position == RIGHT) {

                    strafeLeft(139);

                    //driveAuto(1, 0, 0, 1, 75);
                    //waitForGamePadA();
                    pidPower = .3;
                    driveStraightBackward(180);
                    sleep(SERVO_WAIT_TIME);
                    leftFoundationGrabber.setPosition(0); // puts foundation grabbers down
                    rightFoundationGrabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    //driveAuto(-1, 0, 0, 1, -80);
                    //waitForGamePadA();
                    pidPower = .6;
                    driveStraightForward(170);
                    sleep(SERVO_WAIT_TIME);
                    //rightTurn(.3, 2202);
                    //sleep(SERVO_WAIT_TIME);
                    //strafeRight(10);
                    //driveStraightBackward(20);
                    //sleep(SERVO_WAIT_TIME);
                    //strafeRight(42);
                    //sleep(SERVO_WAIT_TIME);

                    //waitForGamePadA();
                    leftFoundationGrabber.setPosition(.6); // puts foundation grabbers up
                    rightFoundationGrabber.setPosition(.4);
                    sleep(SERVO_WAIT_TIME);

                    //waitForGamePadA();
                    strafeRight(145);

                    // waitForGamePadA();
                    driveStraightBackward(120);

                    strafeRight(100);
                }
                if (position == LEFT) {

                    // driveAuto(0, 1, 0, 1, 60);
                    strafeLeft(139);

                    //driveAuto(1, 0, 0, .25, 75);
                    pidPower = .3;
                    driveStraightBackward(180);
                    sleep(SERVO_WAIT_TIME);
                    leftFoundationGrabber.setPosition(0); // puts foundation grabbers down
                    rightFoundationGrabber.setPosition(1);
                    sleep(SERVO_WAIT_TIME);

                    //driveAuto(-1, 0, 0, .25, -80);
                    pidPower = .6;
                    driveStraightForward(155);
                    sleep(SERVO_WAIT_TIME);

                    // waitForGamePadA();
                    leftFoundationGrabber.setPosition(.6); // puts foundation grabbers up
                    rightFoundationGrabber.setPosition(.4);
                    sleep(SERVO_WAIT_TIME);

                    //waitForGamePadA();
                    strafeRight(260);
                }
            }
        }
    }

