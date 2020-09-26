package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;

//GoBilda 5202 YellowJacket ticks 383.6 = 384
public abstract class CommonOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    double speedAdjust = 8;
    boolean speedUp = false;
    boolean slowDown = false;
    int increment = 1;
    boolean incrementUp = false;
    boolean incrementDown = false;
    boolean grabberbuttonpushed = false;
    boolean grabberbuttonnotPushed = false;
    boolean capstonebuttonpushed = false;
    boolean capstonebuttonnotPushed = false;
    boolean actuatorbuttonpushed = false;
    boolean actuatorbuttonnotPushed = false;
    boolean grabbersUp = true;
    boolean CapGrabUp = true;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean leftTriggerPulled = false;
    int currentHeight;

    static boolean RED = true;
    static boolean BLUE = false;
    static boolean LEFT = true;
    static boolean RIGHT = false;
    boolean alliance = BLUE;
    boolean position = RIGHT;

    boolean armPress = false;


    public DcMotor flm;
    public DcMotor blm;
    public DcMotor frm;
    public DcMotor brm;
    public DcMotor LEFTSUCTIONMOTOR;
    public DcMotor RIGHTSUCTIONMOTOR;
    public DcMotor leftarm;
    public DcMotor rightarm;
    public CRServo LF;
    public CRServo RF;
    public CRServo RB;
    public CRServo LB;
    public CRServo LI;
    public CRServo RI;
    public Servo leftFoundationGrabber;
    public Servo rightFoundationGrabber;
    public Servo CapGrab;
    public Servo leftactuator;
    public Servo rightactuator;
    static final int CYCLE_MS = 300;

    public TouchSensor leftTouchSensor;
    public TouchSensor rightTouchSensor;
    public ColorSensor leftColorSensor;
    public ColorSensor rightColorSensor;

    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, pidPower = .60, correction, rotation;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    PIDController pidRotate, pidDrive;


    public void allianceChooser() {
        if (gamepad1.b) {
            telemetry.addData("red", "alliance");
            alliance = RED;
        } else {
            telemetry.addData("blue", "alliance");
            alliance = BLUE;
        }

        if (gamepad1.a) {
            telemetry.addData("left", "");
            position = LEFT;
        } else {
            telemetry.addData("right", "");
            position = RIGHT;
        }

        telemetry.update();

    }


    public void initHardware() {
        flm = hardwareMap.dcMotor.get("frm");
        blm = hardwareMap.dcMotor.get("blm");
        frm = hardwareMap.dcMotor.get("flm");
        brm = hardwareMap.dcMotor.get("brm");
        LEFTSUCTIONMOTOR = hardwareMap.dcMotor.get("lsm");
        RIGHTSUCTIONMOTOR = hardwareMap.dcMotor.get("rsm");
        leftactuator = hardwareMap.servo.get("LA");
        rightactuator = hardwareMap.servo.get("RA");
        leftactuator.setPosition(1);
        rightactuator.setPosition(0);
        CapGrab = hardwareMap.servo.get("CapGrab");
        CapGrab.setPosition(1);
        leftarm = hardwareMap.dcMotor.get("leftarm");
        rightarm = hardwareMap.dcMotor.get("rightarm");
        LF = hardwareMap.crservo.get("LF");
        RF = hardwareMap.crservo.get("RF");
        LB = hardwareMap.crservo.get("LB");
        RB = hardwareMap.crservo.get("RB");
        LI = hardwareMap.crservo.get("LI");
        RI = hardwareMap.crservo.get("RI");
        leftFoundationGrabber = hardwareMap.servo.get("LFG");
        leftFoundationGrabber.setPosition(.4);
        rightFoundationGrabber = hardwareMap.servo.get("RFG");
        rightFoundationGrabber.setPosition(.6);
        leftTouchSensor = hardwareMap.get(TouchSensor.class, "LT");
        rightTouchSensor = hardwareMap.get(TouchSensor.class, "RT");
        leftColorSensor = hardwareMap.get(ColorSensor.class, "LeftColorSensor");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "RightColorSensor");
        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftarm.setDirection(DcMotorSimple.Direction.FORWARD);
        rightarm.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
    }

    public void motorForward(DcMotor motor, Integer distance, double power) {
        setupMotorToRunToPosition(motor, distance);
        motor.setPower(power);
        while (motor.isBusy()) {
            idle();
        }
        motor.setPower(0);
    }

    public void motorBackward(DcMotor motor, Integer distance, double power) {
        setupMotorToRunToPosition(motor, -distance);
        motor.setPower(-power);
        while (motor.isBusy()) {
            idle();
        }
        motor.setPower(0);
    }

    public void setupMotorToRunToPosition(DcMotor motor, Integer distance) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(distance);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setupMotorToRunWithoutEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void backToNormal() {
        if (gamepad1.x) {
            setupMotorToRunWithoutEncoder(frm);
        }
    }

    public void actuators(double pos1, double pos2) {
        leftactuator.setPosition(pos1);
        rightactuator.setPosition(pos2);
    }

    public void setSpeed() {
        slowDown();
        speedUp();
    }

    private void slowDown() {
        if (gamepad1.dpad_left) {
            if (!slowDown) {
                speedAdjust = 3;
                slowDown = true;
            }
        } else {
            slowDown = false;
        }
    }

    private void speedUp() {
        if (gamepad1.dpad_right) {
            if (!speedUp) {
                speedAdjust = 8;
                speedUp = true;
            }
        } else {
            speedUp = false;
        }
    }

    public void getGeneralTelemetry() {
        telemetry.addData("IncrementLevel", increment);
        telemetry.addData("speed adjust", "%.2f", speedAdjust);
        telemetry.addData("Arm target position:", "%d", currentHeight);
        //telemetry.addData("Left actuator position:", "%d", leftactuator.getPosition());
        //telemetry.addData("Right actuator position:", "%d", rightactuator.getPosition());
        //telemetry.addData("Skystone", checkSkystone());
        //telemetry.addData("Skystone Diffenence Determinent", leftColorSensor.red() - rightColorSensor.red());
        //telemetry.addData("Left red value", leftColorSensor.red());
        //telemetry.addData("Right red value", rightColorSensor.red());
        telemetry.update();
    }

    public void incrementDown() {
        if (gamepad1.left_bumper && !gamepad2.dpad_up) {
            if (!incrementDown) {
                increment -= 1;
                incrementDown = true;
                if (increment < 1) {
                    increment = 1;
                }
            }
        } else {
            incrementDown = false;
        }
    }

    public void incrementUp() {
        if (gamepad2.right_bumper && !gamepad2.dpad_up) {
            if (!incrementUp) {
                increment += 1;
                incrementUp = true;
                if (increment >= 5) {
                    increment = 5;
                }
            }
        } else {
            incrementUp = false;
        }

    }

    public void drive() {
        double yAxis;
        double xAxis;
        double turn;

        yAxis = gamepad1.left_stick_y;
        xAxis = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        //DON'T CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        blm.setPower((yAxis + xAxis - turn) * (-speedAdjust / 10));
        flm.setPower((yAxis - xAxis - turn) * (-speedAdjust / 10));
        brm.setPower((yAxis - xAxis + turn) * (-speedAdjust / 10));
        frm.setPower((yAxis + xAxis + turn) * (-speedAdjust / 10));
    }

    //changed the method to account for two foundation grabbers
    public void foundationGrabberControl() {
        if (gamepad1.x && grabberbuttonpushed == false) {
            leftFoundationGrabber.setPosition(.5);
            rightFoundationGrabber.setPosition(1);
            grabberbuttonpushed = true;
        } else {
            grabberbuttonpushed = false;
        }

        if (gamepad1.y  && grabberbuttonnotPushed == false) {
            leftFoundationGrabber.setPosition(1);
            rightFoundationGrabber.setPosition(.5);
            grabberbuttonnotPushed = true;
        } else {
            grabberbuttonnotPushed = false;
        }


    }

    public void foundationGrabberOnePress(){
        if (gamepad1.y) {
            if (!yPressed) {
                yPressed = true;
                if (grabbersUp) {
                    leftFoundationGrabber.setPosition(0);
                    rightFoundationGrabber.setPosition(1);
                    sleep(75);
                    grabbersUp = false;
                } else {
                    leftFoundationGrabber.setPosition(.6);
                    rightFoundationGrabber.setPosition(.4);
                    sleep(75);
                    grabbersUp = true;
                }
            }
        } else {
            yPressed = false;
        }
    }

    public void lessThanEqualDistance(double target) {
        while (opModeIsActive() && distanceDone(target)) {
            sleep(CYCLE_MS);
            idle();
        }

    }

    public void resetDrive() {
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void resetDriveWithoutEncoder() {
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runWithoutEncoders() {
        flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopDriveMotors() {
        flm.setPower(0);
        blm.setPower(0);
        frm.setPower(0);
        brm.setPower(0);
    }

    public void setDrivePower(double DrivePower) {
        flm.setPower(DrivePower);
        blm.setPower(DrivePower);
        frm.setPower(DrivePower);
        brm.setPower(DrivePower);
    }

    public void runToPosition() {
        flm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void waitForMotorsAndRelayTelm() {
        while (opModeIsActive() && flm.isBusy() && blm.isBusy() && frm.isBusy() && brm.isBusy()) {
            telemetry.addData("FL", flm.getCurrentPosition());
            telemetry.addData("BL", blm.getCurrentPosition());
            telemetry.addData("FR", frm.getCurrentPosition());
            telemetry.addData("BR", brm.getCurrentPosition());
            telemetry.update();
            //idle();
        }
    }

    public void motorSpeedRelay() {
        telemetry.addData("FL speed", flm.getPower());
        telemetry.addData("BL speed", blm.getPower());
        telemetry.addData("FR speed", frm.getPower());
        telemetry.addData("BR speed", brm.getPower());
        telemetry.update();
    }

    public void setDriveY(int y) {
        y = (int) ((y * 1120) / 31.4);
        flm.setTargetPosition(y);
        blm.setTargetPosition(y);
        frm.setTargetPosition(y);
        brm.setTargetPosition(y);
    }

    public void setDriveX(int x) {
        x = (int) ((x * 1120) / 31.4);
        flm.setTargetPosition(x);
        blm.setTargetPosition(-x);
        frm.setTargetPosition(-x);
        brm.setTargetPosition(x);
    }

    public void setDriveRotate(int r) {
        flm.setTargetPosition(r);
        blm.setTargetPosition(r);
        frm.setTargetPosition(-r);
        brm.setTargetPosition(-r);
    }

    public void DriveX(int distance, double speed) {
        setDriveX(distance);
        runToPosition();
        setDrivePower(speed);
        waitForMotorsAndRelayTelm();
        setDrivePower(0);
        resetDrive();
    }

    public void DriveY(int distance, double speed) {
        setDriveY(distance);
        runToPosition();
        setDrivePower(speed);
        waitForMotorsAndRelayTelm();
        setDrivePower(0);
        resetDrive();
    }

    public void armsResetAndRun() {
        leftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightarm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void armsReset() {
        boolean armszero = leftarm.getCurrentPosition() == 0 && rightarm.getCurrentPosition() == 0;

        if (armszero) {
            leftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void armsRunToPosition() {
        leftarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void armsPower(double power) {
        leftarm.setPower(power);
        rightarm.setPower(power);
    }

    public void arms() {

        int maxBlock = 5;
        int blockHeight = 570;
        int fudgeFactor = 60  * (increment - 1);
        currentHeight = (blockHeight * increment) + fudgeFactor;
        int armTopLimit = (blockHeight * (maxBlock + 1)) + (60 * maxBlock);

        if (currentHeight >= armTopLimit) {
            telemetry.addData("Danger", "will robinson");
        } else if (gamepad1.a && !armPress) {
            leftarm.setTargetPosition(currentHeight);
            rightarm.setTargetPosition(currentHeight);
            armsPower(.6);
            armsRunToPosition();
            increment++;
            armPress = true;
        } else if (gamepad1.b) {
            leftarm.setTargetPosition(0);
            rightarm.setTargetPosition(0);
            armsPower(-.5);
            armsRunToPosition();
            armsReset();
            armPress = false;
        }
    }

    /*public void driveAutoForward() {
        if(leftTouchSensor.isPressed() || rightTouchSensor.isPressed())
        blm.setPower(.3);
        flm.setPower(.3);
        brm.setPower(.3);
        frm.setPower(.3);
    }*/

    public void suction() {
        if (gamepad1.left_trigger == 1) {
            LF.setPower(1);
            LB.setPower(-1);
            RF.setPower(1);
            RB.setPower(-1);
            LI.setPower(-1);
            RI.setPower(1);
            LEFTSUCTIONMOTOR.setPower(-1);
            RIGHTSUCTIONMOTOR.setPower(1);
            //driveAutoForward();
        } else if (gamepad1.right_trigger == 1) {
            LF.setPower(-1);
            LB.setPower(1);
            RF.setPower(-1);
            RB.setPower(1);
            LI.setPower(1);
            RI.setPower(-1);
            LEFTSUCTIONMOTOR.setPower(1);
            RIGHTSUCTIONMOTOR.setPower(-1);
            //driveAuto(1,0,0,
        } else {
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);
            LI.setPower(0);
            RI.setPower(0);
            LEFTSUCTIONMOTOR.setPower(0);
            RIGHTSUCTIONMOTOR.setPower(0);

        }
    }

    /*public void suction() {
        if (gamepad1.left_trigger == 1) {
            if (!leftTriggerPulled) {
                leftTriggerPulled = true;
                LF.setPower(1);
                LB.setPower(-1);
                RF.setPower(1);
                RB.setPower(-1);
                LI.setPower(-1);
                RI.setPower(1);
                LEFTSUCTIONMOTOR.setPower(-1);
                RIGHTSUCTIONMOTOR.setPower(1);
                sleep(4000);
                LF.setPower(0);
                LB.setPower(0);
                RF.setPower(0);
                RB.setPower(0);
                LI.setPower(0);
                RI.setPower(0);
                LEFTSUCTIONMOTOR.setPower(0);
                RIGHTSUCTIONMOTOR.setPower(0);
            }
        } else if ((gamepad1.left_trigger == 0) && leftTriggerPulled){
            leftTriggerPulled = false;
            LI.setPower(-1);
            RI.setPower(1);
            sleep(2000);
            LI.setPower(0);
            RI.setPower(0);
        }
    }*/

    public void pushOut() {
        if (gamepad1.left_trigger == 1) {
            LI.setPower(-0.75);
            RI.setPower(0.75);
        } else {
            LI.setPower(0);
            RI.setPower(0);
        }
    }

    public void pushOutBackwards() {
        LI.setPower(-0.75);
        RI.setPower(0.75);
        //sleep(500);
        sleep(800);
        driveAuto(-1, 0, 0, .3, -15);
        runWithoutEncoders();
    }

    public void autoPushOut() {
        LF.setPower(1);
        LB.setPower(-1);
        RF.setPower(1);
        RB.setPower(-1);
        LI.setPower(0.75);
        RI.setPower(-0.75);
        sleep(500);
    }

    public void autoSuckIn() {
        LEFTSUCTIONMOTOR.setPower(-1);
        RIGHTSUCTIONMOTOR.setPower(1);
        LF.setPower(1);
        LB.setPower(-1);
        RF.setPower(1);
        RB.setPower(-1);
        LI.setPower(-1);
        RI.setPower(1);
        //sleep(50);
    }

    public void blockStrafeLeft() {
        flm.setPower(.2);
        blm.setPower(-.4);
        frm.setPower(-.4);
        brm.setPower(.2);
    }

    public void blockStrafeRight() {
        flm.setPower(-.6);
        blm.setPower(.3);
        frm.setPower(.3);
        brm.setPower(-.6);
    }

    public void lineUpBlock() {
        if (gamepad1.dpad_up && (leftTouchSensor.isPressed() || rightTouchSensor.isPressed())) {
            setDrivePower(-.4);
            /*while (gamepad2.left_bumper && opModeIsActive()) {
                if (leftTouchSensor.isPressed() && rightTouchSensor.isPressed()) {
                    blockStrafeRight();
                    sleep(100);
                } else {
                    blm.setPower(0);
                    flm.setPower(0);
                    brm.setPower(0);
                    frm.setPower(0);
                    //strafeRightAuto();
                    sleep(CYCLE_MS);
                    pushOutBackwards();
                    break;
                }
            }*/

            while (gamepad1.right_bumper && opModeIsActive()) {
                if (leftTouchSensor.isPressed() && rightTouchSensor.isPressed()) {
                    blockStrafeLeft();
                    sleep(100);
                } else {
                    blm.setPower(0);
                    flm.setPower(0);
                    brm.setPower(0);
                    frm.setPower(0);
                    //strafeLeftAuto();
                    sleep(CYCLE_MS);
                    //driveAuto(1,0,0,1,2);
                    pushOutBackwards();
                    break;
                }
            }
        }
        stopDriveMotors();
    }

    public void strafeLeftAuto() {
        driveAuto(0, 1, 0, .5, 7);
    }

    public void strafeRightAuto() {
        driveAuto(0, -1, 0, .5, -13);
    }


    public boolean distanceDone(double target) {
        return (abs(flm.getCurrentPosition()) <= abs(target)) && (abs(blm.getCurrentPosition()) <= abs(target))
                && (abs(frm.getCurrentPosition()) <= abs(target))
                && (abs(brm.getCurrentPosition()) <= abs(target));

    }

    public void distanceDrive(double distance_cm) {
        double target = (int) ((distance_cm * 1120) / 31.4);
        resetDrive();
        while (opModeIsActive() && (/*sensorRange.getDistance(DistanceUnit.CM) > 4 ||*/ distanceDone(target))) {
            //telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("distance met", "%s", distanceDone(target));
            telemetry.update();
            flm.setPower(-1);
            blm.setPower(-1);
            brm.setPower(-1);
            frm.setPower(-1);
        }
        stopDriveMotors();
    }

    public void leftActuatorTest() {
        if (gamepad1.a) {
            leftactuator.setPosition(0.65);
            telemetry.addData("Active?", "Yes");
        } else {
            telemetry.addData("Active?", "No");
        }
        telemetry.update();
    }

    public void driveMotorTest() {
        if (gamepad1.dpad_up) {
            flm.setPower(1);
        } else if (gamepad1.dpad_down) {
            frm.setPower(1);
        } else if (gamepad1.dpad_left) {
            blm.setPower(1);
        } else if (gamepad1.dpad_right) {
            brm.setPower(1);
        } else {
            flm.setPower(0);
            blm.setPower(0);
            frm.setPower(0);
            brm.setPower(0);
        }
    }

    public void autoDriveBackwardsIndefinitely() {
        resetDriveWithoutEncoder();
        correction = pidDrive.performPID(getAngle());
        blm.setPower(-(pidPower + (correction / 2)));
        flm.setPower(-(pidPower + (correction / 2)));
        brm.setPower(-(pidPower - (correction / 2)));
        frm.setPower(-(pidPower - (correction / 2)));
    }

    public void autoDriveForwardsIndefinitelySlowly() {
        flm.setPower(.15);
        blm.setPower(.15);
        frm.setPower(.15);
        brm.setPower(.15);
    }

    public boolean checkSkystone() {
        //100 is the deciding constant that determines either stone or Skystone
        return abs(leftColorSensor.red() - rightColorSensor.red()) > 100;
    }

    public void initPID() {
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated() && opModeIsActive()) {
            sleep(50);
            idle();
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                //rightTurn(power);
                sleep(100);
            }

            do {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                //rightTurn(-power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        } else    // left turn.
            do {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                //leftTurn(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        stopDriveMotors();

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rightTurn(double power, double radian) {
        resetDrive();
        while(abs(flm.getCurrentPosition()) <= abs(radian) && opModeIsActive()) {
            blm.setPower(-power);
            flm.setPower(-power);
            brm.setPower(power);
            frm.setPower(power);
        }
        stopDriveMotors();
    }


    public void leftTurn(double power, double radian) {
        resetDrive();
        while(abs(flm.getCurrentPosition()) <= abs(radian) && opModeIsActive() ) {
            blm.setPower(power);
            flm.setPower(power);
            brm.setPower(-power);
            frm.setPower(-power);
        }
        stopDriveMotors();
    }

    public void driveStraightForward(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(flm.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            blm.setPower((pidPower - (correction / 2)));
            flm.setPower((pidPower - (correction / 2)));
            brm.setPower((pidPower - (correction / 2)));
            frm.setPower((pidPower - (correction / 2)));
        }
        stopDriveMotors();
    }

    public void driveStraightBackward(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(flm.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            blm.setPower(-(pidPower + (correction / 2)));
            flm.setPower(-(pidPower + (correction / 2)));
            brm.setPower(-(pidPower - (correction / 2)));
            frm.setPower(-(pidPower - (correction / 2)));
        }
        stopDriveMotors();
    }

    public void strafeLeft(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(flm.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            blm.setPower(pidPower - (correction / 2));
            flm.setPower(-pidPower - (correction / 2));
            brm.setPower(-pidPower + (correction / 2));
            frm.setPower(pidPower + (correction / 2));
        }
        stopDriveMotors();
    }

    public void strafeRight(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(flm.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            blm.setPower(-pidPower - (correction / 2));
            flm.setPower(pidPower - (correction / 2));
            brm.setPower(pidPower + (correction / 2));
            frm.setPower(-pidPower + (correction / 2));
        }
        stopDriveMotors();
    }

    public void setupPIDParameters() {
        pidRotate.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, pidPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
    }

    public void telemetryPID() {
        //telemetry.addData("1 imu heading", lastAngles.firstAngle);
        //telemetry.addData("2 global heading", globalAngle);
        //telemetry.addData("3 correction", correction);
        //telemetry.addData("4 turn rotation", rotation);
        telemetry.addData("5 PID power", pidPower);
        telemetry.addData("fl motor encoder", abs(flm.getCurrentPosition()));
        telemetry.addData("bl motor encoder", abs(blm.getCurrentPosition()));
        telemetry.addData("fr motor encoder", abs(frm.getCurrentPosition()));
        telemetry.addData("br motor encoder", abs(brm.getCurrentPosition()));
        telemetry.addData("frp", frm.getPower());
        telemetry.update();
    }

    public void driveAuto(double straight, double strafe, double turn, double speed, int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDrive();

        blm.setPower((straight + strafe - turn) * (-speed));
        flm.setPower((straight - strafe - turn) * (-speed));
        brm.setPower((straight - strafe + turn) * (-speed));
        frm.setPower((straight + strafe + turn) * (-speed));
        lessThanEqualDistance(distance_encoder);
        motorSpeedRelay();
        stopDriveMotors();
    }

    public void actuatorControl() {
        if (gamepad1.x && actuatorbuttonpushed == false) {
            leftactuator.setPosition(0.65);
            rightactuator.setPosition(-0.65);
            actuatorbuttonpushed = true;
        } else {
            actuatorbuttonpushed = false;
        }

        if (gamepad1.y && actuatorbuttonnotPushed == false) {
            leftactuator.setPosition(0);
            rightactuator.setPosition(0);
            actuatorbuttonnotPushed = true;
        } else {
            actuatorbuttonnotPushed = false;
        }


    }

    public void autoScoreSkystone() {
        if (checkSkystone()) {
            stopDriveMotors();
            pidPower = .4;
            strafeRight(75);
            //driveBackwardsToFoundation();
            LB.setPower(-1);
            RB.setPower(-1);
            LI.setPower(-1);
            RI.setPower(1);
            sleep(3000);
            /*LEFTSUCTIONMOTOR.setPower(0);
            RIGHTSUCTIONMOTOR.setPower(0);
            LF.setPower(0);
            LB.setPower(-1);
            RF.setPower(0);
            RB.setPower(-1);
            LI.setPower(-1);
            RI.setPower(1);
            sleep(2000);
            strafeRight(75);
            driveBackwardsToFoundation();
            autoPushOut();
            strafeRight(25);
            driveStraightForward(50);*/
        }
    }

    public void driveBackwardsSlowlyToFoundation() {
        pidPower = .2;
        while (!leftTouchSensor.isPressed() || !rightTouchSensor.isPressed()) {
            autoDriveBackwardsIndefinitely();
        }
    }

    public void blueAutoDriveForwardAndStoneCheck() {
        autoSuckIn();
        pidPower = .2;
        //driveForwardForSkystoneCollection(10);
        //driveForwardIndefinitlyWithPID();
        //autoDriveForwardsIndefinitelySlowly();

        while (!checkSkystone()) {
            //driveStraightForward(30);
            setDrivePower(.2);
            sleep(400);
            stopDriveMotors();
            sleep(650);
        }
    }

    public void redAutoDriveForwardAndStoneCheck() {
        autoSuckIn();
        //driveForwardForSkystoneCollection(10);
        //driveForwardIndefinitlyWithPID();
        //autoDriveForwardsIndefinitelySlowly();

        while (!checkSkystone()) {
            //driveStraightForward(30);
            setDrivePower(.2);
            sleep(400);
            stopDriveMotors();
            sleep(650);
        }
    }

    public void  waitForGamePadA() {
        while(!gamepad1.a) {
            sleep(20);
        }
    }

    public void capstoneGrabber() {
        if (gamepad1.x) {
            if (!xPressed) {
                xPressed = true;
                if (CapGrabUp) {
                    CapGrab.setPosition(0.5);
                    sleep(75);
                    CapGrabUp = false;
                } else {
                    CapGrab.setPosition(1);
                    sleep(75);
                    CapGrabUp = true;
                }
            }
        } else {
            xPressed = false;
        }

        }

    public void stoneDisposal() {
        LEFTSUCTIONMOTOR.setPower(0);
        RIGHTSUCTIONMOTOR.setPower(0);
        LF.setPower(-1);
        LB.setPower(0);
        RF.setPower(-1);
        RB.setPower(0);
        LI.setPower(-1);
        RI.setPower(1);
    }

    public void stopStoneDisposal() {
        LEFTSUCTIONMOTOR.setPower(0);
        RIGHTSUCTIONMOTOR.setPower(0);
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LI.setPower(0);
        RI.setPower(0);
    }

    public void driveForwardIndefinitlyWithPID() {
        resetDriveWithoutEncoder();
        correction = pidDrive.performPID(getAngle());
        blm.setPower((pidPower - (correction / 2)));
        flm.setPower((pidPower - (correction / 2)));
        brm.setPower((pidPower - (correction / 2)));
        frm.setPower((pidPower - (correction / 2)));
    }

    }
