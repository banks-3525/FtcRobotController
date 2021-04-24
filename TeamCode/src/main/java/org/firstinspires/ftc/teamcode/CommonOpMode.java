package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

//GoBilda 5202 YellowJacket ticks 383.6 = 384
public abstract class CommonOpMode extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    double speedAdjust = 8;
    double ticks;
    double time;
    double rpm;
    double targetRPM = 800;
    double power = .5;
    double powerIncrement;
    double proportionalSpeed;

    int bCounter = 0;
    int aCounter = 0;
    int xCounter = 0;
    int upCounter = 0;

    long setTime = System.currentTimeMillis();

    boolean speedUp = false;
    boolean slowDown = false;
    boolean style = false;
    boolean withinRange = false;
    boolean powerShot = false;
    boolean grabbed = false;
    boolean dpadUpPressed = false;
    boolean pivotedUp = false;
    boolean atRest = false;
    boolean yPressed = false;
    boolean xPressed = false;
    boolean rampUp = false;

    static boolean RED = true;
    static boolean BLUE = false;
    static boolean LEFT = true;
    static boolean RIGHT = false;
    static boolean ROBOT = false;
    static boolean FIELD = true;
    boolean alliance = BLUE;
    boolean position = RIGHT;
    boolean drive = ROBOT;
    boolean armPress = false;

    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    //public DcMotor ringLauncherMotor;
    public DcMotorEx ringLauncherMotor;
    public DcMotor topIntakeMotor;
    public DcMotor bottomIntakeMotor;
    public DcMotor wobbleGrabberMotor;

    public Servo ringPushServo;
    public Servo liftAngleServo;
    public Servo grabberPivotServo;
    public Servo grabberHandServo;

    public DistanceSensor distanceSensor;

    public LED powerIndicator;

    static final int CYCLE_MS = 300;

    public BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, pidPower = .60, correction, rotation;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    PIDController pidRotate, pidDrive, pidStrafe, pidRightStrafe, pidLeftStrafe;

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

    public void driveChooser() {
        if (gamepad1.a) {
            telemetry.addData("Field-Centric Driving:", "Active");
            drive = FIELD;
        } else {
            telemetry.addData("Robot-Centric Driving", "Active");
            drive = ROBOT;
        }

        telemetry.update();
    }

    public void initHardware2020() {
        // note about motors
        //
        // frontLeftMotor is actually front right
        // frontRightMotor is actually front left
        //
        // frontRightMotor and backRightMotor are the only motors
        // with working encoder connections

        frontLeftMotor = hardwareMap.dcMotor.get("fr");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fl");
        backRightMotor = hardwareMap.dcMotor.get("br");
        ringLauncherMotor = (DcMotorEx) hardwareMap.dcMotor.get("RING");
        topIntakeMotor = hardwareMap.dcMotor.get("TIM");
        bottomIntakeMotor = hardwareMap.dcMotor.get("BIM");
        wobbleGrabberMotor = hardwareMap.dcMotor.get("WGM");

        ringLauncherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ringLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ringLauncherMotor.setVelocityPIDFCoefficients(50,3,5,26.85);
        ringLauncherMotor.setPositionPIDFCoefficients(5);
        //ringLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleGrabberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftAngleServo = hardwareMap.servo.get("LAS");
        ringPushServo = hardwareMap.servo.get("RPS");
        grabberPivotServo = hardwareMap.servo.get("ArmRotationServo");
        grabberHandServo = hardwareMap.servo.get("GrabberServo");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        powerIndicator = hardwareMap.led.get("PowerIndicator");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
    }

    public void setupMotorToRunToPosition(DcMotor motor, Integer distance) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(distance);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSpeed() {
        //slowDown();
        //speedUp();
        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (upCounter % 2 == 0) {
                speedAdjust = 4;
            } else {
                speedAdjust = 8;
            }
            dpadUpPressed = true;
            upCounter++;
        } else if (!gamepad1.dpad_up && dpadUpPressed) {
            dpadUpPressed = false;
        }
    }

    private void slowDown() {
        if (gamepad1.dpad_down) {
            if (!slowDown) {
                speedAdjust = 4;
                slowDown = true;
            }
        } else {
            slowDown = false;
        }
    }

    private void speedUp() {
        if (gamepad1.dpad_up) {
            if (!speedUp) {
                speedAdjust = 8;
                speedUp = true;
            }
        } else {
            speedUp = false;
        }
    }

    public void getGeneralTelemetry() {
        //telemetry.addData("Angle Reading:", getAngle());
        telemetry.addData("RPM:", ringLauncherMotor.getVelocity());
        //telemetry.addData("PIDF Coefficients", ringLauncherMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        /*if (targetRPM == 800) {
            telemetry.addData("High Goal:", "Yes");
        } else if (targetRPM == 700) {
            telemetry.addData("Power Goal:", "Yes");
        }

        telemetry.addData("Target RPM:", targetRPM);
        telemetry.addData("Ring Launcher Power:", ringLauncherMotor.getPower());
        telemetry.addData("Power Increment:", powerIncrement);
        telemetry.addData("Wobble Pivot Angle:", grabberPivotServo.getPosition());*/
        //telemetry.addData("Correction:", correction);
        telemetry.addData("Back Left Encoders:", backLeftMotor.getCurrentPosition());
        telemetry.addData("Front Left Encoders:", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Back Right Encoders:", backRightMotor.getCurrentPosition());
        telemetry.addData("Front Right Encoders:", frontRightMotor.getCurrentPosition());
        telemetry.addData("Wobble Goal Enocders:", wobbleGrabberMotor.getCurrentPosition());
        telemetry.addData("Distance in Inches:", distanceSensor.getDistance(DistanceUnit.INCH));
        //telemetry.addData("IncrementLevel", increment);
        telemetry.addData("Y-Axis Left Stick:", gamepad1.left_stick_y);
        telemetry.addData("X-Axis Left Stick:", gamepad1.left_stick_x);
        telemetry.addData("X-Axis Right Stick:", gamepad1.right_stick_x);
        telemetry.addData("Speed Level", speedAdjust);
        telemetry.update();
    }

    public boolean joysticksActive() {
        return (gamepad1.left_stick_y != 0) && (gamepad1.left_stick_x != 0) && (gamepad1.right_stick_x != 0);
    }

    public boolean rightStickActive() {
        return (gamepad1.right_stick_x != 0);
    }

    public double rpm() {
        if (abs(time - System.currentTimeMillis()) > 100) {
            rpm = ((ticks - abs(ringLauncherMotor.getCurrentPosition())) / (time - System.currentTimeMillis())) * 1000;

            ticks = abs(ringLauncherMotor.getCurrentPosition());
            time = System.currentTimeMillis();

            if (abs(rpm()) > 825) {
                ringLauncherMotor.setPower(power -= .05);
            } else if (abs(rpm()) < 775) {
                ringLauncherMotor.setPower(power += .05);
            }
        }

        // 800 is RPM target

        return rpm;
    }

    public void motorBreak() {
        if (!joysticksActive()) {
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void robotCentricDrive() {
        double yAxis;
        double xAxis;
        double strafe;

        yAxis = gamepad1.left_stick_y;
        xAxis = gamepad1.left_stick_x;
        strafe = gamepad1.right_stick_x;
        //DON'T CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        backLeftMotor.setPower((-yAxis + xAxis /*+ strafe*/) * (-speedAdjust / 10));
        frontLeftMotor.setPower((yAxis + xAxis /*- strafe*/) * (-speedAdjust / 10));
        backRightMotor.setPower((yAxis + xAxis /*+ strafe*/) * (-speedAdjust / 10));
        frontRightMotor.setPower((-yAxis + xAxis /*- strafe*/) * (-speedAdjust / 10));

        if (strafe == 1 || strafe == -1) {
            resetAngle();
            while (rightStickActive()) {
                correction = pidStrafe.performPID(getAngle());
                backLeftMotor.setPower((strafe * (-speedAdjust / 10)) + correction);
                frontLeftMotor.setPower(-(strafe * (-speedAdjust / 10)) + correction);
                backRightMotor.setPower((strafe * (-speedAdjust / 10)) + correction);
                frontRightMotor.setPower(-(strafe * (-speedAdjust / 10)) + correction);
            }
        }

        motorBreak();
    }

    public void fieldCentricDrive() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad1.right_stick_x;

        correction = pidDrive.performPID(getAngle());

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontRightMotor.setPower(v1 + correction);
        frontLeftMotor.setPower(-v2 + correction);
        backLeftMotor.setPower(v3 + correction);
        backRightMotor.setPower(-v4 + correction);

        motorBreak();
    }

    public void newFieldCentricDrive() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + Math.PI;
        double rightX = -gamepad1.right_stick_x;

        double newAngle = robotAngle - (90 + getAngle());

        final double v1 = r * Math.cos(newAngle) + rightX;
        final double v2 = r * Math.sin(newAngle) - rightX;
        final double v3 = r * Math.sin(newAngle) + rightX;
        final double v4 = r * Math.cos(newAngle) - rightX;

        frontRightMotor.setPower(v1);
        frontLeftMotor.setPower(-v2);
        backLeftMotor.setPower(v3);
        backRightMotor.setPower(-v4);

        motorBreak();
    }

    public void resetAlignment() {
        correction = pidDrive.performPID(getAngle());

        frontRightMotor.setPower(correction * 2);
        frontLeftMotor.setPower(correction * 2);
        backLeftMotor.setPower(correction * 2);
        backRightMotor.setPower(correction * 2);
    }

    public void lessThanEqualDistance(double target) {
        while (opModeIsActive() && distanceDone(target)) {
            sleep(CYCLE_MS);
            idle();
        }
    }

    public void resetDrive() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDriveWithoutEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void setDrivePower(double DrivePower) {
        frontLeftMotor.setPower(DrivePower);
        backLeftMotor.setPower(DrivePower);
        frontRightMotor.setPower(DrivePower);
        backRightMotor.setPower(DrivePower);
    }

    public void runToPosition() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void waitForMotorsAndRelayTelm() {
        while (opModeIsActive() && frontLeftMotor.isBusy() && backLeftMotor.isBusy() && frontRightMotor.isBusy() && backRightMotor.isBusy()) {
            telemetry.addData("FL", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BL", backLeftMotor.getCurrentPosition());
            telemetry.addData("FR", frontRightMotor.getCurrentPosition());
            telemetry.addData("BR", backRightMotor.getCurrentPosition());
            telemetry.update();
            //idle();
        }
    }

    public void motorSpeedRelay() {
        telemetry.addData("FL speed", frontLeftMotor.getPower());
        telemetry.addData("BL speed", backLeftMotor.getPower());
        telemetry.addData("FR speed", frontRightMotor.getPower());
        telemetry.addData("BR speed", backRightMotor.getPower());
        telemetry.update();
    }

    public boolean distanceDone(double target) {
        return (abs(frontLeftMotor.getCurrentPosition()) <= abs(target)) && (abs(backLeftMotor.getCurrentPosition()) <= abs(target))
                && (abs(frontRightMotor.getCurrentPosition()) <= abs(target))
                && (abs(backRightMotor.getCurrentPosition()) <= abs(target));

    }

    public void driveMotorTest() {
        if (gamepad1.dpad_up) {
            frontLeftMotor.setPower(1);
        } else if (gamepad1.dpad_down) {
            frontRightMotor.setPower(1);
        } else if (gamepad1.dpad_left) {
            backLeftMotor.setPower(1);
        } else if (gamepad1.dpad_right) {
            backRightMotor.setPower(1);
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }

    public void initPID() {
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.075, 0, 0);
        pidStrafe = new PIDController(.075, 0, 1);
        pidRightStrafe = new PIDController(.1, 0, 0);
        pidLeftStrafe = new PIDController(.1, 0, 0);

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
    public void resetAngle() {
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
            do { // right turn
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                backLeftMotor.setPower(power);
                frontLeftMotor.setPower(power);
                backRightMotor.setPower(power);
                frontRightMotor.setPower(power);
            } while (opModeIsActive() && getAngle() > degrees);
        } else
            do { // left turn
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                backLeftMotor.setPower(power);
                frontLeftMotor.setPower(power);
                backRightMotor.setPower(power);
                frontRightMotor.setPower(power);
            } while (opModeIsActive() && getAngle() < degrees);

        // turn the motors off.
        stopDriveMotors();

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rightTurn(double power, double radian) {
        resetDrive();
        while (abs(frontRightMotor.getCurrentPosition()) <= abs(radian) && opModeIsActive()) {
            backLeftMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            frontRightMotor.setPower(power);
            telemetry.addData("rightTurn", "Yes");
            telemetry.update();
        }
        stopDriveMotors();
    }

    public void rightTurnNoPID(double radian) {
        resetDriveWithoutEncoder();
        while (getAngle() >= -radian && opModeIsActive()) {
            backLeftMotor.setPower(-.6);
            frontLeftMotor.setPower(-.6);
            backRightMotor.setPower(-.6);
            frontRightMotor.setPower(-.6);
        }
        stopDriveMotors();
    }

    public void center() {
        resetDriveWithoutEncoder();
        while (!(getAngle() == 0) && opModeIsActive()) {
            backLeftMotor.setPower(correction * 1.5);
            frontLeftMotor.setPower(correction * 1.5);
            backRightMotor.setPower(correction * 1.5);
            frontRightMotor.setPower(correction * 1.5);
        }
        stopDriveMotors();
    }


    public void leftTurn(double power, double radian) {
        resetDrive();
        while (abs(frontRightMotor.getCurrentPosition()) <= abs(radian) && opModeIsActive()) {
            backLeftMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
            frontRightMotor.setPower(-power);
            telemetry.addData("leftTurn", "Yes");
            telemetry.update();
        }
        stopDriveMotors();
    }

    public void leftTurnNoPID(double radian) {
        resetDriveWithoutEncoder();
        while (getAngle() <= radian && opModeIsActive()) {
            backLeftMotor.setPower(.4);
            frontLeftMotor.setPower(.4);
            backRightMotor.setPower(.4);
            frontRightMotor.setPower(.4);
        }
        stopDriveMotors();
    }


    public void driveStraightForward(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontRightMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower(-(pidPower - (correction / 2)));
            frontLeftMotor.setPower((pidPower - (correction / 2)));
            backRightMotor.setPower((pidPower - (correction / 2)));
            frontRightMotor.setPower(-(pidPower - (correction / 2)));
        }
        stopDriveMotors();
    }

    public void driveStraightBackward(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontRightMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower((pidPower + (correction / 2)));
            frontLeftMotor.setPower(-(pidPower + (correction / 2)));
            backRightMotor.setPower(-(pidPower - (correction / 2)));
            frontRightMotor.setPower((pidPower - (correction / 2)));
        }
        stopDriveMotors();
    }

    public void strafeRight(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontRightMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidStrafe.performPID(getAngle());
            //correction = 0;
            telemetryPID();
            backLeftMotor.setPower(-pidPower + correction);
            frontLeftMotor.setPower(pidPower + correction);
            backRightMotor.setPower(-pidPower + correction);
            frontRightMotor.setPower(pidPower + correction);
        }
        stopDriveMotors();
    }

    public void strafeLeft(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontRightMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidStrafe.performPID(getAngle());
            //correction = 0;
            telemetryPID();
            backLeftMotor.setPower(pidPower + correction);
            frontLeftMotor.setPower(-pidPower + correction);
            backRightMotor.setPower(pidPower + correction);
            frontRightMotor.setPower(-pidPower + correction);
        }
        stopDriveMotors();
    }

    public void diagonalUpLeft(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontRightMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower(correction);
            frontLeftMotor.setPower(correction);
            backRightMotor.setPower(pidPower + correction);
            frontRightMotor.setPower(-pidPower + correction);
        }
        stopDriveMotors();
    }

    public void setupPIDParameters() {
        pidRotate.reset();

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, pidPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, pidPower);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();

        pidLeftStrafe.setSetpoint(0);
        pidLeftStrafe.setOutputRange(0, pidPower);
        pidLeftStrafe.setInputRange(-90, 90);
        pidLeftStrafe.enable();

        pidRightStrafe.setSetpoint(0);
        pidRightStrafe.setOutputRange(0, pidPower);
        pidRightStrafe.setInputRange(-90, 90);
        pidRightStrafe.enable();
    }

    public void telemetryPID() {
        //telemetry.addData("1 imu heading", lastAngles.firstAngle);
        //telemetry.addData("2 global heading", globalAngle);
        //telemetry.addData("3 correction", correction);
        //telemetry.addData("4 turn rotation", rotation);
        telemetry.addData("5 PID power", pidPower);
        telemetry.addData("fl motor encoder", abs(frontLeftMotor.getCurrentPosition()));
        telemetry.addData("bl motor encoder", abs(backLeftMotor.getCurrentPosition()));
        telemetry.addData("fr motor encoder", abs(frontRightMotor.getCurrentPosition()));
        telemetry.addData("br motor encoder", abs(backRightMotor.getCurrentPosition()));
        telemetry.addData("Proportional Speed", proportionalSpeed);
        telemetry.update();
    }

    public void driveAuto(double straight, double strafe, double turn, double speed, int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDrive();

        backLeftMotor.setPower((straight + strafe - turn) * (-speed));
        frontLeftMotor.setPower((straight - strafe - turn) * (-speed));
        backRightMotor.setPower((straight - strafe + turn) * (-speed));
        frontRightMotor.setPower((straight + strafe + turn) * (-speed));
        lessThanEqualDistance(distance_encoder);
        motorSpeedRelay();
        stopDriveMotors();
    }

    public void waitForGamePadA() {
        while (!gamepad1.a) {
            sleep(20);
        }
    }
    /*public void ringLauncherRevUp2() {
        //ringLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ringLauncherMotor.setPower(.7);

        rpm = ((ticks - abs(ringLauncherMotor.getCurrentPosition())) / (time - System.currentTimeMillis())) * 1000;

        ticks = abs(ringLauncherMotor.getCurrentPosition());
        time = System.currentTimeMillis();
        if (abs(rpm()) > 825) {
            // ringLauncherMotor.setPower(power -= powerIncrement);
            withinRange = false;
        } else if (abs(rpm()) < 775) {
            // ringLauncherMotor.setPower(power += powerIncrement);
            withinRange = false;
        } else {
            withinRange = true;
        }
    }*/

    public void ringLauncherRevUp() {
        // 800 is RPM target for High Goal
        // 700 is RPM target for Power Shot

        if (abs(time - System.currentTimeMillis()) > 80) {
            rpm = ((ticks - abs(ringLauncherMotor.getCurrentPosition())) / (time - System.currentTimeMillis())) * 1000;

            ticks = abs(ringLauncherMotor.getCurrentPosition());
            time = System.currentTimeMillis();

            /*powerIncrement = ((targetRPM - rpm) / (targetRPM)) * (1/12);
            if (abs(rpm) > (targetRPM + 25)) {
                ringLauncherMotor.setPower(power -= powerIncrement);
                //ringLauncherMotor.setPower(power -= .1);
                withinRange = false;
            } else if (abs(rpm) < (targetRPM - 25)) {
                ringLauncherMotor.setPower(power += powerIncrement);
                //ringLauncherMotor.setPower(power += .1);
                withinRange = false;
            } else {
                withinRange = true;
            }*/

            powerIncrement = ((targetRPM - rpm) / (targetRPM)) * (1/8);
            ringLauncherMotor.setPower(power += powerIncrement);

            if (rpm > (targetRPM + 25)) {
                withinRange = false;
            } else if (rpm < (targetRPM - 25)) {
                withinRange = false;
            } else {
                withinRange = true;
            }
        }

    }

    /*public void goalSwitcher() {
        if (gamepad1.x && !powerShot) {
            targetRPM = 700;
            powerShot = true;
        } else if (gamepad1.x && powerShot) {
            targetRPM = 800;
            powerShot = false;
        }
    }*/

    public void ringLauncherPosition() {
        if (gamepad1.right_trigger == 1 ){ //highGoal
            //liftAngleServo.setPosition(.65);
            liftAngleServo.setPosition(.76);
            targetRPM = 800;
        } else if (gamepad1.right_bumper) { //powerShot
            //liftAngleServo.setPosition(.85);
            liftAngleServo.setPosition(.85);
            targetRPM = 750;
        }
    }

    public void autoRingPushTrigger() {
        /*while (abs(ringLauncherMotor.getVelocity() - 800) > 50) {
            sleep(90);
        }*/
        ringPushServo.setPosition(.2);
        sleep(150);
        ringPushServo.setPosition(.35);
        sleep(450);
        /*ringPushServo.setPosition(.525);
        sleep(750);
        ringPushServo.setPosition(.8);*/
    }

    public void ringIntake() {
        if (gamepad1.left_trigger == 1) {
            topIntakeMotor.setPower(-.9);
            bottomIntakeMotor.setPower(.9);
            liftAngleServo.setPosition(0.1);
        } else if (gamepad1.left_trigger == 0) {
            topIntakeMotor.setPower(0);
            bottomIntakeMotor.setPower(0);
        }

        if (gamepad1.left_bumper) {
            topIntakeMotor.setPower(1);
            bottomIntakeMotor.setPower(-1);
        }
    }

    public void liftControl() {
        if (gamepad1.dpad_up) {
            liftAngleServo.setPosition(.65);
        } else if (gamepad1.dpad_down) {
            liftAngleServo.setPosition(.21);
        }
    }

    /*if (gamepad1.dpad_down) {
            if (!slowDown) {
                speedAdjust = 4;
                slowDown = true;
            }
        } else {
            slowDown = false;
        }*/

    public void wobbleGoalGrabberPivot() {
        // roughly 740 encoder values for a change of 90 degrees
        /*if (gamepad1.y && !pivotedUp) {
            if (yCounter % 2 == 0) {
                grabberPivotServo.setPosition(.365);
            } else {
                grabberPivotServo.setPosition(0);
            }
            pivotedUp = true;
            yCounter++;
        } else if (!gamepad1.y && pivotedUp) {
            pivotedUp = false;
        }*/

        /*if (gamepad2.b) {
            wobbleGrabberMotor.setPower(1);
        } else if (gamepad2.a) {
            wobbleGrabberMotor.setPower(-1);
        } else {
            wobbleGrabberMotor.setPower(0);
        }*/

        if (gamepad1.a) {
            wobbleGrabberMotor.setPower(1);
        } else if (gamepad1.b) {
            wobbleGrabberMotor.setPower(-1);
        } else {
            wobbleGrabberMotor.setPower(0);
        }

        /*if (gamepad2.x) {
            wobbleGrabberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }*/

        /*if (gamepad1.y) {
            wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleGrabberMotor.setPower(-1);
            wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobbleGrabberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }*/

        /*if (gamepad1.a) {
            wobbleGrabberMotor.setTargetPosition(740);
            wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleGrabberMotor.setPower(1);

            while (wobbleGrabberMotor.isBusy()) {

            }
        }*/

        /*if (gamepad1.b && !pivotedUp) {
            if (bCounter % 2 == 0) {
                wobbleGrabberMotor.setTargetPosition(1480);
                wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleGrabberMotor.setPower(1);

                while (wobbleGrabberMotor.isBusy()) {

                }
            } else {
                wobbleGrabberMotor.setTargetPosition(950);
                wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleGrabberMotor.setPower(1);

                while (wobbleGrabberMotor.isBusy()) {

                }
            }
            pivotedUp = true;
            bCounter++;
        } else if (!gamepad1.b && pivotedUp) {
            pivotedUp = false;
        }*/


        /*if (gamepad1.a && !atRest) {
            if (aCounter % 2 == 0) {
                wobbleGrabberMotor.setTargetPosition(0);
                wobbleGrabberMotor.setPower(.5);
            }
            atRest = true;
            aCounter++;
        } else if (!gamepad1.a && atRest) {
            atRest = false;
        }*/
    }

    public void wobbleGoalGrabber() {
        if (gamepad1.x && !grabbed) {
            if (xCounter % 2 == 0) {
                grabberHandServo.setPosition(.75);
            } else {
                grabberHandServo.setPosition(.05);
            }
            grabbed = true;
            xCounter++;
        } else if (!gamepad1.x && grabbed) {
            grabbed = false;
        }
    }

    public void dropOff() {
        wobbleGrabberMotor.setTargetPosition(1200);
        wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGrabberMotor.setPower(1);

        while (wobbleGrabberMotor.isBusy()) {

        }
    }

    public void returnToZero() {
        wobbleGrabberMotor.setTargetPosition(0);
        wobbleGrabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGrabberMotor.setPower(1);

        while (wobbleGrabberMotor.isBusy()) {

        }
    }

    public void ringPush() {
        /*if (gamepad1.dpad_right) {
            ringPushServo.setPosition(.2);
        } else {
            ringPushServo.setPosition(.35);
        }*/

        if (gamepad1.dpad_right) {
            ringPushServo.setPosition(.2);
            sleep(150);
            ringPushServo.setPosition(.35);
            sleep(450);
        } else {
            ringPushServo.setPosition(.35);
        }
    }

    /*public void wobbleArm() {
        if (gamepad1.a) {
            wobbleGrabberMotor.setPower(-.8);
        } else if (gamepad1.b) {
            wobbleGrabberMotor.setPower(.8);
        } else {
            wobbleGrabberMotor.setPower(0);
        }
    }*/

    public void leftPowerShotAim() {
        if (gamepad1.dpad_left) {
            leftTurnNoPID(6);
        }
    }

    public void rightPowerShotAim() {
        if (gamepad1.dpad_down) {
            rightTurnNoPID(0.75);
        }
    }

    public void powerShotAiming() {
        if (gamepad1.dpad_left) {
            strafeRight(80);
            leftTurnNoPID(.5);
            sleep(100);
            autoRingPushTrigger();
            driveStraightForward(5);
            leftTurnNoPID(.5);
            sleep(100);
            strafeRight(50);
            sleep(100);
            autoRingPushTrigger();
            driveStraightForward(5);
            leftTurnNoPID(.5);
            sleep(100);
            strafeRight(50);
            sleep(100);
            autoRingPushTrigger();
        }
    }

    public void taunt() {
        // a tool for those who style hard

        if (gamepad1.back) {
            if (!style) {
                wobbleGrabberMotor.setPower(-.8);
                sleep(200);
                wobbleGrabberMotor.setPower(.8);
                sleep(200);
                wobbleGrabberMotor.setPower(-.8);
                sleep(200);
                wobbleGrabberMotor.setPower(.8);
                sleep(200);
                wobbleGrabberMotor.setPower(-.8);
                sleep(200);
                wobbleGrabberMotor.setPower(.8);
                sleep(200);
                wobbleGrabberMotor.setPower(0);
                style = true;
            }
        } else {
            style = false;
        }
    }
}
