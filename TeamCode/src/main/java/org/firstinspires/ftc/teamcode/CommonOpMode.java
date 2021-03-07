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

    private final ElapsedTime runtime = new ElapsedTime();
    double speedAdjust = 8;
    boolean speedUp = false;
    boolean slowDown = false;
    boolean style = false;

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
    public DcMotor ringLauncherMotor;
    public DcMotor topIntakeMotor;
    public DcMotor bottomIntakeMotor;
    public DcMotor wobbleGrabberMotor;

    public Servo ringPushServo;
    public Servo liftAngleServo;
    public Servo grabberPivotServo;
    public Servo wobbleGrabber;

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
        ringLauncherMotor = hardwareMap.dcMotor.get("RING");
        topIntakeMotor = hardwareMap.dcMotor.get("TIM");
        bottomIntakeMotor = hardwareMap.dcMotor.get("BIM");
        wobbleGrabberMotor = hardwareMap.dcMotor.get("WGM");

        liftAngleServo = hardwareMap.servo.get("LAS");
        ringPushServo = hardwareMap.servo.get("RPS");

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
        //telemetry.addData("Angle Reading:", getAngle());
        telemetry.addData("Correction:", correction);
        //telemetry.addData("Back Left Encoders:", backLeftMotor.getCurrentPosition());
        //telemetry.addData("Front Left Encoders:", frontLeftMotor.getCurrentPosition());
        //telemetry.addData("Back Right Encoders:", backRightMotor.getCurrentPosition());
        //telemetry.addData("Front Right Encoders:", frontRightMotor.getCurrentPosition());
        //telemetry.addData("IncrementLevel", increment);
        //telemetry.addData("speed adjust", "%.2f", speedAdjust);
        telemetry.update();
    }

    public boolean joysticksActive() {
        return (gamepad1.left_stick_y != 0) && (gamepad1.left_stick_x != 0) && (gamepad1.right_stick_x != 0);
    }

    public void robotCentricDrive() {
        double yAxis;
        double xAxis;
        double strafe;

        correction = pidStrafe.performPID(getAngle());

        yAxis = gamepad1.left_stick_y;
        xAxis = gamepad1.left_stick_x;
        strafe = gamepad1.right_stick_x;
        //DON'T CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        backLeftMotor.setPower((-yAxis + xAxis /*+ strafe*/) * (-speedAdjust / 10));
        frontLeftMotor.setPower((yAxis + xAxis /*- strafe*/) * (-speedAdjust / 10));
        backRightMotor.setPower((yAxis + xAxis /*+ strafe*/) * (-speedAdjust / 10));
        frontRightMotor.setPower((-yAxis + xAxis /*- strafe*/) * (-speedAdjust / 10));

        if (strafe == 1 || strafe == -1) {
            backLeftMotor.setPower((strafe * (-speedAdjust / 10)) + correction);
            frontLeftMotor.setPower(-(strafe * (-speedAdjust / 10)) + correction);
            backRightMotor.setPower((strafe * (-speedAdjust / 10)) + correction);
            frontRightMotor.setPower(-(strafe * (-speedAdjust / 10)) + correction);
        }

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

    public void resetAlignment() {
        correction = pidDrive.performPID(getAngle());

        if (gamepad1.b) {
            frontRightMotor.setPower(correction);
            frontLeftMotor.setPower(correction);
            backLeftMotor.setPower(correction);
            backRightMotor.setPower(correction);
        }
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
        pidDrive = new PIDController(.025, 0, 0);
        pidStrafe = new PIDController(.1, 0, 0);
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
            backLeftMotor.setPower(.6);
            frontLeftMotor.setPower(.6);
            backRightMotor.setPower(.6);
            frontRightMotor.setPower(.6);
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
        telemetry.addData("frp", frontRightMotor.getPower());
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

    public void ringLauncher() {
        if (gamepad1.right_trigger == 1) {
            ringLauncherMotor.setPower(1);
            liftAngleServo.setPosition(.475);
        } else if (gamepad1.right_trigger == 0) {
            ringLauncherMotor.setPower(0);
        }
    }

    public void autoRingPushTrigger() {
        ringPushServo.setPosition(.525);
        sleep(750);
        ringPushServo.setPosition(.8);
    }

    public void ringIntake() {
        if (gamepad1.left_trigger == 1) {
            topIntakeMotor.setPower(-.8);
            bottomIntakeMotor.setPower(.8);
            liftAngleServo.setPosition(.145);
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

    public void wobbleGoalGrabberPivot() {
        if (gamepad1.a) {
            grabberPivotServo.setPosition(-.5);
        } else if (gamepad1.b) {
            grabberPivotServo.setPosition(-.15);
        }

    }

    public void wobbleGoalGrabber() {
        if (gamepad1.x) {
            wobbleGrabber.setPosition(.5);
        } else if (gamepad1.y) {
            wobbleGrabber.setPosition(.15);
        }
    }

    public void ringPush() {
        if (gamepad1.dpad_right) {
            ringPushServo.setPosition(.525);
        } else {
            ringPushServo.setPosition(.7);
        }
        /*else if (gamepad1.dpad_left) {
            ringPushServo.setPosition(.15);
        }*/
    }

    public void wobbleArm() {
        if (gamepad1.y) {
            wobbleGrabberMotor.setPower(-.8);
        } else if (gamepad1.x) {
            wobbleGrabberMotor.setPower(.8);
        } else {
            wobbleGrabberMotor.setPower(0);
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
