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


    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;;
    public DcMotor ringLauncher;
    public DcMotor topIntakeMotor;
    public DcMotor bottomIntakeMotor;


    static final int CYCLE_MS = 300;

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

    public void initTestHardware2020() {
        //frontLeftMotor = hardwareMap.dcMotor.get("fr");
        //backLeftMotor = hardwareMap.dcMotor.get("bl");
       // frontRightMotor = hardwareMap.dcMotor.get("fl");
       // backRightMotor = hardwareMap.dcMotor.get("br");
        ringLauncher = hardwareMap.dcMotor.get("Rrm");
        topIntakeMotor = hardwareMap.dcMotor.get("TIM");
        bottomIntakeMotor = hardwareMap.dcMotor.get("BIM");

        //frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        //backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
    }


    public void initHardware() {
        frontLeftMotor = hardwareMap.dcMotor.get("frm");
        backLeftMotor = hardwareMap.dcMotor.get("blm");
        frontRightMotor = hardwareMap.dcMotor.get("flm");
        backRightMotor = hardwareMap.dcMotor.get("brm");
        //rightSuctionMotor = hardwareMap.dcMotor.get("lsm");
        //rightSuctionMotor = hardwareMap.dcMotor.get("rsm");
        rightColorSensor = hardwareMap.get(ColorSensor.class, "RightColorSensor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
            setupMotorToRunWithoutEncoder(frontRightMotor);
        }
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
        backLeftMotor.setPower((yAxis + xAxis - turn) * (-speedAdjust / 10));
        frontLeftMotor.setPower((yAxis - xAxis - turn) * (-speedAdjust / 10));
        backRightMotor.setPower((yAxis - xAxis + turn) * (-speedAdjust / 10));
        frontRightMotor.setPower((yAxis + xAxis + turn) * (-speedAdjust / 10));
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

    public void runWithoutEncoders() {
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

    public void setDriveY(int y) {
        y = (int) ((y * 1120) / 31.4);
        frontLeftMotor.setTargetPosition(y);
        backLeftMotor.setTargetPosition(y);
        frontRightMotor.setTargetPosition(y);
        backRightMotor.setTargetPosition(y);
    }

    public void setDriveX(int x) {
        x = (int) ((x * 1120) / 31.4);
        frontLeftMotor.setTargetPosition(x);
        backLeftMotor.setTargetPosition(-x);
        frontRightMotor.setTargetPosition(-x);
        backRightMotor.setTargetPosition(x);
    }

    public void setDriveRotate(int r) {
        frontLeftMotor.setTargetPosition(r);
        backLeftMotor.setTargetPosition(r);
        frontRightMotor.setTargetPosition(-r);
        backRightMotor.setTargetPosition(-r);
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

    /*public void driveAutoForward() {
        if(leftTouchSensor.isPressed() || rightTouchSensor.isPressed())
        blm.setPower(.3);
        flm.setPower(.3);
        brm.setPower(.3);
        frm.setPower(.3);
    }*/

    public void blockStrafeLeft() {
        frontLeftMotor.setPower(.2);
        backLeftMotor.setPower(-.4);
        frontRightMotor.setPower(-.4);
        backRightMotor.setPower(.2);
    }

    public void blockStrafeRight() {
        frontLeftMotor.setPower(-.6);
        backLeftMotor.setPower(.3);
        frontRightMotor.setPower(.3);
        backRightMotor.setPower(-.6);
    }


    public void strafeLeftAuto() {
        driveAuto(0, 1, 0, .5, 7);
    }

    public void strafeRightAuto() {
        driveAuto(0, -1, 0, .5, -13);
    }


    public boolean distanceDone(double target) {
        return (abs(frontLeftMotor.getCurrentPosition()) <= abs(target)) && (abs(backLeftMotor.getCurrentPosition()) <= abs(target))
                && (abs(frontRightMotor.getCurrentPosition()) <= abs(target))
                && (abs(backRightMotor.getCurrentPosition()) <= abs(target));

    }

    public void distanceDrive(double distance_cm) {
        double target = (int) ((distance_cm * 1120) / 31.4);
        resetDrive();
        while (opModeIsActive() && (/*sensorRange.getDistance(DistanceUnit.CM) > 4 ||*/ distanceDone(target))) {
            //telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("distance met", "%s", distanceDone(target));
            telemetry.update();
            frontLeftMotor.setPower(-1);
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            frontRightMotor.setPower(-1);
        }
        stopDriveMotors();
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

    public void autoDriveBackwardsIndefinitely() {
        resetDriveWithoutEncoder();
        correction = pidDrive.performPID(getAngle());
        backLeftMotor.setPower(-(pidPower + (correction / 2)));
        frontLeftMotor.setPower(-(pidPower + (correction / 2)));
        backRightMotor.setPower(-(pidPower - (correction / 2)));
        frontRightMotor.setPower(-(pidPower - (correction / 2)));
    }

    public void autoDriveForwardsIndefinitelySlowly() {
        frontLeftMotor.setPower(.15);
        backLeftMotor.setPower(.15);
        frontRightMotor.setPower(.15);
        backRightMotor.setPower(.15);
    }

   /* public boolean checkSkystone() {
        //100 is the deciding constant that determines either stone or Skystone
        return abs(leftColorSensor.red() - rightColorSensor.red()) > 100;
    }*/

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
        while (abs(frontLeftMotor.getCurrentPosition()) <= abs(radian) && opModeIsActive()) {
            backLeftMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
            frontRightMotor.setPower(power);
        }
        stopDriveMotors();
    }


    public void leftTurn(double power, double radian) {
        resetDrive();
        while (abs(frontLeftMotor.getCurrentPosition()) <= abs(radian) && opModeIsActive()) {
            backLeftMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
            frontRightMotor.setPower(-power);
        }
        stopDriveMotors();
    }

    public void driveStraightForward(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontLeftMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower((pidPower - (correction / 2)));
            frontLeftMotor.setPower(-(pidPower - (correction / 2)));
            backRightMotor.setPower((pidPower - (correction / 2)));
            frontRightMotor.setPower(-(pidPower - (correction / 2)));
        }
        stopDriveMotors();
    }

    public void driveStraightBackward(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontLeftMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower(-(pidPower + (correction / 2)));
            frontLeftMotor.setPower(-(pidPower + (correction / 2)));
            backRightMotor.setPower(-(pidPower - (correction / 2)));
            frontRightMotor.setPower(-(pidPower - (correction / 2)));
        }
        stopDriveMotors();
    }

    public void strafeLeft(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontLeftMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower(pidPower - (correction / 2));
            frontLeftMotor.setPower(-pidPower - (correction / 2));
            backRightMotor.setPower(-pidPower + (correction / 2));
            frontRightMotor.setPower(pidPower + (correction / 2));
        }
        stopDriveMotors();
    }

    public void strafeRight(int distance_cm) {
        double distance_encoder = (int) ((distance_cm * 383.6) / 31.4);

        resetDriveWithoutEncoder();

        while (abs(frontLeftMotor.getCurrentPosition()) <= abs(distance_encoder) && opModeIsActive()) {
            correction = pidDrive.performPID(getAngle());
            telemetryPID();
            backLeftMotor.setPower(-pidPower - (correction / 2));
            frontLeftMotor.setPower(pidPower - (correction / 2));
            backRightMotor.setPower(pidPower + (correction / 2));
            frontRightMotor.setPower(-pidPower + (correction / 2));
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

    public void driveForwardIndefinitlyWithPID() {
        resetDriveWithoutEncoder();
        correction = pidDrive.performPID(getAngle());
        backLeftMotor.setPower((pidPower - (correction / 2)));
        frontLeftMotor.setPower((pidPower - (correction / 2)));
        backRightMotor.setPower((pidPower - (correction / 2)));
        frontRightMotor.setPower((pidPower - (correction / 2)));
    }

    public void ringLauncherPrototype() {
        if  (gamepad1.right_bumper) {
            ringLauncher.setPower(-.5);
        } else if (gamepad1.left_bumper) {
            ringLauncher.setPower(0);
        }
    }
    public void ringIntake() {
        if (gamepad1.x) {
            topIntakeMotor.setPower(1);
            bottomIntakeMotor.setPower(1);
        } else if (gamepad1.y) {
            topIntakeMotor.setPower(0);
            bottomIntakeMotor.setPower(0);
        }
    }
}
