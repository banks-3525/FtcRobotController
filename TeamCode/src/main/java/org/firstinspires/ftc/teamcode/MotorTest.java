package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MotorTest", group = "")
public class MotorTest extends CommonOpMode {

    @Override
    public void runOpMode() {
        initTestHardware2020();

        waitForStart();

        while (opModeIsActive()) {
            getGeneralTelemetry();
            frontLeftMotor.setPower(1);
            sleep(2000);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(1);
            sleep(2000);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(1);
            sleep(2000);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(1);
            sleep(2000);
            backRightMotor.setPower(0);
        }

    }
}
