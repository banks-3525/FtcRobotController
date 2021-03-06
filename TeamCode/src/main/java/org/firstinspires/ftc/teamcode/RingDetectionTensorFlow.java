/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


@Autonomous(name = "RingDetectionTensorFlow", group = "")
public class RingDetectionTensorFlow extends CommonOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AZkEOOb/////AAABmSRzOCZ6YkL/p6G4Xnxjo66DEIymaDOykw3T5U4u01vc1jTmNeV636h2PblftzkHyuQAbhJ9xbfeguTRP0lNOB0DpHcTKPpG5Sui1iJvTkzSjH5qwHT/T70r4TfoI5KvPHZLN7fX8d4DiWoglOXPma39CHFZdW7pM2PjWn+SjAG3gBFEfPf95S9talZg2Gc43eWaf5ecZX+Bw5JtNQ1c9N3DCZtPtjBAcrfgVeI6N3CjNl9r2pYonr2TnqgeA1yQYLtJJrz/eoPwhT+B8ryCDxq3pB5waj5QAIeswkC7Dy408NDn0O7utKaXwQrb3QMKdM7OOxdsioiJ7TtZrYb/a++8/GVfRBjdcIlW5a4v1VCC";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        initHardware2020();
        initPID();

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberPivotServo.setPosition(0);
        grabberHandServo.setPosition(.75);
        liftAngleServo.setPosition(.675);
        // original is .675

        if (tfod != null) {
            tfod.activate();

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        setupPIDParameters();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        if (updatedRecognitions.size() == 0) {
                            // in the future, we'll want this code to move
                            // the robot and Wobble Goal
                            // to the closest delivery square.

                            // plan of attack:
                            // scan the rings,
                            // drive straight to the closest square,
                            // deliver the Wobble Goal,
                            // drive backwards till the robot is atop the launch line,
                            // stop the robot.

                            resetAngle();
                            ringLauncherMotor.setPower(1);
                            ringLauncherMotor.setVelocity(targetRPM);
                            driveStraightForward(10);
                            sleep(100);
                            strafeLeft(125);
                            sleep(100);
                            driveStraightForward(220);
                            sleep(100);
                            strafeRight(150);
                            //leftTurnNoPID(.5);
                            sleep(3000);
                            autoRingPushTrigger();
                            autoRingPushTrigger();
                            autoRingPushTrigger();
                            sleep(100);
                            driveStraightForward(70);
                            sleep(100);
                            strafeRight(45);
                            sleep(100);
                            dropOff();
                            sleep(1000);
                            grabberHandServo.setPosition(.05);
                            sleep(500);
                            wobbleGrabberMotor.setPower(-1);
                            sleep(750);
                            returnToZero();
                            strafeLeft(80);
                            driveStraightBackward(20);
                            rightTurnNoPID(0);
                            break;
                        } else {
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            /*telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());*/


                                // to preface, this code is made with the assumption
                                // that the robot starts on the side closest to the wall.
                                if (recognition.getLabel().equals("Quad")) {
                                    // in the future, we'll want this code to move
                                    // the robot and Wobble Goal
                                    // to the farthest delivery square.

                                    // plan of attack:
                                    // scan the rings,
                                    // drive straight to the farthest square,
                                    // deliver the Wobble Goal,
                                    // drive backwards till the robot is atop the launch line,
                                    // stop the robot.

                                    resetAngle();
                                    ringLauncherMotor.setPower(1);
                                    ringLauncherMotor.setVelocity(targetRPM);
                                    driveStraightForward(10);
                                    sleep(100);
                                    strafeLeft(125);
                                    sleep(100);
                                    driveStraightForward(225);
                                    sleep(100);
                                    strafeRight(150);
                                    //leftTurnNoPID(.5);
                                    sleep(3000);
                                    autoRingPushTrigger();
                                    autoRingPushTrigger();
                                    autoRingPushTrigger();
                                    sleep(100);
                                    driveStraightForward(210);
                                    sleep(100);
                                    strafeRight(60);
                                    sleep(100);
                                    leftTurnNoPID(50);
                                    sleep(100);
                                    dropOff();
                                    sleep(1000);
                                    grabberHandServo.setPosition(.05);
                                    sleep(500);
                                    wobbleGrabberMotor.setPower(-1);
                                    sleep(750);
                                    returnToZero();
                                    strafeLeft(160);
                                    sleep(1000);
                                    rightTurnNoPID(0);
                                    driveStraightBackward(110);
                                    rightTurnNoPID(0);

                                    telemetry.addData("It's a quad stack.", "4");
                                    telemetry.update();
                                    break;
                                } else if (recognition.getLabel().equals("Single")) {
                                    // in the future, we'll want this code to move
                                    // the robot and Wobble Goal
                                    // to the second-closest delivery square.

                                    // plan of attack:
                                    // scan the rings,
                                    // drive straight to where the middle square should be (wall-side),
                                    // strafe to the middle square,
                                    // deliver the Wobble Goal,
                                    // drive backwards till the robot is atop the launch line,
                                    // stop the robot.

                                    resetAngle();
                                    ringLauncherMotor.setPower(1);
                                    ringLauncherMotor.setVelocity(targetRPM);
                                    driveStraightForward(10);
                                    sleep(100);
                                    strafeLeft(125);
                                    sleep(100);
                                    driveStraightForward(225);
                                    sleep(100);
                                    strafeRight(150);
                                    //leftTurnNoPID(.5);
                                    sleep(3000);
                                    autoRingPushTrigger();
                                    autoRingPushTrigger();
                                    autoRingPushTrigger();
                                    sleep(100);
                                    strafeLeft(125);
                                    sleep(500);
                                    driveStraightForward(160);
                                    sleep(1000);
                                    dropOff();
                                    sleep(1000);
                                    grabberHandServo.setPosition(.05);
                                    sleep(500);
                                    wobbleGrabberMotor.setPower(-1);
                                    sleep(750);
                                    returnToZero();
                                    driveStraightBackward(120);
                                    rightTurnNoPID(0);
                                    break;
                                }
                            }

                            telemetry.update();
                        }
                    }
                }

            break;
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
