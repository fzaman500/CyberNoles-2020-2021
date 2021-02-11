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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlow Auto")
//@Disabled
public class TensorFlowAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AesV96v/////AAABmTw0g0ohBkUSvmgv9G7R1v08xvHe71LdR8CpbHiyOZ7o7L9U2c59Dz2Tze0rDecdXmliGUs0ciGyU70YALutfM1aVvugoWUz3NrLYPj2zMnlKds34CTp5Hc+qX2SldDIPLV3vN33foMd5lzUlS4F5r+SY1vn9YaCbWB2hpsCI0MIVsmOLCqkpNt2/QUpLGG+SOfEy3xVKbikdRWE1XDSLGVMK2/YWxT/GF65AuNYLw5VoW0AhbVmV+b+17114UFgoiVjVnOdA+n4Cv7jhQo3JmPcmCf2Gd6gnmvehSCZdsChB42fgmfoP4V9xpFSWQ2hkAq2+BsH7+hmBtF+XMNWd5xVCWJBiA8o/SCnKANzUA7F";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor intakeFirst = null;
    private CRServo intakeCricket = null;
    private CRServo rampPusher = null;
    private DcMotor conveyerBelt = null;
    private DcMotor shooter = null;
    private DcMotor wobbleFlipper = null;
    private Servo wobbleIntake = null;
    private CRServo twoWheelIntake = null;
    private CRServo conveyerServo = null;

    double[][] directions = {
            {1, -1, -1, 1},   /* up     */
            {-1, 1, 1, -1},   /* down     */
            {-1, -1, 1, 1},   /* left     */
            {1, 1, -1, -1},   /* right     */
    };

    public void move(String direction) {
        if (!direction.equals("none")) {
            int d = 0;
            if (direction.equals("forward"))
                d = 0;
            else if (direction.equals("backward"))
                d = 1;
            else if (direction.equals("left"))
                d = 2;
            else if (direction.equals("right"))
                d = 3;
            //regular
            /*motorFrontLeft.setPower((directions[d][0]));
            motorFrontRight.setPower((directions[d][1]));
            motorBackLeft.setPower((directions[d][2]));
            motorBackRight.setPower((directions[d][3]));*/

            //gyro
            motorFrontLeft.setPower((directions[d][0]) - correction);
            motorFrontRight.setPower((directions[d][1]) + correction);
            motorBackLeft.setPower((directions[d][2]) - correction);
            motorBackRight.setPower((directions[d][3]) + correction);
        }
    }


    public void moveUntilTime(String direction, int time){
        move(direction);
        double debounce = runtime.seconds() + 0.0;
        while (debounce + (time / 1000.0) > runtime.seconds() && opModeIsActive()) {}

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void turnUntilTime(String direction, int time){
        int d = 0;
        if (!direction.equals("none")) {
            if (direction.equals("left"))
                d = -1;
            else if (direction.equals("right"))
                d = 1;
        }
        //regular
            /*motorFrontLeft.setPower((directions[d][0]));
            motorFrontRight.setPower((directions[d][1]));
            motorBackLeft.setPower((directions[d][2]));
            motorBackRight.setPower((directions[d][3]));*/

        //gyro
        motorFrontLeft.setPower(d);
        motorFrontRight.setPower(d);
        motorBackLeft.setPower(d);
        motorBackRight.setPower(d);
        double debounce = runtime.seconds() + 0.0;
        while (debounce + (time / 1000.0) > runtime.seconds() && opModeIsActive()) {}

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    private double limit(double power) {
        if (power > 1)
            return 1;
        else
            return power;
    }


    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }
        //driving
        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //shooter and flipping
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        rampPusher = hardwareMap.get(CRServo.class, "rampPusher");

        //wobble
        wobbleFlipper = hardwareMap.get(DcMotor.class, "wobbleFlipper");
        wobbleIntake = hardwareMap.get(Servo.class, "wobbleIntake");

        //intake and conveyer
        intakeFirst = hardwareMap.get(DcMotor.class, "intakeFirst");
        twoWheelIntake = hardwareMap.get(CRServo.class, "twoWheelIntake");
        intakeCricket = hardwareMap.get(CRServo.class, "intakeCricket"); //?
        conveyerBelt = hardwareMap.get(DcMotor.class, "conveyerBelt");
        conveyerServo = hardwareMap.get(CRServo.class, "conveyerServo");


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                wobbleIntake.setPosition(1); //hold
                sleep(2000);
                rampPusher.setPower(1); //push
                sleep(4000);
                rampPusher.setPower(0);
                sleep(2000);
                shooter.setPower(-1); //on shooter
                moveUntilTime("forward", 750); // go up to discs

                //scan discs
                sleep(1000);
                String disc_number = discs(5);
                if (disc_number == "Quad") {
                    telemetry.addData("Yay", disc_number);
                    telemetry.update();
                }
                else if (disc_number == "Single") {
                    telemetry.addData("Yay", disc_number);
                    telemetry.update();
                }


                if (disc_number.equals("Quad")) {
                    moveUntilTime("forward", 2250);
                    moveUntilTime("right", 2000);
                    turnUntilTime("left", 100);
                    sleep(2000);
                    wobbleFlipper.setPower(-1); //move wobble
                    sleep(1000);
                    wobbleFlipper.setPower(0);
                    sleep(1000);
                    wobbleFlipper.setPower(1); //move wobble
                    sleep(1000);
                    wobbleFlipper.setPower(0);
                    sleep(1000);
                    wobbleIntake.setPosition(0); //let go
                    sleep(1000);
                    moveUntilTime("backward", 1600);
                }
                else if (disc_number.equals("Single")) {
                    moveUntilTime("forward", 1500);
                    sleep(2000);
                    wobbleFlipper.setPower(-1); //move wobble
                    sleep(1000);
                    wobbleFlipper.setPower(0);
                    sleep(1000);
                    wobbleIntake.setPosition(0); //let go
                    sleep(1000);
                    wobbleFlipper.setPower(1); //move wobble
                    sleep(1000);
                    wobbleFlipper.setPower(0);
                    moveUntilTime("backward", 750);
                    moveUntilTime("right", 2000);
                }
                else {
                    moveUntilTime("forward", 750);
                    moveUntilTime("right", 2000);
                    sleep(2000);
                    wobbleFlipper.setPower(-1); //move wobble
                    sleep(1000);
                    wobbleFlipper.setPower(0);
                    sleep(1000);
                    wobbleIntake.setPosition(0); //let go
                    sleep(1000);
                    moveUntilTime("backward", 250);
                }
                sleep(1000);
                //moveUntilTime("right", 500);
                conveyerBelt.setPower(.9);
                sleep(1000);
                conveyerBelt.setPower(0);
                sleep(750);
                conveyerBelt.setPower(.9);
                sleep(750);
                conveyerBelt.setPower(0);
                sleep(750);
                conveyerBelt.setPower(.9);
                sleep(750);
                conveyerBelt.setPower(0);
                //moveUntilTime("left", 500);
                shooter.setPower(0);
                moveUntilTime("forward", 300);

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
        parameters.cameraDirection = CameraDirection.BACK;

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

    public String discs(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        String disc_number;
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLabel() == "Single")
                            return "Single";
                        else if (recognition.getLabel() == "Quad")
                            return "Quad";
                    }
                    telemetry.update();
                }
            }
        }
        return "None";
    }
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
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
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .50;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}
//Test