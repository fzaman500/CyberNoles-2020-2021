package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Comp1S3", group = "T3")
public class TeleOpS3 extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private CRServo intakeTurner = null;
    private DcMotor conveyerBelt = null;
    private DcMotor shooter = null;

    private double drive = 0;
    private double turn = 0;
    private double strafe = 0;


    private double limit(double power) {
        if (power > 1) {
            return 1;
        } else
            return power;
    }

    @Override
    public void runOpMode() {

        telemetry.addData(">", "Press Start To Run TeleOp");
        telemetry.update();

        waitForStart();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        intakeTurner = hardwareMap.get(CRServo.class, "intakeTurner");
        conveyerBelt = hardwareMap.get(DcMotor.class, "conveyerBelt");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        while (opModeIsActive()) {
            //Drivetrain commands.
            //Made changes to the conditions for reverse.

            if (gamepad1.left_stick_y >= 0.05) {
                drive = 1;
            } else if (gamepad1.left_stick_y <= -0.05) {
                drive = -1;
            }

            if (gamepad1.left_stick_x >= 0.05) {
                turn = 1;
            } else if (gamepad1.left_stick_x <= -0.05) {
                turn = -1;
            }

            if (gamepad1.right_stick_x >= 0.05) {
                strafe = 1;
            } else if (gamepad1.right_stick_x <= -0.05) {
                strafe = -1;
            }

            leftFrontDrive.setPower(limit(drive + turn + strafe));
            leftBackDrive.setPower(limit(drive + turn - strafe));
            rightFrontDrive.setPower(limit(drive - turn + strafe));
            rightBackDrive.setPower(limit(drive - turn - strafe));

            //Shooter commands.
            //int degreeTurn = 0; //To be changed to a new value based on angle between lower position
            //and conveyer belt.
            boolean position = true; //True if arm is at lower position.

            /*
            if (gamepad2.a) {
                if (position)
                {
                    servoMotor.rotate(degreeTurn);
                    position = false;
                }
                else
                {
                    servoMotor.rotate(-degreeTurn);
                    position = true;
                }
                **/

            if (gamepad2.a) {
                if (position) {
                    intakeTurner.setPower(1);
                    position = false;

                } else {
                    intakeTurner.setPower(-1);
                    position = true;
                }
            }

                boolean beltRunning = false; //True if conveyer belt is running.

                if (gamepad2.left_bumper) {

                    if (beltRunning = false) {
                        conveyerBelt.setPower(1);
                        beltRunning = true;

                    } else {
                        conveyerBelt.setPower(0);
                        beltRunning = false;
                    }
                }

                boolean shooterRunning = false; //True if shooter is running.

                if (gamepad2.right_bumper) {

                    if (shooterRunning = false) {
                        shooter.setPower(1);
                        shooterRunning = true;

                    } else {
                        shooter.setPower(0);
                        shooterRunning = false;
                    }
                }
            }
        }
    }