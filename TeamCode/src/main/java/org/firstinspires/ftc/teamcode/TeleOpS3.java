package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


    private double limit (double power){
        if (power > 1)
            return 1;
        else
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

            if (gamepad1.left_stick_y >= 0.05)
                drive = 1;

            else if (gamepad1.left_stick_y <= -0.05)
                drive = -1;

            if (gamepad1.left_stick_x >= 0.05)
                turn = 1;

            else if (gamepad1.left_stick_x <= -0.05)
                turn = -1;

            if (gamepad1.right_stick_x >= 0.05)
                strafe = 1;

            else if (gamepad1.right_stick_x <= -0.05)
                strafe = -1;

            leftFrontDrive.setPower(limit(drive + turn + strafe));
            leftBackDrive.setPower(limit(drive + turn - strafe));
            rightFrontDrive.setPower(limit(drive - turn + strafe));
            rightBackDrive.setPower(limit(drive - turn - strafe));

            //Shooter commands.
            if (gamepad2.a) {
                intakeTurner.setPower(1);
                
            } else if (gamepad2.b) {
                intakeTurner.setPower(-1);
            }

            if (gamepad2.left_bumper) {
                conveyerBelt.setPower(1);
            }

            if (gamepad2.right_bumper) {
                shooter.setPower(1);
            }
        }
    }
}