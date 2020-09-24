package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleOpS3 extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

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

        while (opModeIsActive()) {

            if (gamepad1.left_stick_y >= 0.05)
                drive = 1;
            else if (gamepad1.left_stick_y <= 0.05)
                drive = -1;

            if (gamepad1.left_stick_x >= 0.05)
                turn = 1;
            else if (gamepad1.left_stick_x <= 0.05)
                turn = -1;

            if (gamepad1.right_stick_x >= 0.05)
                strafe = 1;
            else if (gamepad1.right_stick_x <= 0.05)
                strafe = -1;

            leftFrontDrive.setPower(limit(drive + turn + strafe));
            leftBackDrive.setPower(limit(drive + turn - strafe));
            rightFrontDrive.setPower(limit(drive - turn + strafe));
            rightBackDrive.setPower(limit(drive - turn - strafe));


        }
    }
}