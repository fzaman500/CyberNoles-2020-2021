package org.firstinspires.ftc.teamcode;

/* Import the cool API modules */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.abs;

/* Program for controller */
@TeleOp(name = "TeleOpS3", group = "T3")
public class teleOpDebug extends LinearOpMode {

    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor intakeFirst = null;
    private CRServo intakeTurner = null;
    private CRServo intakeFlipper = null;
    private DcMotor shooter = null;
    private DcMotor wobbleFlipper = null;
    private Servo wobbleIntake = null;

    private double limit(double power) {
        if (power > 1)
            return 1;
        else
            return power;
    }

    @Override
    public void runOpMode() {
        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightBackDrive");
        intakeFirst = hardwareMap.get(DcMotor.class, "intakeFirst");
        intakeTurner = hardwareMap.get(CRServo.class, "intakeTurner");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intakeFlipper = hardwareMap.get(CRServo.class, "intakeFlipper");
        wobbleFlipper = hardwareMap.get(DcMotor.class, "wobbleFlipper");
        wobbleIntake = hardwareMap.get(Servo.class, "wobbleIntake");


        double maxPower = 1;

        double[][] directions = {
                {-1, 1, 1, -1},   /* up     */
                {1, -1, -1, 1},   /* down     */
                {1, 1, -1, -1},   /* left     */
                {-1, -1, 1, 1}   /* right     */
        };
//turnin forward turns right
        //classic
        telemetry.addData(">", "Press Start To Run TeleOp");
        telemetry.update();

        /* Doesn't start input events until program intializes */
        waitForStart();

        shooter.setPower(0);
        intakeTurner.setPower(0);
        intakeFirst.setPower(0);


        while (opModeIsActive()) {

            /* This inverts the joystick inputs,
             * since the joystick inputs are gay. */
            /* Up, down, left, and right on the joystick are
             * not static values. It's not just "joystick is up
             * or joystick isn't up", the joystick can be
             * halfway up, causing the value to be 0.5 (to represent half),
             * and making the robot move at half the speed. Also,
             * if the joystick is all the way to the right and you
             * move it upwards, the value for right will decrease
             * even if you don't move the joystick left, since the
             * joystick moves in a circular movement, not a square. */
            double leftStickX = -gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = -gamepad1.right_stick_x;

            /* These variables are placeholders for the values
             * we will set the power of the motors to. We will
             * use these are a medium for our calculations.*/
            double FLpower = 0.f;
            double FRpower = 0.f;
            double BLpower = 0.f;
            double BRpower = 0.f;

            double totalPower = abs(leftStickX) + abs(leftStickY);

            /* This only makes the code move the robot if the
             * LEFT joystick is being used. This section of the
             * code will be used for movement (strafing) of the
             * robot, using ONLY the LEFT joystick. */
            if (leftStickX != 0 || leftStickY != 0) {
                /* These are placeholders for the directions
                 * that are actually being inputted, which will
                 * be used for the instructions for the directions
                 * used in the beginning of the code. */
                int dir1 = 0;
                int dir2 = 0;

                /* This checks where the joystick is in an x and
                 * y location. This is used to assign values for
                 * the direction placeholders.*/
                if (leftStickX > 0) {
                    dir1 = 3;
                } else if (leftStickX < 0) {
                    dir1 = 2;
                }
                if (leftStickY > 0) {
                    dir2 = 1;
                } else if (leftStickY < 0) {
                    dir2 = 0;
                }

                /* Since movement is not static, as said earlier, the
                 * instructions for the directions that the joystick
                 * is at are applied to the motor's power by a certain
                 * amount, depending on how far the joystick is pressed.
                 * For example, pushing the joystick halfway up will
                 * only using the instructions for "up" at a half amount,
                 * resulting in the robot moving forward at half speed.*/
                FLpower += limit(Math.pow((directions[dir1][0] * totalPower * abs(leftStickX)) + (directions[dir2][0] * totalPower * abs(leftStickY)), 3));
                FRpower += limit(Math.pow((directions[dir1][1] * totalPower * abs(leftStickX)) + (directions[dir2][1] * totalPower * abs(leftStickY)), 3));
                BLpower += limit(Math.pow((directions[dir1][2] * totalPower * abs(leftStickX)) + (directions[dir2][2] * totalPower * abs(leftStickY)), 3));
                BRpower += limit(Math.pow((directions[dir1][3] * totalPower * abs(leftStickX)) + (directions[dir2][3] * totalPower * abs(leftStickY)), 3));
            }

            if (abs(rightStickX) > 0.05) {
                /* This code changes the power of each wheel by the same
                 * amount, depending on how far the RIGHT joystick is
                 * pressed, which will rotate the robot at a certain speed. */
                //{-1, 1, -1, 1},   /* up     */
                if (leftStickX != 0 || leftStickY != 0) {
                    FLpower = (FLpower - rightStickX) / 2;
                    FRpower = (FRpower - rightStickX) / 2;
                    BLpower = (BLpower - rightStickX) / 2;
                    BRpower = (BRpower - rightStickX) / 2;
                } else {
                    FLpower = -rightStickX;
                    FRpower = -rightStickX;
                    BLpower = -rightStickX;
                    BRpower = -rightStickX;
                }
            }

            /* Applies maximum power setting */
            FLpower *= maxPower;
            FRpower *= maxPower;
            BLpower *= maxPower;
            BRpower *= maxPower;

            /* Applies the power to motors */
            //telemetry.addData(">", String.valueOf(FLpower));
            motorFrontLeft.setPower(FLpower);
            motorFrontRight.setPower(FRpower);
            motorBackLeft.setPower(BLpower);
            motorBackRight.setPower(BRpower);

            if (gamepad2.a) {
                intakeTurner.setPower(-1);
            }
            else {
                intakeTurner.setPower(0);
            }

            if (gamepad2.b) {
                intakeFlipper.setPower(1);
            }

            if (gamepad2.x) {
                wobbleFlipper.setPower(0.5);
            }

            else if (gamepad2.y) {
                wobbleFlipper.setPower(-0.5);
            }

            else {
                wobbleFlipper.setPower(0);
            }

            if (gamepad1.x && wobbleIntake.getPosition() < 1) {
                wobbleIntake.setPosition(wobbleIntake.getPosition() + 0.1);
            }

            else if (gamepad1.y && wobbleIntake.getPosition() > 0) {
                wobbleIntake.setPosition(wobbleIntake.getPosition() - 0.1);
            }

            if (gamepad2.dpad_up) {
                intakeFirst.setPower(1);
            }

            else if (gamepad2.dpad_down) {
                intakeFirst.setPower(-1);
            }

            else {
                intakeFirst.setPower(0);
            }

            if (gamepad2.left_bumper) {
                shooter.setPower(-1);
            }
            else if (gamepad2.right_bumper) {
                shooter.setPower(1);
            }
            else {
                shooter.setPower(0);
            }

            idle();
        }
    }
}
