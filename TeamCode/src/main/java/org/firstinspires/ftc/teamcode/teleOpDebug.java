package org.firstinspires.ftc.teamcode;

/* Import the cool API modules */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.abs;
import java.lang.Math;
import java.util.Calendar;

/* Program for controller */
@TeleOp(name = "TeleOpS3", group = "T3")
public class teleOpDebug extends LinearOpMode {

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

    float distancePerRotation = 28;
    int shooterPreviousPosition = 0;
    int shooterCurrentPosition = 0;
    float shooterDifference = 0;

    long previousTime = 0;
    long currentTime = 0;
    long elapsedTime = 0;

    Calendar timeCalendar = Calendar.getInstance();

    private double limit(double power) {
        if (power > 1)
            return 1;
        else
            return power;
    }

    @Override
    public void runOpMode() {
        //driving
        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //shooter and flipping
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        rampPusher = hardwareMap.get(CRServo.class, "rampPusher");

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //wobble
        wobbleFlipper = hardwareMap.get(DcMotor.class, "wobbleFlipper");
        wobbleIntake = hardwareMap.get(Servo.class, "wobbleIntake");

        //intake and conveyer
        intakeFirst = hardwareMap.get(DcMotor.class, "intakeFirst");
        twoWheelIntake = hardwareMap.get(CRServo.class, "twoWheelIntake");
        intakeCricket = hardwareMap.get(CRServo.class, "intakeCricket"); //?
        conveyerBelt = hardwareMap.get(DcMotor.class, "conveyerBelt");
        conveyerServo = hardwareMap.get(CRServo.class, "conveyerServo");

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

        conveyerBelt.setPower(0);
        shooter.setPower(0);
        intakeCricket.setPower(0);
        intakeFirst.setPower(0);


        while (opModeIsActive()) {

            //encoders section

            shooterPreviousPosition = shooterCurrentPosition;
            shooterCurrentPosition = shooter.getCurrentPosition();
            shooterDifference = (-shooterCurrentPosition) - (-shooterPreviousPosition);
            float turnDifference = shooterDifference/distancePerRotation;

            timeCalendar = Calendar.getInstance();

            previousTime = currentTime;
            currentTime = timeCalendar.getTimeInMillis();
            elapsedTime = currentTime - previousTime;

            float elapsedTimeFloat = (float)elapsedTime;

            float shooterSpeed = (turnDifference/elapsedTimeFloat)*60000;

            telemetry.addData("Distance", shooterCurrentPosition);
            telemetry.addData("Shooter RPM", shooterSpeed);
            telemetry.addData("Shooter Change", shooterDifference);

            telemetry.update();

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

            //ramp
            if (gamepad2.b) {
                rampPusher.setPower(1);
            }
            else {
                rampPusher.setPower(0);
            }
            /*
            //intake
            if (gamepad2.right_stick_x) {

                intakeFirst.setPower(1);
                twoWheelIntake.setPower(-1);
                intakeCricket.setPower(-1);
                conveyerServo.setPower(-1);

            }
            else {
                intakeFirst.setPower(0);
                intakeCricket.setPower(0);
                twoWheelIntake.setPower(0);
                conveyerServo.setPower(0);
            }
            */

            if (gamepad2.right_stick_y > 0) {
                intakeFirst.setPower(1);
            }
            else if (gamepad2.right_stick_y < 0) {
                intakeFirst.setPower(-1);
            }

            if (gamepad2.left_stick_y > 0) {
                intakeCricket.setPower(1);
                conveyerServo.setPower(1);
            }

            if (gamepad2.left_stick_y < 0) {
                intakeCricket.setPower(-1);
                conveyerServo.setPower(-1);
            }

            if (gamepad2.dpad_up)
                conveyerBelt.setPower(-1);
            }

            if (gamepad2.dpad_down) {
                conveyerBelt.setPower(1);
            }

            else {
                conveyerBelt.setPower(0);
            }

            //wobble
            if (gamepad2.x) {
                wobbleFlipper.setPower(0.75);
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

            // Combined into one if else to stop speed issues
            //for some reason doesnt spin as fast as gamepad1.a
            if (gamepad2.a) {
                shooter.setPower(-0.75);
            }

            if (gamepad2.b)
            {
                shooter.setPower(0.75);
            }
            //faster than gamepad2.left_bumper (but its not supposed to be)
            else if (gamepad1.a) {
                shooter.setPower(-0.65);
            }
            else {
                shooter.setPower(0);
            }

            idle();
        }
    }
}
