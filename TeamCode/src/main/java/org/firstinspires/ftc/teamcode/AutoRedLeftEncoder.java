package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "AutoRedLeftEncoder", group = "Linear OpMode")
public class AutoRedLeftEncoder extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    static final double MOTOR_TICK_COUNT = 1120;
    double quarterTurn = (int)MOTOR_TICK_COUNT/4;

    double[][] directions = {
            {-1, 1, 1, -1},   /* up     */
            {1, -1, -1, 1},   /* down     */
            {1, 1, -1, -1},   /* left     */
            {-1, -1, 1, 1},   /* right     */
            {-1, 1, -1, 1},   /* turn right     */
            {1, -1, 1, -1}   /* turn left     */
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
            else if (direction.equals("turn right")) {
                resetAngle();
                d = 4;
            }
            else if (direction.equals("turn left")) {
                resetAngle();
                d = 5;
            }
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

    public void moveUntilTicks(String direction, int ticks){
        int FRnewTarget = motorFrontRight.getTargetPosition() + ticks;
        int FLnewTarget = motorFrontLeft.getTargetPosition() + ticks;
        int BRnewTarget = motorBackRight.getTargetPosition() + ticks;
        int BLnewTarget = motorBackLeft.getTargetPosition() + ticks;

        move(direction);

        motorFrontRight.setTargetPosition(FRnewTarget);
        motorFrontLeft.setTargetPosition(FLnewTarget);
        motorBackRight.setTargetPosition(BRnewTarget);
        motorBackLeft.setTargetPosition(BLnewTarget);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (motorFrontRight.isBusy() || motorFrontLeft.isBusy() || motorBackRight.isBusy() || motorBackLeft.isBusy()) {
            telemetry.addData("Status", "Running to position");
            telemetry.update();
        }

        /*while (motorFrontLeft.isBusy()) {
            telemetry.addData("Status", "Running to position");
            telemetry.update();
        }

        while (motorBackRight.isBusy()) {
            telemetry.addData("Status", "Running to position");
            telemetry.update();
        }

        while (motorBackLeft.isBusy()) {
            telemetry.addData("Status", "Running to position");
            telemetry.update();
        }*/

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(1000);
    }

    private double limit(double power) {
        if (power > 1)
            return 1;
        else
            return power;
    }

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightBackDrive");

        //reset encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition());
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            /*moveUntilTime("right", 2000);
            sleep(3000);
            moveUntilTime("turn right", 2000);
            sleep(3000);
            moveUntilTime("turn left", 2000);*/

            moveUntilTicks("right", 2000);


            sleep(30000);



           /* motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int newTarget = motorFrontRight.getTargetPosition() + (int) MOTOR_TICK_COUNT;
            motorFrontRight.setTargetPosition(newTarget);
            motorFrontRight.setPower(.5);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (motorFrontRight.isBusy()) {
                telemetry.addData("Status", "Running to position");
                telemetry.update();
            }
            motorFrontRight.setPower(0);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(30000);
            telemetry.addData("Status", "Complete");
            telemetry.update();*/
        }
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
        double correction, angle, gain = .30;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}