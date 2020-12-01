package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedTHIS", group = "Linear OpMode")
public class AutoRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private CRServo intakeTurner = null;
    private Servo wobbleLatcher = null;
    private CRServo wobbleTurner = null;
    private DcMotor conveyerBelt = null;
    private DcMotor shooter = null;

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
            motorFrontLeft.setPower((directions[d][0]));
            motorFrontRight.setPower((directions[d][1]));
            motorBackLeft.setPower((directions[d][2]));
            motorBackRight.setPower((directions[d][3]));
        }
    }

    public int inchesToTime(double in){
        double tDouble = (in)*17.54385964912;
        int t = (int) tDouble;
        return t;
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
        conveyerBelt = hardwareMap.get(DcMotor.class, "conveyerBelt");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        waitForStart();

        while (opModeIsActive()) {
            shooter.setPower(-1);
            moveUntilTime("forward", inchesToTime(58));
            conveyerBelt.setPower(-1);
            sleep(200);
            conveyerBelt.setPower(0);
            sleep(200);
            conveyerBelt.setPower(-1);
            sleep(200);
            conveyerBelt.setPower(0);
            sleep(200);
            moveUntilTime("backward", inchesToTime(58));
            shooter.setPower(0);
            sleep(30000);
        }
    }
}