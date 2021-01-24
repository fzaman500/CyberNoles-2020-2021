package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoRedComp2", group = "Linear OpMode")
public class  AutoRedLeft extends LinearOpMode {

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
            motorFrontLeft.setPower((directions[d][0]));
            motorFrontRight.setPower((directions[d][1]));
            motorBackLeft.setPower((directions[d][2]));
            motorBackRight.setPower((directions[d][3]));
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

        //wobble
        wobbleFlipper = hardwareMap.get(DcMotor.class, "wobbleFlipper");
        wobbleIntake = hardwareMap.get(Servo.class, "wobbleIntake");

        //intake and conveyer
        intakeFirst = hardwareMap.get(DcMotor.class, "intakeFirst");
        twoWheelIntake = hardwareMap.get(CRServo.class, "twoWheelIntake");
        intakeCricket = hardwareMap.get(CRServo.class, "intakeCricket"); //?
        conveyerBelt = hardwareMap.get(DcMotor.class, "conveyerBelt");
        conveyerServo = hardwareMap.get(CRServo.class, "conveyerServo");

        waitForStart();

        while (opModeIsActive()) {
            wobbleIntake.setPosition(1); //hold
            sleep(2000);
            rampPusher.setPower(1); //push
            sleep(4000);
            rampPusher.setPower(0);
            sleep(2000);
            shooter.setPower(-1); //on shooter
            moveUntilTime("forward", 2500);
            //moveUntilTime("left", 500);
            sleep(2000);
            wobbleFlipper.setPower(-1); //move wobble
            sleep(1000);
            wobbleFlipper.setPower(0);
            sleep(1000);
            wobbleIntake.setPosition(0); //let go
            sleep(1000);
            //wobbleFlipper.setPower(.75); //return wobble
            //sleep(1500);
            //wobbleFlipper.setPower(0);
            moveUntilTime("backward", 600);
            conveyerBelt.setPower(1);
            sleep(1000);
            conveyerBelt.setPower(0);
            sleep(750);
            conveyerBelt.setPower(1);
            sleep(750);
            conveyerBelt.setPower(0);
            sleep(750);
            conveyerBelt.setPower(1);
            sleep(750);
            conveyerBelt.setPower(0);
            //moveUntilTime("left", 500);
            shooter.setPower(0);
            moveUntilTime("forward", 500);
            break;
        }
    }
}