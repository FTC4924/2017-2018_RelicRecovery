package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Accelerometer.NewNaiveAccelerationIntegrator;
import org.firstinspires.ftc.teamcode.RobotPosition;
import org.firstinspires.ftc.teamcode.RotationalDirection;

/**
 * Created by 4924_Users on 12/30/2017.
 */

public class GyroRobot extends LinearOpMode {

    protected static final double DRIVE_POWER = 0.4;
    protected static final double TURN_POWER = 0.4;
    //to center of cryptobox
    protected static final double CRYPTOBOX_OFFSET = 6.5; //offset of left/right areas of cryptobox
    private static final int ENCODER_TICKS_PER_ROTATION = 1120; //encoder counts per shaft turn
    private static final int MOTOR_TEETH = 32;
    private static final int WHEEL_TEETH = 16;
    private static final double GEAR_RATIO = WHEEL_TEETH / (double) MOTOR_TEETH; //48 teeth on motor gear, 32 teeth on wheel gear
    private static final double WHEEL_DIAMETER = 4;
    //declares our hardware, initialized later in init()
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //wheel diameter * pi
    private static final double COUNTS_PER_INCH =
            (ENCODER_TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    private static final double GYRO_TURN_TOLERANCE_DEGREES = 5;
    private static final int ENCODER_TOLERANCE = 10;
    protected static Telemetry staticTelemetry;
    protected static double CRYPTOBOX_CENTER_DISTANCE = Double.MIN_VALUE; //distance from center of relic-side
    protected static RobotPosition STARTING_POSITION;
    protected static DcMotor[] DRIVE_BASE_MOTORS = new DcMotor[4];
    protected static DcMotor[] ALL_MOTORS = new DcMotor[8];
    protected static ElapsedTime elapsedTime = new ElapsedTime();

    static BNO055IMU imu;

    static BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
    Orientation angles;

    static DcMotor frontLeftMotor;
    static DcMotor frontRightMotor;
    static DcMotor backLeftMotor;
    static DcMotor backRightMotor;

    static RelicRecoveryVuMark vuMark; //enum set based on pictogram reading
    static double CRYPTOBOX_LEFT_DISTANCE;
    static double CRYPTOBOX_RIGHT_DISTANCE;
    protected int cameraMonitorViewId;

    static VuforiaLocalizer vuforia; //later initialized with (sic) vuforiaParameters
    static VuforiaTrackables relicTrackables;
    static VuforiaTrackable relicTemplate;
    static DcMotor collectionMotor;
    VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
    static DcMotor linearSlideMotor;
    static DcMotor deliveryMotor;
    static Servo barServo;
    static CRServo elbowServo;
    static CRServo clawServo;
    static Servo jewelArmX;
    static Servo jewelArmY;
    static Servo alignmentDevice;
    static ColorSensor colorSensor;


    {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 15000;
    }

    static void reverse(DcMotor d) {

        d.setDirection(d.getDirection().inverted());
    }

    protected static void reverseDriveBase() {

        reverse(frontLeftMotor);
        reverse(frontRightMotor);
        reverse(backLeftMotor);
        reverse(backRightMotor);
    }

    protected void driveWithEncoders(double movePower, double moveDistanceInInches) {

        setMotorsTargets(moveDistanceInInches, DRIVE_BASE_MOTORS);

        setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, DRIVE_BASE_MOTORS);

        while (Math.abs(DRIVE_BASE_MOTORS[1].getCurrentPosition() - DRIVE_BASE_MOTORS[1].getTargetPosition()) > ENCODER_TOLERANCE && opModeIsActive())

            setMotorsPowers(movePower, DRIVE_BASE_MOTORS);

        setMotorsPowers(0, DRIVE_BASE_MOTORS);

        setMotorsModes(DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_BASE_MOTORS);
    }

    protected static void setMotorsTargets(double encoderTargetInInches, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setTargetPosition(
                    (int) (COUNTS_PER_INCH * encoderTargetInInches) + d.getCurrentPosition());
    }


    protected static void setMotorsPowers(double power, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setPower(power);
    }

    protected static void setMotorsModes(DcMotor.RunMode runMode, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setMode(runMode);
    }

    protected static void turn(double power, RotationalDirection direction) {

        if (direction == RotationalDirection.CLOCKWISE) {

            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
        } else if (direction == RotationalDirection.COUNTER_CLOCKWISE) {

            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
        } else
            throw new IllegalArgumentException("RotationalDirection may be clockwise or counter-clockwise only");
    }

    protected void turnToPosition(double turnPower, double desiredHeading) {

        //desiredHeading is the angle that we want to move to, it should be -180<x<180
        setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DRIVE_BASE_MOTORS);
        //we reduce desiredHeading just in case it is formatted incorrectly
        while (getHeading() - desiredHeading > GYRO_TURN_TOLERANCE_DEGREES) {

            telemetry.addData("Heading", getHeading());
            telemetry.update();
            turn(turnPower, RotationalDirection.COUNTER_CLOCKWISE);
        }
        //if it is faster to turn CCW, then robot turns CCW

        while (getHeading() - desiredHeading < -GYRO_TURN_TOLERANCE_DEGREES) {

            telemetry.addData("Heading", getHeading());
            telemetry.update();
            turn(turnPower, RotationalDirection.CLOCKWISE);
        }
        setMotorsPowers(0, DRIVE_BASE_MOTORS);
        //defaults to CW turning
    }

    protected void turnWithEncoders(double turnPower, double desiredHeading) {

        setMotorsTargets(desiredHeading, new DcMotor[]{frontLeftMotor, backLeftMotor});
        setMotorsTargets(-desiredHeading, new DcMotor[]{frontRightMotor, backRightMotor});


        setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, DRIVE_BASE_MOTORS);
        //setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new DcMotor[] {frontRightMotor, backLeftMotor, backRightMotor});
        while (Math.abs(DRIVE_BASE_MOTORS[1].getCurrentPosition() - DRIVE_BASE_MOTORS[1].getTargetPosition()) > ENCODER_TOLERANCE && opModeIsActive()) {

            writeTelemetry(getHeading());
            frontLeftMotor.setPower(-turnPower);
            frontRightMotor.setPower(turnPower);
            backLeftMotor.setPower(-turnPower);
            backRightMotor.setPower(turnPower);
            writeTelemetry(DRIVE_BASE_MOTORS[1].getTargetPosition());
        }/*
        if ((frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) > 5) {
            turn(turnPower, RotationalDirection.COUNTER_CLOCKWISE);
        }*/
        setMotorsPowers(0, DRIVE_BASE_MOTORS);

        setMotorsModes(DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_BASE_MOTORS);
    }

    protected double getHeading() {
        updateGyro();
        return angles.firstAngle;
    }

    protected static void writeTelemetry(Object status) {

        staticTelemetry.addData(status.toString(), "");
        staticTelemetry.update();
    }

    void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    double calculateInches() {

        if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
        else return CRYPTOBOX_CENTER_DISTANCE;
    }

    public final void runOpMode() {

        try {
            Init();
            while (!opModeIsActive()) Init_Loop();
            waitForStart();
            Start();
            while (opModeIsActive()) Loop();
            Stop();
        } catch (Exception e) {

            telemetry.addData("err", e.getMessage());
            telemetry.addData("errStackTrace", e.getStackTrace());
        }
    }

    protected void Init() {

        telemetry.addData("Status", "init");
        telemetry.update();
        //these names are set in the configuration on the Robot Controller phone
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU_Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMU_Parameters.accelerationIntegrationAlgorithm = new NewNaiveAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMU_Parameters);
        telemetry.update();
        updateGyro();
        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        linearSlideMotor = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");
        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");
        barServo = hardwareMap.get(Servo.class, "barServo");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        jewelArmY = hardwareMap.get(Servo.class, "armY");
        jewelArmX = hardwareMap.get(Servo.class, "armX");
        alignmentDevice = hardwareMap.get(Servo.class, "alignmentDevice");

        //one set of motors has to be reversed because they are facing a different way
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        collectionMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        deliveryMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        DRIVE_BASE_MOTORS[0] = frontLeftMotor;
        DRIVE_BASE_MOTORS[1] = frontRightMotor;
        DRIVE_BASE_MOTORS[2] = backLeftMotor;
        DRIVE_BASE_MOTORS[3] = backRightMotor;

        ALL_MOTORS[0] = frontLeftMotor;
        ALL_MOTORS[1] = frontRightMotor;
        ALL_MOTORS[2] = backLeftMotor;
        ALL_MOTORS[3] = backRightMotor;
        ALL_MOTORS[4] = collectionMotor;
        ALL_MOTORS[5] = linearSlideMotor;
        ALL_MOTORS[6] = deliveryMotor;

        {

            staticTelemetry = telemetry;
        }

        barServo.setPosition(1);
        //Set the continous servos to a neutral power, to make sure they do not move while
        // initiallizing
        clawServo.setPower(0);
        elbowServo.setPower(0);
    }

    protected void Init_Loop() {}
    protected void Start() {

        elapsedTime.reset();
    }
    protected void Loop() {}
    protected void Stop() {

        setMotorsModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER, ALL_MOTORS);
    }
    }
