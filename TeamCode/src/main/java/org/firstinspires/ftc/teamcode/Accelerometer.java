package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.*;

/**
 * Created by user6 on 12/9/2017.
 */

@Autonomous(name="position sensing test")
public class Accelerometer extends AccelerometerRobot {


    public static Location startingPosition;

    public static void driveToPoint(Location targetPosition) {

        startingPosition = currentPosition();
        double targetAngle = Location.getTargetAngle(currentPosition(), targetPosition);
        turnToPosition(TURN_POWER, targetAngle);
//        driveWithEncoders(DRIVE_POWER, Location.distance(currentPosition(), targetPosition));
        setMotorsPowers(DRIVE_POWER, DRIVE_BASE_MOTORS);
        double lastLeftPower = 0;
        double lastRightPower = 0;
        while(Location.distance(currentPosition(), targetPosition) > 0.5) {

            setMotorsPowers(lastLeftPower + Location.proportionalError(startingPosition, targetAngle)*PROPORTIONAL_COEFFICIENT, LEFT_MOTORS);
            setMotorsPowers(lastRightPower - Location.proportionalError(startingPosition, targetAngle)*PROPORTIONAL_COEFFICIENT, RIGHT_MOTORS);
        }
    }

    public static final DcMotor[] LEFT_MOTORS = {frontLeftMotor, backLeftMotor};
    public static final DcMotor[] RIGHT_MOTORS = {frontRightMotor, backRightMotor};

    public static final double PROPORTIONAL_COEFFICIENT = 1;

    public static Location currentPosition() {

        return new Location(imu.getPosition().x, imu.getPosition().y);
    }

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    private static class Location {

        double x;
        double y;

        Location(double x, double y) {

            this.x = x;
            this.y = y;
        }

        static double deltaX(Location l1, Location l2) {

            return l2.x - l1.x;
        }

        static double deltaY(Location l1, Location l2) {

            return l2.x - l1.x;
        }

        static double distance(Location l1, Location l2) {

            return sqrt(pow(deltaX(l1, l2), 2) + pow(deltaY(l1, l2), 2));
        }

        static double getTargetAngle(Location l1, Location l2) {

            double hypotenuse = distance(l1, l2);
            return formatAngle(asin(hypotenuse/deltaY(l1, l2)) + getHeading());
        }

        static double proportionalError(Location startingPosition, double targetAngle) {

            return distance(startingPosition, currentPosition())*sin(targetAngle);
        }
    }

    boolean isFinished = false;

    public void runOpMode() {

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

        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        relicExtension = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");
        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");


        //one set of motors has to be reversed because they are facing a different way
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        collectionMotor.setDirection(DcMotor.Direction.FORWARD);
        relicExtension.setDirection(DcMotor.Direction.FORWARD);
        deliveryMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while(opModeIsActive()) {
            telemetry.addData("X axis position:", imu.getPosition().x);
            if(!isFinished) {

                driveToPoint(new Location(12, 0));
                isFinished = true;
            }

        }
    }
}
