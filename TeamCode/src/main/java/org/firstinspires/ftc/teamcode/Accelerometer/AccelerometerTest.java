package org.firstinspires.ftc.teamcode.Accelerometer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;

/**
 * Created by 4924_Users on 12/16/2017.
 */

@Disabled
@Autonomous(name="AccelerometerTest")
public class AccelerometerTest extends Accelerometer {

    ArrayList<Double> xAccelList = new ArrayList<>();
    ArrayList<Double> yAccelList = new ArrayList<>();
    ArrayList<Double> zAccelList = new ArrayList<>();

    public double average(ArrayList<Double> a) {

        double result = 0;
        for(double d: a) {

            result += d;
        }

        return result/a.size();
    }

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
        imu.startAccelerationIntegration(new Position(DistanceUnit.CM, 0, 0, 0, System.nanoTime()), new Velocity(DistanceUnit.CM, 0, 0, 0, System.nanoTime()), 20);

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

            telemetry.addData("X Accel:", imu.getLinearAcceleration().xAccel);
            xAccelList.add(imu.getLinearAcceleration().xAccel);
            telemetry.addData("X Vel:", imu.getVelocity().xVeloc);
            telemetry.addData("X Position:", imu.getPosition().x);
            telemetry.addData("Y Accel:", imu.getLinearAcceleration().yAccel);
            yAccelList.add(imu.getLinearAcceleration().yAccel);
            telemetry.addData("Y Vel:", imu.getVelocity().yVeloc);
            telemetry.addData("Y Position:", imu.getPosition().y);
            telemetry.addData("Z Accel:", imu.getLinearAcceleration().zAccel);
            zAccelList.add(imu.getLinearAcceleration().zAccel);
            telemetry.addData("Z Vel:", imu.getVelocity().zVeloc);
            telemetry.addData("Z Position:", imu.getPosition().z);
            telemetry.addData("Test", "Testy test");
            telemetry.update();
        }

        telemetry.addData("X avg accel", average(xAccelList));
        telemetry.addData("Y avg accel", average(yAccelList));
        telemetry.addData("Z avg accel", average(zAccelList));
        telemetry.update();
    }
}
