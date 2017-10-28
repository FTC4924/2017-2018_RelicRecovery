package ExperimentalClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Disabled
@Autonomous(name = "fourwheelencoder")
public class fourWheeledEncoderTest extends OpMode {

    final double COUNTS_PER_INCH = 1120 * (32 / 48D) * 1 / 4D * 1 / Math.PI;
    int FINAL_POSITION = 1120;
    DcMotor frontLeftMotor;

    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;


    /*DcMotor[] motors = {

            frontLeftMotor

            frontRightMotor,
            backLeftMotor,
            backRightMotor,

    };*/

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(FINAL_POSITION + frontLeftMotor.getCurrentPosition());
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setTargetPosition(FINAL_POSITION + frontLeftMotor.getCurrentPosition());
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setTargetPosition(FINAL_POSITION + frontLeftMotor.getCurrentPosition());
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setTargetPosition(FINAL_POSITION + frontLeftMotor.getCurrentPosition());
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        /*for (DcMotor motor : motors) {

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(FINAL_POSITION + motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/
    }

    @Override
    public void init_loop() {


    }

    @Override
    public void start() {
        frontLeftMotor.setPower(0.5);

        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

    }

    public void loop() {
//        if(frontLeftMotor.getCurrentPosition() >= 1000) frontLeftMotor.setPower(0);
        telemetry.addData("Path1", "Drove to%7d", frontLeftMotor.getCurrentPosition());
        telemetry.update();
    }

    public double calculateInches(RelicRecoveryVuMark vuMark) {

        telemetry.addData("vumark status", vuMark.toString());
        if (vuMark == RelicRecoveryVuMark.LEFT) return 35.5 - 6.5;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return 35.5 + 6.5;
        else return 35.5; //CENTER or UNKNOWN
    }

    /**
     * Created by 4924_Users on 10/11/2017.
     */
    @Autonomous(name = "Color sensor test")
    public static class ColorSensorTest extends LinearOpMode {
        public ColorSensor rightBeaconSensor;
        public ColorSensor leftBeaconSensor;

        @Override
        public void runOpMode() {
            int leftBlueReading = 0;
            int rightRedReading = 0;
            int rightBlueReading = 0;
            int leftRedReading = 0;
            if (leftBeaconSensor.red() > 0) {
                leftRedReading = leftBeaconSensor.red();
            }
            if (leftBeaconSensor.blue() > 0) {
                leftBlueReading = leftBeaconSensor.blue();
            }
            if (rightBeaconSensor.red() > 0) {
                rightRedReading = rightBeaconSensor.red();
            }
            if (rightBeaconSensor.blue() > 0) {
                rightBlueReading = rightBeaconSensor.blue();
            }
            telemetry.addData("Left color sensor red reading", leftRedReading);
            telemetry.addData("Left color sensor blue reading", leftBlueReading);
            telemetry.addData("Right color sensor red reading", rightRedReading);
            telemetry.addData("Right color sensor blue reading", rightBlueReading);

        }
    }
}
