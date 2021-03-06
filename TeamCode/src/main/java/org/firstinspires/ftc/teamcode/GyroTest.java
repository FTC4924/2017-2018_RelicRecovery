package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by 4924_Users on 1/26/2018.
 */

@Disabled
@Autonomous(name = "GyroTest")
public class GyroTest extends IterativeRobot {

    static Servo armY = null;
    static Servo armX = null;
    ColorSensor sensorColor;


    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    public void runOpMode() {

        telemetry.addData("Status", "init");
        telemetry.update();
        //these names are set in the configuration on the Robot Controller phone
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        leftCollectionMotor = hardwareMap.get(DcMotor.class, "leftCollectionMotor");
        relicExtension = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");
        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        armY = hardwareMap.get(Servo.class, "armY");
        armX = hardwareMap.get(Servo.class, "armX");



        //one set of motors has to be reversed because they are facing a different way
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftCollectionMotor.setDirection(DcMotor.Direction.FORWARD);
        relicExtension.setDirection(DcMotor.Direction.FORWARD);
        deliveryMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //on the red board, the robot will be facing backwards, so we want to reverse the motors
        if (startingPosition().isRed()) {

            reverseDriveBase();
        }

        //we want the motors' encoders to be set to 0 when robot is initialized
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DRIVE_BASE_MOTORS[0] = frontLeftMotor;
        DRIVE_BASE_MOTORS[1] = frontRightMotor;
        DRIVE_BASE_MOTORS[2] = backLeftMotor;
        DRIVE_BASE_MOTORS[3] = backRightMotor;

        ALL_MOTORS[0] = frontLeftMotor;
        ALL_MOTORS[1] = frontRightMotor;
        ALL_MOTORS[2] = backLeftMotor;
        ALL_MOTORS[3] = backRightMotor;

        boolean jewelDone = false;

        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU_Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMU_Parameters);

        {

            staticTelemetry = telemetry;
        }

        if (isAutonomous()) { //below is Autonomous-only code, this removes unnecessary loading

            vuforiaParameters.vuforiaLicenseKey = "AdDqKyD/////AAAAGQ/rpKTVlUiMmdwxDFRT5LiD8kI3QucN9xL8BbQRw/rYsleEjKBm/GOQB4GnSmvyzTFNFOBfZQ9o06uen5gYZvJthDx8OSVm78QegaFqHEGPjDRtqIAuLxz+12HVQXbIutqXfR595SNIl0yKUbbXFTq21ElXEDDNwO0Lv8ptnJPLib85+omkc5c8xfG6oNIhFg+sPIfCrpFACHdrr23MpY8AzLHiYleHnhpyY/y/IqsXw7CYPV2kKY70GEwH8I0MGxBw8tw8EoYpXk4vxUzHAPfgvBDztFz3x9fpcxoeqb0jl2L7GB7Aq7u+Sea+g4FoTG/9FD4rEy4I/Lz+OjdbE2eEUCGnyy10Q5o3AGG5R3cW";
            //license key set, necessary for vuforia code to be used
            vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //uses front camera of Robot Controller for detection
            //if above code is changed to ...CameraDirection.BACK;, the back-facing camera will be used instead

            //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            //if above code is enabled, the Driver Station will display the camera reading from the Robot Controller on its screen

            vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters); //vuforia object initialized based on set vuforiaParameters
            relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTrackables.activate(); //start listening for camera's data
        }
        {

            CRYPTOBOX_CENTER_DISTANCE = 15.5;
        }

        {
            if (STARTING_POSITION.isNear()) {

                if (STARTING_POSITION.isBlue()) { //cryptobox is reversed when colors change

                    CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
                    CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
                } else {

                    CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
                    CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
                }
            } else { //STARTING_POSITION.isFar()

                CRYPTOBOX_LEFT_DISTANCE = -1;
                CRYPTOBOX_RIGHT_DISTANCE = -1;
            }
        }

        while(!opModeIsActive()) {

            if (isAutonomous()) {

                vuMark = RelicRecoveryVuMark.from(relicTemplate); //LEFT, CENTER, or RIGHT if read, UNKNOWN if undetermined reading
                telemetry.addData("vuMark", vuMark.toString()); //LEFT, CENTER, or RIGHT, useful for debugging
                telemetry.addData("Status", "init_loop");
                telemetry.update();
            }
        }

        waitForStart();
        elapsedTime.reset();

        armY.setPosition(0.65);
        ElapsedTime opmodeRunTime = new ElapsedTime();
        armY.setPosition(0.15);

        boolean isFinished = false;
        boolean startGlyph = false;


        while(opModeIsActive()) {
            if (!isFinished) {

                turnToPosition(TURN_POWER, 90);
                opmodeRunTime.reset();
                while (opmodeRunTime.time() < 1) {
                }
                turnToPosition(TURN_POWER, 179);
                opmodeRunTime.reset();
                while (opmodeRunTime.time() < 1) {
                }
                turnToPosition(TURN_POWER, -90);
                opmodeRunTime.reset();
                while (opmodeRunTime.time() < 1) {
                }
                turnToPosition(TURN_POWER, 0);
                isFinished = true;
            }
        }

        setMotorsModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER, ALL_MOTORS);
    }

}
