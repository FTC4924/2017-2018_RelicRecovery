package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 12/2/2017.
 */
@Autonomous(name = "Encoder Telemetry")
public class EncoderTelemetry extends Robot{
    private boolean isFinished = false;
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    public void loop() {

        telemetry.addData( "frontLedtMotor:", frontLeftMotor.getCurrentPosition());
        telemetry.addData("frontRightMotor:", frontRightMotor.getCurrentPosition());
        telemetry.addData("backLeftMotor:", backLeftMotor.getCurrentPosition());
        telemetry.addData("backRightMotor:", backRightMotor.getCurrentPosition());
        telemetry.update();
        if (!isFinished) {
            driveWithEncoders(DRIVE_POWER, 10.0f);
            isFinished = true;
        }

    }
}
