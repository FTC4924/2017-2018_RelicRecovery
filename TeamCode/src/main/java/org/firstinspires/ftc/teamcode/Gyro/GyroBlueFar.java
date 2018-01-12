package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotPosition;

import static com.sun.tools.javac.util.Constants.format;

/**
 * Created by 4924_Users on 1/5/2018.
 */
@Autonomous(name = "GyroBlueFar")
public class GyroBlueFar extends GyroAutonomous {


    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    @Override
    public void Loop() {

        if (elapsedTime.seconds() > 3 && !jewelDone) {
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.update();


            if (colorSensor.red() > colorSensor.blue()) {
                if (startingPosition().isRed()) {

                    jewelArmX.setPosition(0.3);
                    telemetry.addLine("Color Red; Kicking Blue; On my right");
                } else {

                    jewelArmX.setPosition(0.8);
                    telemetry.addLine("Color Red; Kicking Red; On my Left");
                }

                jewelDone = true;
                telemetry.update();
            } else if (colorSensor.blue() > colorSensor.red()) {

                if (startingPosition().isBlue()) {

                    jewelArmX.setPosition(0.3);
                    telemetry.addLine("Color Blue; Kicking Red; On my right ");
                } else {

                    jewelArmX.setPosition(0.8);
                    telemetry.addLine("Color Blue; Kicking Blue; On my left");
                }

                jewelDone = true;
                telemetry.update();
            } else {

                telemetry.addLine("Too Close To Tell");
                jewelDone = true;
                telemetry.update();
            }
        }

        if (elapsedTime.seconds() > 5 && jewelDone) {

            jewelArmX.setPosition(0.4);
            jewelArmY.setPosition(0.65);
            startGlyph = true;
        }

        if ((!isFinished) && startGlyph) {

            driveWithEncoders(DRIVE_POWER, 10);
            driveWithEncoders(DRIVE_POWER, 13);
            turnToPosition(TURN_POWER, -90);
            driveWithEncoders(DRIVE_POWER, calculateInches());
            if (startingPosition().isRed()) reverseDriveBase();
            turnToPosition(TURN_POWER, 90);
            elapsedTime.reset();
            while ((elapsedTime.time() < 4) && opModeIsActive()) collectionMotor.setPower(-0.5);
            collectionMotor.setPower(0);
            elapsedTime.reset();
            while ((elapsedTime.time() < 1) && opModeIsActive()) setMotorsPowers(0.2, DRIVE_BASE_MOTORS);
            driveWithEncoders(DRIVE_POWER, -5);
            collectionMotor.setPower(0);
            setMotorsPowers(0, DRIVE_BASE_MOTORS);
            isFinished = true;
        }
    }
}
