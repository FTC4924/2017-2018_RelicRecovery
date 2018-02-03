package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotPosition;

/**
 * Created by 4924_Users on 1/20/2018.
 */
@Disabled
@Autonomous(name = "gyroTest")
public class GyroTest extends GyroAutonomous {

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    @Override
    public void Loop() {

        super.Loop();


        if((!isFinished) && startGlyph) {

            driveWithEncoders(DRIVE_POWER, 10);
            turnWithEncoders(DRIVE_POWER, 18);
            turnToPosition(TURN_POWER, 90);
            isFinished = true;
        }
    }

    public void runOpMode() {
        Init();
        while (!opModeIsActive()) Init_Loop();
        Start();
        while (opModeIsActive()) Loop();
        Stop();
    }
}
