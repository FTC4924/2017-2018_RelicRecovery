package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotPosition;

/**
 * Created by 4924_Users on 1/5/2018.
 */
@Autonomous(name = "GyroRedFar")
public class GyroRedFar extends GyroBlueFar {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_FAR;
    }
}
