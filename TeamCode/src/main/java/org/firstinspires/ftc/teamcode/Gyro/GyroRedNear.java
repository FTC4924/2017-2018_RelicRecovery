package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotPosition;

/**
 * Created by 4924_Users on 1/5/2018.
 */
@Disabled
@Autonomous(name = "GyroRedNear")
public class GyroRedNear extends GyroBlueNear {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }

}
