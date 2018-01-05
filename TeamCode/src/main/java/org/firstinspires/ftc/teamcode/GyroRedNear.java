package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/5/2018.
 */
@Autonomous(name = "GyroRedNear")
public class GyroRedNear extends GyroBlueNear {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }

}
