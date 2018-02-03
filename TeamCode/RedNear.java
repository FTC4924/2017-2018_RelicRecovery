package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by 4924_Users on 11/4/2017.
 */

@Disabled
@Autonomous(name = "RedNear")
public class RedNear extends BlueNear {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }
}
