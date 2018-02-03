package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 1/27/2018.
 */
@Autonomous(name="RedNear")
public class IterativeRedNear extends IterativeBlueNear {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }
}
