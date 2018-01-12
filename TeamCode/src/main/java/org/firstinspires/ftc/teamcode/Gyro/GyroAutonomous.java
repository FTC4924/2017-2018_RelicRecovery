package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.RobotPosition;

/**
 * Created by 4924_Users on 1/6/2018.
 */

public abstract class GyroAutonomous extends GyroRobot {


    boolean isFinished = false;
    boolean startGlyph = false;
    boolean jewelDone = false;

    protected abstract RobotPosition startingPosition();

    {

        STARTING_POSITION = startingPosition();
    }

    @Override
    public void Init() {

        super.Init();
        if (startingPosition().isRed()) {

            reverseDriveBase();
        }
    }

    @Override
    public void Init_Loop() {

        super.Init_Loop();
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        //parameters.vuforiaLicenseKey = "ATsODcD/////AAAAAVw2lR...d45oGpdljdOh5LuFB9nDNfckoxb8COxKSFX";
        parameters.vuforiaLicenseKey = "AfgNWRP/////AAAAGTvgaD9NHEcegp9M9gVfWchUPuYO0sndvqbZcgtG8KGB6PCuQ3QWbV9b4twhV5fD/kl/3Iblwrmzj4Vw8DnXF6MS8PbHDXTMrzndAccgzFu+7cej0vmkb657jYHPtLz4Y7U5zIdyPHkbPz+9gRo9gtaUGBa3p8mezZd2qLlJNc4hcv1tcP4hRXsIPEf++0q7tVIco0JGNnd76A7G0REDE9/IKqaO32xvwPnPNS7C+NqcHdZjfRQMjy2FRWzfang2iz9z/Gu20FzSv2n3fhIEWWqhjg7c2UjVeteWTVdDX3Hrxzb/eDJtXfl6V7fPcmXz1dZjQZrsF2pk87O4cRus9N/SYC12cwsVA+HabGLO0Z1R";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        vuMark = RelicRecoveryVuMark.from(relicTemplate); //LEFT, CENTER, or RIGHT if read, UNKNOWN if undetermined reading
        telemetry.addData("vuMark", vuMark.toString()); //LEFT, CENTER, or RIGHT, useful for debugging
        telemetry.addData("Status", "init_loop");
        telemetry.update();
    }

    @Override
    public void Start() {

        super.Start();
        relicTrackables.activate();
        jewelArmY.setPosition(0.65);
        ElapsedTime elapsedTime = new ElapsedTime();
        jewelArmY.setPosition(0.15);
    }
}
