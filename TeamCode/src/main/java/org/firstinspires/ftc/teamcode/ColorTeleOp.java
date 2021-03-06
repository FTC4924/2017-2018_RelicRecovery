/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "RRHolonomic", group = "Iterative Opmode")

public class ColorTeleOp extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor collectionLeft = null;
    private DcMotor collectionRight = null;
    private DcMotor relicExtension = null;
    private DcMotor deliveryMotor = null;
    private Servo barServo = null;
    private CRServo elbowServo = null;
    private CRServo clawServo = null;
    private Servo armY = null;
    private Servo armX = null;
    private Servo alignmentDevice = null;

    /*
     Code to run ONCE when the driver hits INIT
     */

    boolean leftBumperPressedBefore = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        collectionLeft = hardwareMap.get(DcMotor.class, "collectionLeft");
        collectionRight = hardwareMap.get(DcMotor.class, "collectionRight");
        relicExtension = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");
        barServo = hardwareMap.get(Servo.class, "barServo");
        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");
        clawServo = hardwareMap.get(CRServo.class, "clawServo");
        armY = hardwareMap.get(Servo.class, "armY");
        armX = hardwareMap.get(Servo.class, "armX");
        alignmentDevice= hardwareMap.get(Servo.class, "alignmentDevice");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        collectionLeft.setDirection(DcMotor.Direction.REVERSE);
        collectionRight.setDirection(DcMotor.Direction.FORWARD);
        relicExtension.setDirection(DcMotor.Direction.FORWARD);
        deliveryMotor.setDirection(DcMotor.Direction.FORWARD);
        //Set the 180 servos to their middle position
        //armX.setPosition(0.4);
        //armY.setPosition(0.65);
        //Set the continous servos to a neutral power, to make sure they do not move while
        // initiallizing
        clawServo.setPower(0);
        elbowServo.setPower(0);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    double x;
    double y;
    static double alignment = 0.8;
    boolean leftTriggerPressed = false;
    //barServo;



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //We initialize and give certain values to certain variables, so that we can use them later
        double collectionPower = 0.0;
        double deliveryPower = 0.0;
        final double BARMOVE = -0.6;
        final double BARDOWN = 0.8;
        double servoStep = 0.02;

        //we set what to do when the motor is not given power, which is to brake completely,
        //instead of coasting
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double drive = -gamepad1.left_stick_y;
        //drive is what direction we want to move, either forwards, backwards, or neither
        double holonomic = -gamepad1.left_stick_x;
        //holonomic is what direction we want to move sideways
        double turnRight = gamepad1.right_trigger;
        //turnRight is how much we want to turn right
        double turnLeft = gamepad1.left_trigger;
        //turnLeft is how much we want to turn left
        boolean collectionPowerUp = gamepad2.b;
        //collectionPowerUp is dependent on whether or not we want the collection to collect
        boolean collectionPowerDown = gamepad2.a;
        //collectionPowerDown is dependent on whether or not we want the collection deliver
        //(Push downwards)
        /*boolean deliveryUp = gamepad2.dpad_up;
        //deliveryUp is dependent on whether or not we want the delivery to deliver
        boolean deliveryDown = gamepad2.dpad_down; */
        //deliveryDown is dependent on whether or not we want the delivery to go downwards
        double linearSlide = gamepad2.left_stick_y;
         //this extends the linearSlide using the measuring tape to extend it
        boolean slowSpeed = gamepad1.left_bumper;
        //slowSpeed says whether or not the driver wants to go at a slower speed, which can cause
        // finer adjustments
        double elbow = gamepad1.right_stick_y * 0.83;
        //elbow is a variable set to the amount of power the driver wants to give to move this servo
        double clawPower = -gamepad2.right_stick_y;
        //clawPower is a variable set to the amount of power the driver wants to give to move
        // this servo


        if (collectionPowerUp) {
            //if we want it to collect, we set collectionPower to 1
            collectionPower = 1;
        } else if (collectionPowerDown) {
            //if we want the collection to deliver/spin backwards, we set collectionPower to -1
            collectionPower = -1;
        }

        if (gamepad2.left_bumper) {
            //if the driver hits the left bumper, that signals that the driver wants to make
            // the kicker go up, and kick the block up
            barServo.setPosition(0.2);
            leftBumperPressedBefore = true;

        } else if (leftBumperPressedBefore) {
            //If the driver does not press this button, we let the kicker fall down
            barServo.setPosition(0.8);
            leftBumperPressedBefore = false;

        }

        if (gamepad2.left_trigger > 0) {
            leftTriggerPressed = true;
        } else {
            leftTriggerPressed = false;
        }

        if (gamepad2.dpad_up && !leftTriggerPressed) {
            deliveryPower = -1;
        }
        if(!leftTriggerPressed && gamepad2.dpad_down) {
            deliveryPower = 1;
        }

        if (gamepad2.dpad_up && leftTriggerPressed) {
            if(y + servoStep < 1) y += servoStep;
        }
        if(leftTriggerPressed && gamepad2.dpad_down) {
            if(y - servoStep > 0) y -= servoStep;
        }



        if (gamepad2.dpad_left && leftTriggerPressed) {
            if(x + servoStep < 1) x += servoStep;
        }
        if (gamepad2.dpad_right && leftTriggerPressed){
            if(x - servoStep > 0) x -= servoStep;
        }

        //This is temporary debugging code
        if (gamepad1.a) {
            elbow = -0.83;
        } else if (gamepad1.y) {
            elbow = 0.83;
        }

        if (gamepad1.dpad_left) {
            alignment = 0.2;
        } else if(gamepad1.dpad_up){
            if(alignment - servoStep > 0.2) alignment -= servoStep;
        } else if(gamepad1.dpad_down) {
            if(alignment + servoStep < 0.8) alignment += servoStep;
        }



        //we calculate the power to send to each different wheel, which each need their own power
        //since it is calculated in different ways, because of the turning and holonomic abilities

        double frontLeftPower =
                Range.clip(drive - holonomic + turnRight - turnLeft, -1.0, 1.0);
        double frontRightPower =
                Range.clip(drive + holonomic - turnRight + turnLeft, -1.0, 1.0);
        double backRightPower =
                Range.clip(drive - holonomic - turnRight + turnLeft, -1.0, 1.0);
        double backLeftPower =
                Range.clip(drive + holonomic + turnRight - turnLeft, -1.0, 1.0);
        //We also clip the values, so when we calculate the slowSpeed values,
        //it does not turn the value 1.5 (which would be clipped automatically by the motor) to 0.3,
        //which is not a fifth of what the power of the motor would be

        if (slowSpeed) {
            //to calculate the slower speed, we multiply the previously calculated powers by 0.2
            frontLeftPower = 0.2 * (frontLeftPower);
            frontRightPower = 0.2 * (frontRightPower);
            backRightPower = 0.2 * (backRightPower);
            backLeftPower = 0.2 * (backLeftPower);
        }

        //elbow = Range.clip(elbow,-1.0,1.0);

        // Send calculated power to wheels and motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        collectionLeft.setPower(collectionPower);
        collectionRight.setPower(collectionPower);
        relicExtension.setPower(linearSlide);
        deliveryMotor.setPower(deliveryPower);
        elbowServo.setPower(elbow);
        clawServo.setPower(clawPower);
        if (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down) alignmentDevice.setPosition(alignment);
        if (leftTriggerPressed) {
            armX.setPosition(x);
            armY.setPosition(y);
        }

        // Show the elapsed game time
        telemetry.addData("Elbow Power", elbow);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Position i-n Y-Axis", y);
        telemetry.addData("Position in X-Axis", x);
        telemetry.addData("Alignment Device Position", alignment);
       //we make the turn values 0, so that the robot will stop turning
        turnLeft = 0;
        turnRight = 0;

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //We make sure to turn everything off when the driver hits STOP
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        collectionLeft.setPower(0);
        collectionRight.setPower(0);
        relicExtension.setPower(0);
        deliveryMotor.setPower(0);
        elbowServo.setPower(0);
        clawServo.setPower(0);
    }

}