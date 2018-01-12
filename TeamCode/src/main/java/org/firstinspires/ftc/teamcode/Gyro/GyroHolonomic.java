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

package org.firstinspires.ftc.teamcode.Gyro;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@Disabled
@TeleOp(name = "GyroHolonomic", group = "Iterative Opmode")

public class GyroHolonomic extends GyroRobot {

    @Override
    public void Init() {

        super.Init();
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void Start() {

        super.Start();
        elapsedTime.reset();
    }

    @Override
    public void Loop() {

        super.Loop();
        //We initialize and give certain values to certain variables, so that we can use them later
        double collectionPower = 0.0;
        double deliveryPower = 0.0;
        final double BAR_MOVE = -1.0;
        final double BAR_DOWN = 1.0;

        double drive = -gamepad1.left_stick_y;
        //drive is what direction we want to    move, either forwards, backwards, or neither
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
        boolean deliveryUp = gamepad2.dpad_up;
        //deliveryUp is dependent on whether or not we want the delivery to deliver
        boolean deliveryDown = gamepad2.dpad_down;
        //deliveryDown is dependent on whether or not we want the delivery to go downwards
        double linearSlide =  (gamepad2.left_stick_y);
         //this extends the linearSlide using the measuring tape to extend it
        boolean slowSpeed = gamepad1.left_bumper;
        //slowSpeed says whether or not the driver wants to go at a slower speed, which can cause
        // finer adjustments
        double elbow = gamepad1.right_stick_y;
        //elbow is a variable set to the amount of power the driver wants to give to move this servo
        double clawPower = gamepad2.right_stick_y;
        //clawPower is a variable set to the amount of power the driver wants to give to move
        // this servo

        if (collectionPowerUp) collectionPower = 1;
        else if (collectionPowerDown) collectionPower = -1;

        if (gamepad2.left_bumper) barServo.setPosition(BAR_DOWN + BAR_MOVE);
        else barServo.setPosition(BAR_DOWN);

        if (deliveryUp) deliveryPower = -1;
        else if (deliveryDown) deliveryPower = 1;

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
            //to calculate the slower speed, we multiply the prevously calculated powers by 0.2
            frontLeftPower = 0.2 * (frontLeftPower);
            frontRightPower = 0.2 * (frontRightPower);
            backRightPower = 0.2 * (backRightPower);
            backLeftPower = 0.2 * (backLeftPower);
        }

        // Send calculated power to wheels and motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        collectionMotor.setPower(collectionPower);
        linearSlideMotor.setPower(linearSlide);
        deliveryMotor.setPower(deliveryPower);
        elbowServo.setPower(elbow);
        clawServo.setPower(clawPower);
        telemetry.addData("Time to end: ", 160 - elapsedTime.seconds());
    }

    @Override
    public void Stop() {

        super.Stop();
        //We make sure to turn everything off when the driver hits STOP
        //In fact, the "STOP" function is assigned to a button with an image
        //the "STOP" button's image is an red opaque square, in a white circle
        //hitting this "stops" the program
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        collectionMotor.setPower(0);
        linearSlideMotor.setPower(0);
        deliveryMotor.setPower(0);
        elbowServo.setPower(0);
        clawServo.setPower(0);
    }

}