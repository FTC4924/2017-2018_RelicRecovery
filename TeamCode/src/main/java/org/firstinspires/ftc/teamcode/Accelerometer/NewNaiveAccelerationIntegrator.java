package org.firstinspires.ftc.teamcode.Accelerometer;

import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

/**
 * Created by user6 on 12/9/2017.
 */

public class NewNaiveAccelerationIntegrator implements BNO055IMU.AccelerationIntegrator {

        //------------------------------------------------------------------------------------------
        // State
        //------------------------------------------------------------------------------------------

        BNO055IMU.Parameters parameters;
        Position position;
        Velocity velocity;
        Acceleration acceleration;

        public Position getPosition() { return this.position; }
        public Velocity getVelocity() { return this.velocity; }
        public Acceleration getAcceleration() { return this.acceleration; }

        //------------------------------------------------------------------------------------------
        // Construction
        //------------------------------------------------------------------------------------------

        public NewNaiveAccelerationIntegrator()
        {
            this.parameters = null;
            this.position = new Position();
            this.velocity = new Velocity();
            this.acceleration = null;
        }

        //------------------------------------------------------------------------------------------
        // Operations
        //------------------------------------------------------------------------------------------

        @Override public void initialize(@NonNull BNO055IMU.Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity)
        {
            this.parameters = parameters;
            this.position = initialPosition != null ? initialPosition : this.position;
            this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
            this.acceleration = null;
        }

        @Override public void update(Acceleration linearAcceleration)
        {
            // We should always be given a timestamp here
            if (linearAcceleration.acquisitionTime != 0)
            {
                // We can only integrate if we have a previous acceleration to baseline from
                if (acceleration != null)
                {
                    Acceleration accelPrev    = acceleration;
                    Velocity     velocityPrev = velocity;

                    acceleration = linearAcceleration;

                    if (accelPrev.acquisitionTime != 0)
                    {

                        if(Math.abs(acceleration.xAccel) > 0.2) { //filtering out noise data - if the acceleration is less than 0.2, the delta is given to be 0... TODO: confirm if this does as intended
                            Velocity deltaVelocity = meanIntegrate(acceleration, accelPrev);
                            velocity = plus(velocity, deltaVelocity);
                        }
                    }

                    if (velocityPrev.acquisitionTime != 0)
                    {
                        Position deltaPosition = meanIntegrate(velocity, velocityPrev);
                        position = plus(position, deltaPosition);
                    }

                    if (parameters != null && parameters.loggingEnabled)
                    {
                        RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", (acceleration.acquisitionTime - accelPrev.acquisitionTime)*1e-9, acceleration, velocity, position);
                    }
                }
                else
                    acceleration = linearAcceleration;
            }
        }
    }
