// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.ShooterUtils;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.LocationConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import pabeles.concurrency.IntOperatorTask.Min;

/** Add your docs here. */
public class ShooterCalculator {

    CommandSwerveDrivetrain commandSwerveDrivetrain;

    public ShooterCalculator(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    }

    public double calculateFlywheelSpeed(double distanceToTargetMeters, double launchAngleDegrees, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual physics calculations to determine the required wheel speed
        double requiredSpeed = Math.sqrt(9.81 * distanceToTargetMeters); // Simplified example
        return requiredSpeed;
    }

    public double calculateHoodAngle(double distanceToTargetMeters, double initialHeightMeters, double targetHeightMeters) {
        // Placeholder implementation
        // Replace with actual calculations to determine the required hood angle
        double angle = Math.toDegrees(Math.atan2(targetHeightMeters - initialHeightMeters, distanceToTargetMeters));
        return angle/360;
    }


    public double calculateFlywheelShootRPMFromCurrentPose()
    {

        double wheelSpeed = flywheelRPMFormula(commandSwerveDrivetrain.getDistanceToHub());
        
        SmartDashboard.putNumber("WheelSpeedShootCalculated", wheelSpeed);

        wheelSpeed = MathUtil.clamp(wheelSpeed, ShooterConstants.MIN_FLYWHEEL_SPEED, ShooterConstants.MAX_FLYWHEEL_SPEED);
        return wheelSpeed;
    }

    public double calculatePassRPMFromCurrentPose()
    {
        double wheelSpeed = pasRPMFormula(commandSwerveDrivetrain.getXDistanceToHub());

        SmartDashboard.putNumber("WheelSpeedPassCalculated", wheelSpeed);

        wheelSpeed = MathUtil.clamp(wheelSpeed, ShooterConstants.MIN_FLYWHEEL_SPEED, ShooterConstants.MAX_FLYWHEEL_SPEED);

        return wheelSpeed;
    }

    public double calculateRestFlywheelSpeedFromCurrentPose()
    {
        /*
         * A reimplementation of a pinhole like formula for use
         * 10(placeholder)/distance * restrpm
         */
        double rpm = 0;
        Pose2d pose = commandSwerveDrivetrain.getPose();
        double y = pose.getY();
        if ((y > LocationConstants.MIN_SHOOT_Y) && (y < LocationConstants.MAX_SHOOT_Y)) {
            rpm = (frc.robot.constants.ShooterConstants.IDLE_RPM_DIST/commandSwerveDrivetrain.getDistanceToHub())*frc.robot.constants.ShooterConstants.IDLE_RPM;

        } else {
            rpm = (frc.robot.constants.ShooterConstants.IDLE_RPM_DIST_PASS/commandSwerveDrivetrain.getXDistanceToHub())*calculatePassRPMFromCurrentPose();
        }

        rpm = MathUtil.clamp(rpm, ShooterConstants.IDLE_RPM_MIN, ShooterConstants.IDLE_RPM_MAX);
        return rpm;
    }
    
    //TODO: FIX THESE
    // formül kullanabilirsin ama yani iş kardeşim ya vaktin kalırsa yap
    // onun yerine interpolasyon daha iyi

     public static final double flywheelRPMFormula(double x)
    {
        double a = -29.72883;
        double b = 473.27393;
        double c = -2886.63609;
        double d = 8444.7507;
        double f = -11605.2592;
        double g = 7475.15141;

        if(x < 1.7)
        {
            return 1500;
        }

        double y = (((((a * x + b) * x + c) * x + d) * x + f) * x + g);

        if(x > 5)
        {
            y = 2631;
            y += 20*(x-5);
        }

        return y/0.81;
    }


    
    public static final double pasRPMFormula(double x)
    {
        double a = 350;
        double b = 1500;

        double y = a*x + b;

        return y;
    }

}