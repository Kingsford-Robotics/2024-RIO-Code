// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class SpeakerScore extends SequentialCommandGroup {
  /** Creates a new SpeakerScore. */
    public SpeakerScore(Elevator elevator, Intake intake, Pivot pivot, Shooter shooter, RobotContainer container) {

        //TODO: Add logic to adjust angle based on distance from target.
        addCommands(
            new InstantCommand(() -> shooter.setShooterPercent(1.0), shooter),
            new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(11.00)), elevator),
            new ParallelCommandGroup(
                new SpeakerAlign(container),

                new RunCommand(() -> {
                    // Calculate the desired angle based on the distance from the Limelight
                    
                    double distance = 
                    Math.sqrt(
                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX(), 2) + 
                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)
                    );
                    
                    //Distance is in meters but the calculateDesiredAngle function requires feet.
                    double desiredAngle = calculateDesiredAngle(Units.metersToFeet(distance));
                    pivot.setPivotAngle(Rotation2d.fromDegrees(desiredAngle));

                }, pivot),
                
                new SequentialCommandGroup(
                    //TODO: Check what RPM I need to spin up to. I don't think too much speed is every an issue, so I set to full power.
                    //TODO: Add check if the pivot is at the desired angle.
                    new WaitCommand(0.3),
                    new WaitUntilCommand(() -> pivot.angleErrorDegrees() < 2.0 && elevator.reachedSetpoint() && LimelightHelpers.getTX("limelight") < 2.0 && shooter.getShooterRPM() > 2500.0),
                    new InstantCommand(() -> intake.setSpeed(1.0), intake)
                )
            )
        );
    }

    private double calculateDesiredAngle(double distance) {
        // Define the data points
        /*double[] distances = {3.45, 4.5, 5.5, 6.5, 7.5, 8.5};
        double[] angles = {14.6, 19.6, 25.5, 27.5, 30.5, 33.0};
    

        // Find the two data points that the distance falls between
        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                // Calculate the fraction of the way that the distance is between the two data points
                double fraction = (distance - distances[i]) / (distances[i + 1] - distances[i]);
    
                // Linearly interpolate the angle
                return angles[i] + fraction * (angles[i + 1] - angles[i]);
            }
        }
    
        // If the distance is outside the range of the data points, return the nearest data point
        if (distance < distances[0]) {
            return angles[0];
        } else {
            return angles[angles.length - 1];
        }*/
        
        return -0.405124 * distance*distance + 8.44125* distance - 9.7179;

    }
}

//-0.405124 x^2 + 8.44125 x - 9.7179