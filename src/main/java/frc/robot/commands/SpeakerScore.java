// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class SpeakerScore extends SequentialCommandGroup {
  /** Creates a new SpeakerScore. */

  private double distance;
    public SpeakerScore(Elevator elevator, Intake intake, Pivot pivot, Shooter shooter, RobotContainer container) {
        addCommands(
            //Stop elevator and pivot motions.
            new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
                new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot),
                new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0))
            ),  

            new InstantCommand(() -> shooter.setShooterPercent(1.0), shooter),

            new ParallelRaceGroup(
                new ConditionalCommand(
                    //Coming from not in home.
                    new SequentialCommandGroup(
                        new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10)), pivot),
                        new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 8.0),
                        new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(11.50)), elevator)
                    ),

                    //Coming from in home
                    new SequentialCommandGroup(
                        new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(11.50)), elevator)
                    ),
                    () -> elevator.getHeight() < Units.inchesToMeters(9.75)
                ),

                new SpeakerAlign(container)
            ),
            
            new ParallelRaceGroup(
                new SpeakerAlign(container),

                new RunCommand(() -> {
                    // Calculate the desired angle based on the distance from the Limelight
                    
                    distance = 
                    Math.sqrt(
                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX(), 2) + 
                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)
                    );
                    
                    //Distance is in meters but the calculateDesiredAngle function requires feet.
                    double desiredAngle = calculateDesiredAngle(Units.metersToFeet(distance));
                    pivot.setPivotAngle(Rotation2d.fromDegrees(desiredAngle));

                }, pivot),
                
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> pivot.angleErrorDegrees() < 1.5 && elevator.reachedSetpoint() && LimelightHelpers.getTX("limelight") < 1.5 && LimelightHelpers.getTV("limelight") && distance < Units.feetToMeters(11) && shooter.getShooterRPM() > 3000.0).withTimeout(2),
                    new InstantCommand(() -> intake.setSpeed(1.0), intake),
                    new WaitUntilCommand(() -> false)
                )
            )
        );
    }

    private double calculateDesiredAngle(double distance) {
        return Math.max(-0.43544 * distance * distance + 9.29602 * distance - 12.2466, 15);
    }
}