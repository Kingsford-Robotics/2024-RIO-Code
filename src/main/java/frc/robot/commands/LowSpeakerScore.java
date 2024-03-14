// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class LowSpeakerScore extends SequentialCommandGroup {
  /** Creates a new SpeakerScore. */
    public LowSpeakerScore(Elevator elevator, Intake intake, Pivot pivot, Shooter shooter, Swerve swerve) {
        addCommands(
            //Stop elevator and pivot motions.
            new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
                new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot)
            ),  

            new InstantCommand(() -> shooter.setShooterPercent(1.0), shooter),

            new ParallelRaceGroup(
                new AutoSpeakerAlign(swerve),

                new SequentialCommandGroup(
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(12.85)), elevator),
                            new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(10.5)),
                            new InstantCommand(() -> 
                                {
                                    // Calculate the desired angle based on the distance from the Limelight

                                    double distance = 
                                    Math.sqrt(
                                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX(), 2) + 
                                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)
                                    );

                                    //Distance is in meters but the calculateDesiredAngle function requires feet.
                                    double desiredAngle = calculateDesiredAngle(Units.metersToFeet(distance));
                                    pivot.setPivotAngle(Rotation2d.fromDegrees(desiredAngle));
                                }, pivot
                            )
                        ),

                        new InstantCommand(() -> 
                                {
                                    // Calculate the desired angle based on the distance from the Limelight

                                    double distance = 
                                    Math.sqrt(
                                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX(), 2) + 
                                        Math.pow(LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)
                                    );

                                    //Distance is in meters but the calculateDesiredAngle function requires feet.
                                    double desiredAngle = calculateDesiredAngle(Units.metersToFeet(distance));
                                    pivot.setPivotAngle(Rotation2d.fromDegrees(desiredAngle));
                                }, pivot
                            ),
                        () -> elevator.getHeight() > Units.inchesToMeters(9.75)
                    ),

                    new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 8.0),
                    new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(0.1)), elevator)
                )
            ),

            new ParallelRaceGroup(
                new AutoSpeakerAlign(swerve),

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
                    new WaitUntilCommand(() -> pivot.angleErrorDegrees() < 1.0 && elevator.getHeight() < Units.inchesToMeters(1.5)),
                    new ParallelRaceGroup(
                        new WaitUntilCommand(() -> LimelightHelpers.getTX("limelight") < 2.0 && shooter.getShooterRPM() > 3000.0),
                        new WaitCommand(1.5)
                    ),
                    new InstantCommand(() -> intake.setSpeed(1.0), intake),
                    new WaitCommand(0.25)
                )
            )
        );
    }

    private double calculateDesiredAngle(double distance) {
        return MathUtil.clamp(4.35357 + 4.38036 * distance - 0.116071 * distance * distance, 16, 90);
    }
}
