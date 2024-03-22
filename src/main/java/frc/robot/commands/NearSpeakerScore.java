// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class NearSpeakerScore extends SequentialCommandGroup {
  /** Creates a new NearSpeakerScore. */
  public NearSpeakerScore(Elevator elevator, Intake intake, Pivot pivot, Shooter shooter) {
    addCommands(
      //Stop elevator and pivot motions.
      new SequentialCommandGroup(
          new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
          new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot),
          new InstantCommand(() -> LimelightHelpers.setPipelineIndex("limelight", 0))
      ),  

      new InstantCommand(() -> shooter.setShooterPercent(1.0), shooter),

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

      new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(17.0))), //TODO: Tune this angle.
        
      new WaitUntilCommand(() -> pivot.angleErrorDegrees() < 1.5 && elevator.reachedSetpoint() && shooter.getShooterRPM() > 3000.0).withTimeout(2.0),
      new InstantCommand(() -> intake.setSpeed(1.0), intake),
      new WaitUntilCommand(() -> false)   
    );
  }
}