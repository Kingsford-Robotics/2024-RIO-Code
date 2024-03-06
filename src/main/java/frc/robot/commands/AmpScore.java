// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class AmpScore extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  
  public AmpScore(Pivot pivot, Elevator elevator, Intake intake, Shooter shooter, RobotContainer container) {
      addCommands(
        //Stop elevator and pivot motions.
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
          new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot),
          new InstantCommand(() -> shooter.setShooterPercent(0.4), shooter)
        ),

        new SequentialCommandGroup(
          new ConditionalCommand(
          //Coming from not in home.
          new SequentialCommandGroup(
              new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10)), pivot),
              new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 8.0),
              new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(12.75)), elevator),
              new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(11))
          ),

          //Coming from in home
          new SequentialCommandGroup(
              new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(12.75)), elevator),
              new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(11))
          ),
          () -> elevator.getHeight() < Units.inchesToMeters(9.75)
        )
      ),

      new SequentialCommandGroup(
        new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(90)), pivot),
        new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 87.0),
        new InstantCommand(() -> intake.setSpeed(1.0), intake),
        new WaitCommand(1.0)
      )
    );
  }
}