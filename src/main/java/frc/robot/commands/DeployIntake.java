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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class DeployIntake extends SequentialCommandGroup {
  public DeployIntake(Elevator elevator, Pivot pivot, Intake intake) {
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          elevator.setHeight(Units.inchesToMeters(12.83)),
          new ConditionalCommand(
            new ParallelCommandGroup(
              pivot.setPivotAngle(Rotation2d.fromDegrees(9.0)),
              elevator.setHeight(Units.inchesToMeters(0.1))
            ),
            new SequentialCommandGroup(
              pivot.setPivotAngle(Rotation2d.fromDegrees(9.0)),
              elevator.setHeight(Units.inchesToMeters(0.1))
            ),
            () -> pivot.getCANcoder().getDegrees() > 9.0
          ),
          pivot.setPivotAngle(Rotation2d.fromDegrees(-5.0))
        ),
        new ConditionalCommand(
          new ParallelCommandGroup(
            pivot.setPivotAngle(Rotation2d.fromDegrees(9.0)),
            elevator.setHeight(Units.inchesToMeters(0.1))
          ),
          new SequentialCommandGroup(
            pivot.setPivotAngle(Rotation2d.fromDegrees(9.0)),
            elevator.setHeight(Units.inchesToMeters(0.1))
          ),
          () -> pivot.getCANcoder().getDegrees() > 9.0
        ),
        () -> elevator.getHeight() > Units.inchesToMeters(10) && pivot.getCANcoder().getDegrees() < 9.0
      ),
      new SequentialCommandGroup(
        pivot.setPivotAngle(Rotation2d.fromDegrees(-5.0)),
        new InstantCommand(() -> intake.setSpeed(1), intake),
        new WaitUntilCommand(() -> !intake.getBeamBreak())
      )
    );
  }
}