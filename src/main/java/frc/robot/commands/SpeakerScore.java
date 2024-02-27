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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class SpeakerScore extends SequentialCommandGroup {
  /** Creates a new SpeakerScore. */
  public SpeakerScore(Elevator elevator, Intake intake, Pivot pivot, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          elevator.setHeight(Units.inchesToMeters(12.78)),
          new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(20.0)), pivot),
          new WaitUntilCommand(pivot::reachedSetpoint)
        ), 
        new ConditionalCommand(
          new ParallelCommandGroup(
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(20.0)), pivot),
            new WaitUntilCommand(pivot::reachedSetpoint),
            elevator.setHeight(Units.inchesToMeters(12.78))
          ),
          new SequentialCommandGroup(
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(8.0)), pivot),
            new WaitUntilCommand(pivot::reachedSetpoint),
            new ParallelCommandGroup(
              elevator.setHeight(Units.inchesToMeters(12.78)),
              new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(20)), pivot),
              new WaitUntilCommand(pivot::reachedSetpoint)
            )
          ),
          () -> pivot.getCANcoder().getDegrees() > 8.0), 
        () -> elevator.getHeight() > Units.inchesToMeters(10) && pivot.getCANcoder().getDegrees() < 8.0
      ),

      //Fix this logic once I get RPMs setup. Wait until within speed tolerance.
      new InstantCommand(() -> shooter.setShooterPercent(-0.7), shooter),
      new WaitCommand(1.0),
      new InstantCommand(() -> intake.setSpeed(1.0), intake)
    );
  }
}