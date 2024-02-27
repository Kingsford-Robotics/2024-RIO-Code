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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class MaxDist extends SequentialCommandGroup {
  /** Creates a new SpeakerScore. */
  public MaxDist(Elevator elevator, Intake intake, Pivot pivot, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          elevator.setHeight(Units.inchesToMeters(12.78)),
          pivot.setPivotAngle(Rotation2d.fromDegrees(30.0))
        ), 
        new ConditionalCommand(
          new ParallelCommandGroup(
            pivot.setPivotAngle(Rotation2d.fromDegrees(30.0)),
            elevator.setHeight(Units.inchesToMeters(12.78))
          ),
          new SequentialCommandGroup(
            pivot.setPivotAngle(Rotation2d.fromDegrees(8.0)),
            new ParallelCommandGroup(
              elevator.setHeight(Units.inchesToMeters(12.78)),
              pivot.setPivotAngle(Rotation2d.fromDegrees(30))
            )
          ),
          () -> pivot.getCANcoder().getDegrees() > 8.0), 
        () -> elevator.getHeight() > Units.inchesToMeters(10) && pivot.getCANcoder().getDegrees() < 8.0
      ),

      //Fix this logic once I get RPMs setup. Wait until within speed tolerance.
      new InstantCommand(() -> shooter.setShooterPercent(-1), shooter),
      new WaitCommand(4.0),
      new InstantCommand(() -> intake.setSpeed(1.0), intake)
    );
  }
}