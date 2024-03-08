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
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbDeploy extends SequentialCommandGroup {
  /** Creates a new ClimbDeploy. */
  public ClimbDeploy(Pivot pivot, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //Stop elevator and pivot motions.
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
        new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot),
        new InstantCommand(() -> elevator.retractActuator(), elevator)
      ),

      new SequentialCommandGroup(
          new ConditionalCommand(
          //Coming from not in home.
          new SequentialCommandGroup(
              new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10)), pivot),
              new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 8.0),
              new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(12.8)), elevator),
              new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(11))
          ),

          //Coming from in home
          new SequentialCommandGroup(
              new InstantCommand(() ->elevator.setHeight(Units.inchesToMeters(12.8)), elevator),
              new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(11))
          ),
          () -> elevator.getHeight() < Units.inchesToMeters(9.75)
        )
      ),

      new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(95)), pivot),
      new WaitUntilCommand(pivot::reachedSetpoint)
    );
  }
}
