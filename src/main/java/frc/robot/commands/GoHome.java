// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoHome extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHome(Elevator elevator, Pivot pivot) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        elevator.setHeight(Units.inchesToMeters(12.78)),  
        new ConditionalCommand(
          new ParallelCommandGroup(
          pivot.setPivotAngle(Rotation2d.fromDegrees(8.0)),
          elevator.setHeight(Units.inchesToMeters(12.78))
          ),

          new SequentialCommandGroup(
            pivot.setPivotAngle(Rotation2d.fromDegrees(8.0)),
            elevator.setHeight(Units.inchesToMeters(12.78))
          ),
          () -> pivot.getCANcoder().getDegrees() > 8.0),
        () -> elevator.getHeight() > Units.inchesToMeters(10)),

      pivot.setPivotAngle(Rotation2d.fromDegrees(-12.3)),
      elevator.setHeight(Units.inchesToMeters(12.3))
    );
  }
}
