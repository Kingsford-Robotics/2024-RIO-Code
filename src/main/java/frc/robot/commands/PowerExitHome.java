// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class PowerExitHome extends SequentialCommandGroup {
  /** Creates a new PowerExitHome. */
  public PowerExitHome(Elevator elevator, Pivot pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> elevator.setSpeed(0.1), elevator),
      new InstantCommand(() -> elevator.resetLimitCheck(), elevator),
      new WaitUntilCommand(() -> elevator.getTopLimitPressed()),
      new InstantCommand(() -> pivot.setSpeed(0.3), pivot),
      new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 10.0),
      new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10)), pivot)
    );
  }
}