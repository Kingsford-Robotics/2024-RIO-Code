// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;

public class ClimbRetract extends SequentialCommandGroup {
  /** Creates a new ClimbRetract. */
  public ClimbRetract(Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
      new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(0)), elevator),
      new WaitUntilCommand(() -> elevator.getHeight() < Units.inchesToMeters(1))
    );
  }
}
