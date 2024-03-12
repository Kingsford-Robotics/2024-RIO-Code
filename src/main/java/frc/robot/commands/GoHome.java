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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class GoHome extends SequentialCommandGroup {
  /** Creates a new GoHome. */
  public GoHome(Elevator elevator, Pivot pivot) {
    addCommands(

    new SequentialCommandGroup(
      new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
      new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot)
    ),   

    new ConditionalCommand(
        //Not stuck under the home position, so it can go straight up.
        new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(11.25)), elevator),
         
        //Could be stuck under the home position, so pivot up first.
        new SequentialCommandGroup(
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10.0)), pivot),
            new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 8.0),
            new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(11.25)), elevator)
        ),
        () -> elevator.getHeight() > Units.inchesToMeters(9.5)
    ),
      new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(11)),
      new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(-2.0)), pivot),
      new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() < -1.5),
      new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(10.5)), elevator)
    );
  }
}