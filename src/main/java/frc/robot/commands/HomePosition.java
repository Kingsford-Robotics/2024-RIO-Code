// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class HomePosition extends SequentialCommandGroup {
  /** Creates a new Home. */
  public HomePosition(Elevator elevator, Pivot pivot) {
    super(
      new ParallelCommandGroup(elevator.setHeight(0.0), pivot.setPivotAngle(Rotation2d.fromDegrees(90))),
      pivot.setPivotAngle(Rotation2d.fromDegrees(0)),
      elevator.setHeight(0.0)
    );
  }
}