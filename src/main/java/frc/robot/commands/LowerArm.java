// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class LowerArm extends SequentialCommandGroup {
  /** Creates a new SpeakerScore. */
    public LowerArm(Elevator elevator, Intake intake, Pivot pivot) {
        addCommands(
            //Stop elevator and pivot motions.
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
                new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot)
            ),  

            new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(12.9)), elevator),
            new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(12.7)),
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(12)), pivot),
            new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 10),
            new InstantCommand(() -> elevator.setHeight(0.3), elevator)
        );
    }
}