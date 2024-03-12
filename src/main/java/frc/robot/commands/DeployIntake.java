// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LedColor;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedDriver;
import frc.robot.subsystems.Pivot;

public class DeployIntake extends SequentialCommandGroup {
  public DeployIntake(Elevator elevator, Pivot pivot, Intake intake, RobotContainer container, LedDriver ledDriver) {
    addCommands( 
      new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setHeight(elevator.getHeight()), elevator),
            new InstantCommand(() -> pivot.setPivotAngle(pivot.getCANcoder()), pivot)
          ), 

        new ParallelRaceGroup(
          new SequentialCommandGroup(
            new ConditionalCommand(
              new SequentialCommandGroup(
                new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(11.0)), elevator),
                new WaitUntilCommand(() -> elevator.getHeight() > Units.inchesToMeters(10.5)),
                new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10.0)), pivot)
              ),
                new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(10.0)), pivot)
              , 
              () -> elevator.getHeight() > Units.inchesToMeters(9.75)),
            new WaitUntilCommand(() -> pivot.getCANcoder().getDegrees() > 8.0),
            new InstantCommand(() -> elevator.setHeight(Units.inchesToMeters(0.0)), elevator),
            new WaitUntilCommand(() -> elevator.getHeight() < Units.inchesToMeters(1.0)),
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(-1.75)), pivot),
            new InstantCommand(() -> intake.setSpeed(0.8), intake),
            new WaitUntilCommand(() -> !intake.getBeamBreak()),

            new InstantCommand(() -> ledDriver.setColor(LedColor.StrobeWhite), ledDriver)
          ),
          new TrackNote(container)
      )
    );
  }
}