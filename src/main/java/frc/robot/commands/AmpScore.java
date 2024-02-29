// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class AmpScore extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  
  public AmpScore(Pivot pivot, Elevator elevator, Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(
      /* 
      new ConditionalCommand(
          //If coming from home position, go up first and then set angle.
          new SequentialCommandGroup(
            elevator.setHeight(Units.inchesToMeters(12.78)),
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(90.0)), pivot),
            new WaitUntilCommand(pivot::reachedSetpoint)
          ),
        
        //Not coming from home position
        new ConditionalCommand(
        //If the angle is already greater than 8 degrees, can move pivot
        //and elevator in parallel.  
        new ParallelCommandGroup(
          new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(90.0)), pivot),
          new WaitUntilCommand(pivot::reachedSetpoint),
          elevator.setHeight(Units.inchesToMeters(12.78))
          ),

          //If the angle is less than 8 degrees, first get angle to 8
          //and then move pivot and elevator in parallel.
          new SequentialCommandGroup(
            new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(8.0)), pivot),
            new WaitUntilCommand(pivot::reachedSetpoint),
            new ParallelCommandGroup(
              elevator.setHeight(Units.inchesToMeters(12.78)),
              new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(90)), pivot),
              new WaitUntilCommand(pivot::reachedSetpoint)
            )
          ),
          () -> pivot.getCANcoder().getDegrees() > 8.0),

        () -> elevator.getHeight() > Units.inchesToMeters(10)),

      new InstantCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(90)), pivot),
      new WaitUntilCommand(pivot::reachedSetpoint),
      new PrintCommand("Pivot to Angle"),
      new InstantCommand(() -> intake.setSpeed(1), intake),
      new InstantCommand(() -> shooter.setShooterPercent(-0.4), shooter)
      */
    );
  }
}