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
        new ConditionalCommand(
          
          //If coming from home position, go up first and then set angle.
          new SequentialCommandGroup(
            elevator.setHeight(Units.inchesToMeters(12.78)),
            pivot.setPivotAngle(Rotation2d.fromDegrees(96.0))
          ),
        
        //Not coming from home position
        new ConditionalCommand(
        //If the angle is already greater than 8 degrees, can move pivot
        //and elevator in parallel.  
        new ParallelCommandGroup(
          pivot.setPivotAngle(Rotation2d.fromDegrees(96.0)),
          elevator.setHeight(Units.inchesToMeters(12.78))
          ),

          //If the angle is less than 8 degrees, first get angle to 8
          //and then move pivot and elevator in parallel.
          new SequentialCommandGroup(
            pivot.setPivotAngle(Rotation2d.fromDegrees(8.0)),
            new ParallelCommandGroup(
              elevator.setHeight(Units.inchesToMeters(12.78)),
              pivot.setPivotAngle(Rotation2d.fromDegrees(96))
            )
          ),
          () -> pivot.getCANcoder().getDegrees() > 8.0),

        () -> elevator.getHeight() > Units.inchesToMeters(10)),

      pivot.setPivotAngle(Rotation2d.fromDegrees(96)),
      new PrintCommand("Pivot to Angle"),
      new InstantCommand(() -> intake.setSpeed(0.3), intake),
      new InstantCommand(() -> shooter.setShooterPercent(0.3), shooter)
    );
  }
}