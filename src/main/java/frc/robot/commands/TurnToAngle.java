// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class TurnToAngle extends Command {
  /** Creates a new TurnToAngle. */

  private PIDController pidController;
  private RobotContainer container;
  private Swerve swerve;

  private double thetaKP;
  private double thetaKI;
  private double thetaKD;

  private Rotation2d setAngle;

  public TurnToAngle(RobotContainer container, Rotation2d angle, Swerve swerve) {
    this.container = container;
    this.setAngle = angle;
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaKP = 0.35;
    thetaKI = 0.0;
    thetaKD = 0.02;
    pidController = new PIDController(thetaKP, thetaKI, thetaKD);
    pidController.setSetpoint(setAngle.getRadians());
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    pidController.setTolerance(0.0);
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d theta = swerve.getHeading();
    double output = pidController.calculate(theta.getRadians());
    output = MathUtil.clamp(output, -0.35, 0.35);

    /*if(Math.abs(output) < thetafeedforward){
      output += Math.copySign(thetafeedforward, output);
    }*/

    if(Math.abs(setAngle.minus(theta).getDegrees()) > 1.0){
      container.setAutoAlignTurn(output);
    }

    else{
      container.setAutoAlignTurn(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    container.setAutoAlignTurn(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
