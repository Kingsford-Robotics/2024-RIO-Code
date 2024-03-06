// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class AmpAlign extends Command {
  PIDController thetaController;
  PIDController strafeController;
  RobotContainer container;

  double strafeFeedforward;
  double strafeKP;
  double strafeKI;
  double strafeKD;

  public AmpAlign(RobotContainer robotContainer) {
    container = robotContainer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strafeFeedforward = 0.020;
    strafeKP = 0.3;
    strafeKI = 0.0;
    strafeKD = 0.0;

    strafeController = new PIDController(strafeKP, strafeKI, strafeKD);
    strafeController.setSetpoint(0.0);
    strafeController.setTolerance(0.0);
    strafeController.reset();

    LimelightHelpers.setPipelineIndex("limelight", 1);  //Sets to amp pipeline.
    LimelightHelpers.setLEDMode_ForceOn("limelight");   //Turns on the LEDs for better targeting.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOffset = LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getX();
    double strafeOutput = strafeController.calculate(xOffset);
    strafeOutput = Math.max(Math.min(strafeOutput, 0.4), -0.4);

    if(Math.abs(strafeOutput) < strafeFeedforward){
      strafeOutput += Math.copySign(strafeFeedforward, strafeOutput);
    }

    if(Math.abs(strafeOutput) > Units.inchesToMeters(1.0)){
      container.setAutoAlignStrafe(-strafeOutput);
    }

    else{
      container.setAutoAlignStrafe(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    container.setAutoAlignTurn(0.0);
    container.setAutoAlignStrafe(0.0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}