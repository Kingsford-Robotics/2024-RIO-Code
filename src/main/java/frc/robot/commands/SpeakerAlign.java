// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class SpeakerAlign extends Command {
  /** Creates a new SpeakerAlign. */
  PIDController pidController;
  RobotContainer container;



  ShuffleboardTab tab;

  double thetafeedforward;
  double thetaKP;
  double thetaKI;
  double thetaKD;

  public SpeakerAlign(RobotContainer robotContainer) {
    container = robotContainer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaKP = 0.008;
    thetaKI = 0.0;
    thetaKD = 0.001;
    thetafeedforward= 0.015;
    pidController = new PIDController(thetaKP, thetaKI, thetaKD);
    pidController.setSetpoint(0.0);
    pidController.setTolerance(0.0);
    pidController.reset();
    LimelightHelpers.setPipelineIndex("limelight", 0);  //Sets to speaker pipeline.
    LimelightHelpers.setLEDMode_ForceOn("limelight");   //Turns on the LEDs for better targeting.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var tx = LimelightHelpers.getTX("limelight");
    double output = pidController.calculate(tx);
    output = Math.max(Math.min(output, 0.4), -0.4);

    if(Math.abs(output) < thetafeedforward){
      output += Math.copySign(thetafeedforward, output);
    }

    if(Math.abs(tx) > 0.5){
      container.setAutoAlignTurn(output);
    }

    else{
      container.setAutoAlignTurn(0.0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    container.setAutoAlignTurn(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}