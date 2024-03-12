// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import org.photonvision.PhotonCamera;

public class TrackNote extends Command {
  /** Creates a new TrackNote. */
  private RobotContainer container;
  private PIDController strafeController;

  private PhotonCamera photonCamera;

  double strafefeedforward;
  double strafeKP;
  double strafeKI;
  double strafeKD;

  public TrackNote(RobotContainer container) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.container = container;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strafeKP = 0.005;
    strafeKI = 0.0;
    strafeKD = 0.0;
    strafeController = new PIDController(strafeKP, strafeKI, strafeKD);
    strafeController.setSetpoint(0.0);
    strafeController.setTolerance(0.0);
    strafeController.reset();

    photonCamera = new PhotonCamera("PhotonVision");

    container.setIntakeCentric(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = photonCamera.getLatestResult();

    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var xOffset = target.getYaw();
      var targetArea = target.getArea();

      if(targetArea > 0.1){
        double output = strafeController.calculate(xOffset);
        output = MathUtil.clamp(output, -0.4, 0.4);

        container.setAutoAlignStrafe(output);
      }
      else{
        container.setAutoAlignStrafe(0.0);
      }
    }
    else{
      container.setAutoAlignStrafe(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    container.setAutoAlignStrafe(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
