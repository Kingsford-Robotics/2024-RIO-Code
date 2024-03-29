// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

import org.photonvision.PhotonCamera;

public class FastAutoTrackNote extends Command {
  /** Creates a new TrackNote. */
  private Swerve swerve;
  private PIDController strafeController;

  private PhotonCamera photonCamera;
  private Timer timer;

  private final double maxSpeed = 0.75;
  private final double trackingSpeed = 3.0;
  
  double strafeKP;
  double strafeKI;
  double strafeKD;

  public FastAutoTrackNote(Swerve swerve) {
    this.swerve = swerve;
    timer = new Timer();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strafeKP = 0.1;
    strafeKI = 0.0;
    strafeKD = 0.0;
    strafeController = new PIDController(strafeKP, strafeKI, strafeKD);
    strafeController.setSetpoint(0.0);
    strafeController.setTolerance(0.0);
    strafeController.reset();

    photonCamera = new PhotonCamera("PhotonVision");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = photonCamera.getLatestResult();

    if (result.hasTargets()) {
      timer.reset();
      var target = result.getBestTarget();
      var xOffset = target.getYaw();
      var targetArea = target.getArea();

      if(targetArea > 0.05){
        double output = strafeController.calculate(xOffset);
        output = MathUtil.clamp(output, -maxSpeed, maxSpeed);

        swerve.drive(new Translation2d(-trackingSpeed, -output), 0.0, false, true);
      }
      else{
        swerve.drive(new Translation2d(-trackingSpeed, 0.0), 0.0, false, true);
      }
    }
    else{
      timer.start();
      swerve.drive(new Translation2d(-trackingSpeed, 0.0), 0.0, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, true);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0);
  }
}