// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */

  private CANSparkMax pivotLeftMotor;
  private CANSparkMax pivotRightMotor;

  private SparkPIDController pivotPIDController;
  private RelativeEncoder pivotMotorEncoder;
  private CANcoder pivotAbsoluteEncoder;

  DigitalInput pivotDownLimitSwitch;
  DigitalInput pivotUpLimitSwitch;

  public Pivot() {
    pivotLeftMotor = new CANSparkMax(PivotConstants.pivotLeftMotorID, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(PivotConstants.pivotRightMotorID, MotorType.kBrushless);

    pivotPIDController = pivotLeftMotor.getPIDController();
    pivotMotorEncoder = pivotLeftMotor.getEncoder();

    pivotMotorEncoder.setPositionConversionFactor(360.0 / PivotConstants.pivotGearRatio);

    pivotAbsoluteEncoder = new CANcoder(PivotConstants.pivotAbsoluteEncoderID);
    pivotAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

    pivotDownLimitSwitch = new DigitalInput(PivotConstants.pivotDownLimitSwitchID);
    pivotUpLimitSwitch = new DigitalInput(PivotConstants.pivotUpLimitSwitchID);
    
    pivotLeftMotor.restoreFactoryDefaults();
    pivotRightMotor.restoreFactoryDefaults();

    pivotRightMotor.setInverted(true);
    pivotRightMotor.follow(pivotLeftMotor, true);

    pivotLeftMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);
    pivotRightMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);

    pivotLeftMotor.setOpenLoopRampRate(PivotConstants.openLoopRamp);
    pivotRightMotor.setOpenLoopRampRate(PivotConstants.openLoopRamp);

    pivotLeftMotor.setClosedLoopRampRate(PivotConstants.closedLoopRamp);
    pivotRightMotor.setClosedLoopRampRate(PivotConstants.closedLoopRamp);

    //Set motor to brake mode
    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    pivotPIDController.setP(PivotConstants.pivotKP);
    pivotPIDController.setI(PivotConstants.pivotKI);
    pivotPIDController.setD(PivotConstants.pivotKD);
    pivotPIDController.setFF(PivotConstants.pivotKF);
    pivotPIDController.setIZone(PivotConstants.pivotIZone);
    pivotPIDController.setIMaxAccum(PivotConstants.pivotIMaxAccum, 0);
    pivotPIDController.setSmartMotionMaxVelocity(PivotConstants.pivotMaxVelocity, 0);
    pivotPIDController.setSmartMotionMaxAccel(PivotConstants.pivotMaxAccel, 0);
    pivotPIDController.setSmartMotionAllowedClosedLoopError(PivotConstants.pivotAllowedError, 0);

    pivotLeftMotor.burnFlash();
    pivotRightMotor.burnFlash();
  }

  public void setPivotAngle(Rotation2d angle){
    pivotPIDController.setReference(angle.getDegrees(), ControlType.kSmartMotion);
  }

  public void setPivotSpeed(double speed){
    pivotLeftMotor.set(speed);
  }

  public Rotation2d getPivotAngle(){
    return Rotation2d.fromDegrees(pivotMotorEncoder.getPosition());
  }

  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(pivotAbsoluteEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute(){
    double absolutePosition = getCANcoder().getRotations() - PivotConstants.pivotAbsoluteOffset.getRotations();
    pivotMotorEncoder.setPosition(absolutePosition);
  }

  @Override
  public void periodic() {
  
    //Update the kF value for the pivot motor based on the angle cosine.
    //TODO: Check logic. Do not change value if calculation is the same.
    
    pivotPIDController.setFF(PivotConstants.pivotKF / Math.cos(getPivotAngle().getRadians()));
  }
}