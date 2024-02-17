// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  
  //Shuffleboard Data
  private ShuffleboardTab tab; 
  private GenericEntry absoluteAngelEntry;
  private GenericEntry pivotSpeedEntry;
  private GenericEntry pivotAngleSetpointEntry;
  private GenericEntry downLimitSwitchEntry;
  private GenericEntry upLimitSwitchEntry;

  private CANSparkMax pivotLeftMotor;
  private CANSparkMax pivotRightMotor;

  private CANcoder pivotAbsoluteEncoder;
  private ArmFeedforward pivotFeedforward;
  private ProfiledPIDController pidController;

  private DigitalInput pivotDownLimitSwitch;
  private DigitalInput pivotUpLimitSwitch;

  public Pivot() {
    //Shuffleboard Setup
    tab = Shuffleboard.getTab("Pivot");
    absoluteAngelEntry = tab.add("Absolute Angle", 0.0).getEntry();
    pivotSpeedEntry = tab.add("Pivot Speed", 0.0).getEntry();
    pivotAngleSetpointEntry = tab.add("Pivot Angle Setpoint", 0.0).getEntry();
    downLimitSwitchEntry = tab.add("Down Limit Switch", false).getEntry();
    upLimitSwitchEntry = tab.add("Up Limit Switch", false).getEntry();
    
    pivotLeftMotor = new CANSparkMax(PivotConstants.pivotLeftMotorID, MotorType.kBrushless);
    pivotRightMotor = new CANSparkMax(PivotConstants.pivotRightMotorID, MotorType.kBrushless);


    pivotAbsoluteEncoder = new CANcoder(PivotConstants.pivotAbsoluteEncoderID);

    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    //Put sensor 0 to 360 degrees absolute.
    canConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    pivotAbsoluteEncoder.getConfigurator().apply(canConfig);

    pivotFeedforward = new ArmFeedforward(PivotConstants.pivotKS, PivotConstants.pivotKG, PivotConstants.pivotKV);

    //TODO: Add velocity and acceleration constraints in constants.
    pidController = new ProfiledPIDController(PivotConstants.pivotKP, PivotConstants.pivotKI, 0.0, new Constraints(Units.degreesToRadians(20), Units.degreesToRadians(40)));

    pivotDownLimitSwitch = new DigitalInput(PivotConstants.pivotDownLimitSwitchID);
    pivotUpLimitSwitch = new DigitalInput(PivotConstants.pivotUpLimitSwitchID);
    
    pivotLeftMotor.restoreFactoryDefaults();
    pivotRightMotor.restoreFactoryDefaults();

    pivotLeftMotor.setInverted(false);
    pivotRightMotor.setInverted(true);

    pivotRightMotor.follow(pivotLeftMotor, true);

    pivotLeftMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);
    pivotRightMotor.setSmartCurrentLimit(PivotConstants.pivotCurrentLimit);

    pivotLeftMotor.setOpenLoopRampRate(PivotConstants.openLoopRamp);
    pivotRightMotor.setOpenLoopRampRate(PivotConstants.openLoopRamp);

    //pivotLeftMotor.setClosedLoopRampRate(PivotConstants.closedLoopRamp);
    //pivotRightMotor.setClosedLoopRampRate(PivotConstants.closedLoopRamp);

    //Set motor to brake mode
    pivotLeftMotor.setIdleMode(IdleMode.kBrake);
    pivotRightMotor.setIdleMode(IdleMode.kBrake);

    //pivotPIDController.setP(PivotConstants.pivotKP);
    //pivotPIDController.setI(PivotConstants.pivotKI);
    //pivotPIDController.setIZone(PivotConstants.pivotIZone);
    //pivotPIDController.setIMaxAccum(PivotConstants.pivotIMaxAccum, 0);

    pivotLeftMotor.burnFlash();
    pivotRightMotor.burnFlash();
  }

  public Command setPivotAngle(Rotation2d angle){
    return new Command() {
      {
        addRequirements(Pivot.this);
      }
    
      @Override
      public void initialize() {
        pidController.reset(getCANcoder().getRadians());
      }

      @Override
    public void execute() {
      pidController.setGoal(angle.getRadians());
      double output = pidController.calculate(getCANcoder().getRadians()); //+ pivotFeedforward.calculate(getCANcoder().getRadians(), pidController.getSetpoint().velocity) * 0.5;
      double constrainedOutput = Math.max(-0.2, Math.min(0.2, output));
      pivotLeftMotor.set(constrainedOutput);
      pivotAngleSetpointEntry.setDouble(output);
    }

    @Override
    public boolean isFinished() {
      return pidController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
      setPivotSpeed(0.0);
    }
  }; 
}

  public void setPivotSpeed(double speed){
    if(getPivotUpLimitSwitch() && speed > 0){
      speed = 0;
    }

    else if(getPivotDownLimitSwitch() && speed < 0){
      speed = 0;
    }

    pivotLeftMotor.set(speed);
  }

  public Rotation2d getCANcoder(){

    //TODO: Adjust offset to pull from constant.
    return Rotation2d.fromRotations(pivotAbsoluteEncoder.getAbsolutePosition().getValue()).minus(Rotation2d.fromDegrees(130.0));
  }

  //*Returns a value from -1 to 1 */
  public Rotation2d getPivotSpeed(){
    return Rotation2d.fromRotations(pivotAbsoluteEncoder.getVelocity().getValueAsDouble());
  }

  public boolean getPivotUpLimitSwitch(){
    return pivotUpLimitSwitch.get();
  }

  public boolean getPivotDownLimitSwitch(){
    return pivotDownLimitSwitch.get();
  }

  @Override
  public void periodic() {
    absoluteAngelEntry.setDouble(getCANcoder().getDegrees());
    pivotSpeedEntry.setDouble(getPivotSpeed().getDegrees());
    downLimitSwitchEntry.setBoolean(pivotDownLimitSwitch.get());
    upLimitSwitchEntry.setBoolean(pivotUpLimitSwitch.get());

    //Limit Switch Hard Stops
    
    if(getPivotUpLimitSwitch() && getPivotSpeed().getDegrees() > 0.0)
    {
      setPivotSpeed(0.0);
    }

    else if(getPivotDownLimitSwitch() && getPivotSpeed().getDegrees() < 0.0){
      setPivotSpeed(0.0);
    }
  }
}