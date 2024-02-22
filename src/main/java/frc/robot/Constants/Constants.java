package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class Swerve {
        public static final int pigeonID = 35;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(27.25);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(189.22);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(107.3);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(167.4);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 34;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(256.46);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class ElevatorConstants {
        //CAN and Port IDs
        public static final int elevatorMotorID = 9;
        public static final int topLimitSwitchID = 0;
        public static final int bottomLimitSwitchID = 1;

        //Elevator Travel Constants
        public static final double elevatorMaxTravel = Units.inchesToMeters(12.87);
        public static final double softLimit = Units.inchesToMeters(1); //Slows speed down when close to limit.
        public static final double errorThreshold = Units.inchesToMeters(0.125); //0.125 tolerance for positions.

        //Sensor to Mechanism Ratio
        private static final double gearRatio = 28.0;
        private static final double sprocketDiameter = Units.inchesToMeters(1.77);
        public static final double sensorToMechRatio = gearRatio / (sprocketDiameter * Math.PI);

        //PID Values
        public static final double kP = 100;
        public static final double kI = 5;

        //Feedforward Values
        public static final double kV = 22.37;
        public static final double kA = 0.03;
        public static final double kG = 0.24;

        //Motion Magic Values
        public static final double cruiseVelocity = Units.inchesToMeters(10);
        public static final double acceleration = Units.inchesToMeters(20);
        public static final double jerk = Units.inchesToMeters(40);

        //Current Limiting
        public static final int currentLimit = 40;
    }

    public static final class IntakeConstants {
        public static final int intakeMotorID = 17;
        public static final int beamBreakID = 4;
    }

    public static final class ShooterConstants {
        public static final int shooterLeftMotorID = 15;
        public static final int shooterRightMotorID = 14;

        //PID Values
        public static final double shooterKP = 0;
        public static final double shooterKI = 0;

        public static final double shooterKF = 0;
        public static final double shooterRampRate = 0;
        public static final int shooterToleranceRPM = 10;
    }

    public static final class PivotConstants {
        public static final int pivotLeftMotorID = 21;
        public static final int pivotRightMotorID = 20;
        public static final int pivotCurrentLimit = 30;
        public static final double openLoopRamp = 0.2;
        public static final double closedLoopRamp = 0.2;

        public static final int pivotDownLimitSwitchID = 2;
        public static final int pivotUpLimitSwitchID = 3;

        public static final int pivotAbsoluteEncoderID = 30;
        public static final Rotation2d pivotAngleOffset = Rotation2d.fromDegrees(130);

        //PID Values
        public static final double pivotKP = 5.0;
        public static final double pivotKI = 0.0;
        public static final double pivotKD = 0.0;

        public static final double pivotKG = 0.19;
        public static final double pivotKV = 3.41;
        public static final double pivotKA = 0.01;
        public static final double pivotKS = 0.0;

        public static final Rotation2d pivotAbsoluteOffset = Rotation2d.fromDegrees(0);
    }

    public static final class LedConstants {
        public static final int ledDriverID = 0;
    }
}