package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final class Swerve {
        public static final int pigeonID = 35;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.5);
        public static final double wheelBase = Units.inchesToMeters(27);
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
        public static final double driveKV = 2.88; //Old Value: 1.51
        public static final double driveKA = 0.26; //Old Value: 0.27

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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(190.986328125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 32;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(107.9296875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 33;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(168.3984375);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 34;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-106.259765625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class ElevatorConstants {
        //CAN and Port IDs
        public static final int elevatorMotorID = 9;
        public static final int topLimitSwitchID = 0;
        public static final int bottomLimitSwitchID = 1;
        public static final int homeLimitSwitchID = 2;
        public static final int linearActuatorID = 0;

        //Elevator Travel Constants
        public static final double elevatorMaxTravel = Units.inchesToMeters(12.95);
        public static final double softLimit = Units.inchesToMeters(1); //Slows speed down when close to limit.
        public static final double errorThreshold = Units.inchesToMeters(0.15); //0.15 tolerance for positions.

        //Sensor to Mechanism Ratio
        private static final double gearRatio = 27.0;
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
        public static final double cruiseVelocity = Units.inchesToMeters(20);   //12 inches per second
        public static final double acceleration = Units.inchesToMeters(50);     //40 inches per second squared
        public static final double jerk = Units.inchesToMeters(75);             //60 inches per second cubed

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
        public static final int shooterToleranceRPM = 50;
    }

    public static final class PivotConstants {
        public static final int pivotLeftMotorID = 21;
        public static final int pivotRightMotorID = 20;
        public static final int pivotCurrentLimit = 30;
        public static final double openLoopRamp = 0.2;
        public static final double closedLoopRamp = 0.2;

        public static final int pivotAbsoluteEncoderID = 30;
        public static final Rotation2d pivotAngleOffset = Rotation2d.fromDegrees(130);

        //PID Values
        public static final double pivotKP = 8.0;
        public static final double pivotKI = 0.5;
        public static final double pivotKD = 0.0;

        public static final double pivotKG = 0.25;
        public static final double pivotKV = 3.41;
        public static final double pivotKA = 0.01;
        public static final double pivotKS = 0.0;

        public static final Rotation2d pivotAbsoluteOffset = Rotation2d.fromDegrees(0);

        public static final double pivotVelocity = Units.degreesToRadians(130);          //130 degrees per second
        public static final double pivotAcceleration = Units.degreesToRadians(220);     //220 degrees per second squared
    }

    public static final class LedConstants {
        public static final int ledDriverID = 0;
    }
}