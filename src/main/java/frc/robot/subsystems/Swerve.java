package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    SwerveDrivePoseEstimator swervePoseEstimator;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    private ShuffleboardTab tab; 
    private GenericEntry frontLeftAngle;
    private GenericEntry frontLeftSpeed;
    private GenericEntry frontLeftOffset;

    private GenericEntry frontRightAngle;
    private GenericEntry frontRightSpeed;
    private GenericEntry frontRightOffset;

    private GenericEntry backLeftAngle;
    private GenericEntry backLeftSpeed;
    private GenericEntry backLeftOffset;

    private GenericEntry backRightAngle;
    private GenericEntry backRightSpeed;
    private GenericEntry backRightOffset;

    //Odometry Output
    private Field2d field;

    //Rev Power Distribution Panel Output
    private PowerDistribution pdp;

    public Swerve() {
        tab = Shuffleboard.getTab("Swerve");
        frontLeftAngle = tab.add("Front Left Angle", 0).getEntry();
        frontLeftSpeed = tab.add("Front Left Speed", 0).getEntry();
        frontLeftOffset = tab.add("Front Left Offset", 0).getEntry();
        
        frontRightAngle = tab.add("Front Right Angle", 0).getEntry();
        frontRightSpeed = tab.add("Front Right Speed", 0).getEntry();
        frontRightOffset = tab.add("Front Right Offset", 0).getEntry();

        backLeftAngle = tab.add("Back Left Angle", 0).getEntry();
        backLeftSpeed = tab.add("Back Left Speed", 0).getEntry();
        backLeftOffset = tab.add("Back Left Offset", 0).getEntry();

        backRightAngle = tab.add("Back Right Angle", 0).getEntry();
        backRightSpeed = tab.add("Back Right Speed", 0).getEntry();
        backRightOffset = tab.add("Back Right Offset", 0).getEntry();

        field = new Field2d();
        tab.add("Field", field);

        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        pdp = new PowerDistribution(36, ModuleType.kRev);

        tab.add("PDP", pdp);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        //swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void setDriveCurrentLimit(double maxCurrent){
        for(SwerveModule mod : mSwerveMods){
            mod.setDriveCurrentLimit(maxCurrent);
        }
    }

    public void setAngleCurrentLimit(double maxCurrent){
        for(SwerveModule mod : mSwerveMods){
            mod.setAngleCurrentLimit(maxCurrent);
        }
    }

    public void setDefaultCurrentLimits(){
        for(SwerveModule mod : mSwerveMods){
            mod.setDefaultCurrentLimits();
        }
    }

    public void updateOdometryFromVision(Pose2d pose, double timestamp){
        swervePoseEstimator.addVisionMeasurement(pose, timestamp);
    }

    @Override
    public void periodic(){
        swervePoseEstimator.update(getGyroYaw(), getModulePositions());

        //Update Shuffleboard values
        frontLeftAngle.setDouble(mSwerveMods[0].getState().angle.getDegrees());
        frontLeftSpeed.setDouble(mSwerveMods[0].getState().speedMetersPerSecond);
        frontLeftOffset.setDouble(Constants.Swerve.Mod0.angleOffset.getDegrees());

        frontRightAngle.setDouble(mSwerveMods[1].getState().angle.getDegrees());
        frontRightSpeed.setDouble(mSwerveMods[1].getState().speedMetersPerSecond);
        frontRightOffset.setDouble(Constants.Swerve.Mod1.angleOffset.getDegrees());

        backLeftAngle.setDouble(mSwerveMods[2].getState().angle.getDegrees());
        backLeftSpeed.setDouble(mSwerveMods[2].getState().speedMetersPerSecond);
        backLeftOffset.setDouble(Constants.Swerve.Mod2.angleOffset.getDegrees());

        backRightAngle.setDouble(mSwerveMods[3].getState().angle.getDegrees());
        backRightSpeed.setDouble(mSwerveMods[3].getState().speedMetersPerSecond);
        backRightOffset.setDouble(Constants.Swerve.Mod3.angleOffset.getDegrees());

        field.setRobotPose(getPose());
    }
}