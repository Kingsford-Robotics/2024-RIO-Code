package frc.robot;

import org.ietf.jgss.Oid;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystems */
    private final Elevator s_Elevator = new Elevator();
    private final Intake s_Intake = new Intake();
    //private final Jetson s_Jetson = new Jetson();
    //private final LedDriver s_LedDriver = new LedDriver();
    //private final Limelight s_Limelight = new Limelight();
    private final Pivot s_Pivot = new Pivot();
    private final Shooter s_Shooter = new Shooter();
    private final Swerve s_Swerve = new Swerve();

    private final Command m_ElevatorTeleopCommand = s_Elevator.GetElevatorTeleop(() -> -OIConstants.elevatorSpeed.getAsDouble() * 0.2);
    private final Command m_PivotTeleopCommand = s_Pivot.GetPivotTeleop(() -> -OIConstants.pivotSpeed.getAsDouble() * 0.2);
    private final Command m_homePosition = new HomePosition(s_Elevator, s_Pivot);
    private final Command m_deployIntake = new DeployIntake(s_Elevator, s_Pivot, s_Intake);

    private enum targetMode {
        kSpeaker,
        kAmp,
        kPass
    }

    private targetMode m_TargetMode = targetMode.kSpeaker;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -OIConstants.translationSupplier.get(),
                () -> -OIConstants.strafeSupplier.get(),
                () -> -OIConstants.rotationSupplier.get(),
                () -> OIConstants.robotCentric.getAsBoolean(),
                () -> OIConstants.slowSpeed.getAsBoolean()
            )
        );

        s_Pivot.setDefaultCommand(
            m_PivotTeleopCommand
        );

        s_Elevator.setDefaultCommand(
            m_ElevatorTeleopCommand
        );

        s_Shooter.setDefaultCommand(
            new InstantCommand(() -> s_Shooter.setShooterPercent(-OIConstants.shooterSpeed.getAsDouble() * 1.0), s_Shooter)
        );

        s_Intake.setDefaultCommand(
            new InstantCommand(() -> s_Intake.setSpeed(OIConstants.intakeSpeed.getAsDouble()), s_Intake)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        OIConstants.homeButton.onTrue(m_homePosition);
        OIConstants.deployIntake.onTrue(m_deployIntake);

        /* Co-Driver Buttons */

        //Reverse Intake
        OIConstants.reverseIntake.onTrue(new InstantCommand(() -> s_Intake.setSpeed(-0.5), s_Intake))
            .onFalse(new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake));
        
        //Speaker Mode
        OIConstants.speakerTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kSpeaker));

        //Amp Mode
        OIConstants.ampTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kAmp));

        //Pass Mode
        OIConstants.passTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kPass));
        
        //TODO: Add climb commands.
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
