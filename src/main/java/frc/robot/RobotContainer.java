package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Constants.ElevatorConstants;
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
    private final Elevator s_Elevator;
    private final Intake s_Intake;
    //private final Jetson s_Jetson = new Jetson();
    //private final LedDriver s_LedDriver = new LedDriver();
    //private final Limelight s_Limelight = new Limelight();
    private final Pivot s_Pivot;
    private final Shooter s_Shooter;
    private final Swerve s_Swerve;

    private CompetitionData s_CompetitionData;

    public enum targetMode {
        kSpeaker,
        kAmp
    }

    public targetMode m_TargetMode = targetMode.kSpeaker;

    private SequentialCommandGroup m_GoHome;
    private SequentialCommandGroup m_AmpScore;
    private Command m_deployIntake;
    private SequentialCommandGroup m_SpeakerScore;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Elevator = new Elevator();
        s_Intake = new Intake();
        //s_Jetson = new Jetson();
        //s_LedDriver = new LedDriver();
        //s_Limelight = new Limelight();
        s_Pivot = new Pivot();
        s_Shooter = new Shooter();
        s_Swerve = new Swerve();

        s_CompetitionData = new CompetitionData(this, s_Elevator);

        m_GoHome = new GoHome(s_Elevator, s_Pivot);
        m_AmpScore = new AmpScore(s_Pivot, s_Elevator, s_Intake, s_Shooter);
        m_deployIntake = new DeployIntake(s_Elevator, s_Pivot, s_Intake);
        m_SpeakerScore = new SpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter);

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
        //A Button
        OIConstants.ampTarget.onTrue(new InstantCommand(()-> m_TargetMode = targetMode.kAmp));

        OIConstants.speakerTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kSpeaker));

        OIConstants.climbDeploy.whileTrue(
            new ParallelCommandGroup(
                s_Pivot.manualControl(() -> -OIConstants.pivotSpeed.getAsDouble() * 0.2),
                s_Elevator.manualControl(() -> -OIConstants.elevatorSpeed.getAsDouble() * 0.4)
            )
        );

        OIConstants.climbRetract.whileTrue(
            new PowerExitHome(s_Elevator, s_Pivot)
        );

        //Left Stick Center Button
        OIConstants.homeButton.whileTrue(
            new GoHome(s_Elevator, s_Pivot)
        );

        //Drive left trigger
        OIConstants.deployIntake.whileTrue(
            m_deployIntake.finallyDo(
                (interrupted) -> {
                    new SequentialCommandGroup(
                        new WaitCommand(0.15),
                        new InstantCommand(() -> s_Intake.setSpeed(0.0)),
                        new GoHome(s_Elevator, s_Pivot)
                    ).schedule();
                }
            )
        );

        OIConstants.shoot.whileTrue(
            new ConditionalCommand(
                m_SpeakerScore, 
                m_AmpScore, 
                () -> m_TargetMode == targetMode.kSpeaker
            )
        ).onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setShooterPercent(0.0), s_Shooter),
                new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake),
                new GoHome(s_Elevator, s_Pivot)
            )
        );

        OIConstants.resetGyro.onTrue(
            new InstantCommand(() -> s_Swerve.zeroHeading(), s_Swerve)
        );

        /* Co-Driver Buttons */

        //Reverse Intake
        OIConstants.reverseIntake.whileTrue(new InstantCommand(() -> s_Intake.setSpeed(-0.3), s_Intake)).
            onFalse(new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake));
        
        //Speaker Mode
        OIConstants.speakerTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kSpeaker));

        //Amp Mode
        OIConstants.ampTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kAmp));
        
        //TODO: Add climb commands.*/
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
