package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
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
    public final Elevator s_Elevator;
    private final Intake s_Intake;
    public final Pivot s_Pivot;
    private final Shooter s_Shooter;
    private final Swerve s_Swerve;
    private final CompetitionData s_CompetitionData;
    private final LedDriver s_LedDriver;

    public enum targetMode {
        kSpeaker,
        kAmp,
        kMidfield,
        kNear,
        kPodium
    }

    public targetMode m_TargetMode = targetMode.kSpeaker;

    private SequentialCommandGroup m_AmpScore;
    private Command m_deployIntake;
    private SequentialCommandGroup m_SpeakerScore;

    private double autoAlignTurn;       //Supplies a value to control the angle to a setpoint while driving.
    private double autoAlignStrafe;     //Supplies a value to control the side-to-side position while driving.
    private double autoDrive;
    private boolean intakeCentric = false;

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Elevator = new Elevator();
        s_Intake = new Intake();
        s_Pivot = new Pivot();
        s_Shooter = new Shooter();
        s_Swerve = new Swerve();
        s_CompetitionData = new CompetitionData(this, s_Elevator, s_Swerve);
        s_LedDriver = new LedDriver();
        
        NamedCommands.registerCommand("lowSpeakerScore", new LowSpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter, s_Swerve).withTimeout(5));
        NamedCommands.registerCommand("intake", new DeployIntake(s_Elevator, s_Pivot, s_Intake));
        NamedCommands.registerCommand("home", new GoHome(s_Elevator, s_Pivot));
        NamedCommands.registerCommand("stopIntakeShooter", new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake));
        NamedCommands.registerCommand("trackNote", new AutoTrackNote(s_Swerve).withTimeout(2.5));
        NamedCommands.registerCommand("fastTrackNote", new FastAutoTrackNote(s_Swerve).withTimeout(1.75));
        NamedCommands.registerCommand("lowerArm", new LowerArm(s_Elevator, s_Intake, s_Pivot));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Competition").add(autoChooser);

        autoAlignTurn = 0.0;
        autoAlignStrafe = 0.0;
        autoDrive = 0.0;

        m_AmpScore = new AmpScore(s_Pivot, s_Elevator, s_Intake, s_Shooter, RobotContainer.this);
        m_deployIntake = new DeployIntake(s_Elevator, s_Pivot, s_Intake);
        m_SpeakerScore = new SpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter, RobotContainer.this);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -OIConstants.translationSupplier.get(),
                () -> -OIConstants.strafeSupplier.get(),
                () -> -OIConstants.rotationSupplier.get(),
                () -> OIConstants.robotCentric.getAsBoolean(),
                () -> intakeCentric,
                () -> OIConstants.slowSpeed.getAsBoolean(),
                () -> autoAlignTurn,
                () -> autoAlignStrafe,
                () -> autoDrive
            )
        );
        
        s_Intake.setDefaultCommand(
            new RunCommand(
                () -> s_Intake.setSpeed(OIConstants.intakeSpeed.getAsDouble()), 
                s_Intake
            )
        );

        s_Shooter.setDefaultCommand(
            new RunCommand(
                () -> s_Shooter.setShooterPercent(OIConstants.shooterSpeed.getAsDouble()), 
                s_Shooter
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /*Controller Mappings*/
        
        //A Button
        OIConstants.ampTarget.onTrue(new InstantCommand(()-> m_TargetMode = targetMode.kAmp));

        //Y Button
        OIConstants.speakerTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kSpeaker));

        //B Button
        OIConstants.trapTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kMidfield));

        //POV Up Button
        OIConstants.podiumManualShot.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kPodium));

        //POV Down Button
        OIConstants.nearManualShot.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kNear));

        //X Button
        OIConstants.reverseIntake.whileTrue(new RunCommand(() -> s_Intake.setSpeed(-0.25), s_Intake)).
            onFalse(new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake));

        //Left Bumper
        OIConstants.manualActive.whileTrue(
            new ParallelCommandGroup(
                s_Pivot.manualControl(() -> -OIConstants.pivotSpeed.getAsDouble() * 0.2),
                s_Elevator.manualControl(() -> -OIConstants.elevatorSpeed.getAsDouble() * 0.4)
            )
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Elevator.setHeight(s_Elevator.getHeight()), s_Elevator),
                new InstantCommand(() -> s_Pivot.setPivotAngle(s_Pivot.getCANcoder()), s_Pivot)
            )
        );

        //Climber Deploy
        OIConstants.climberDeploy.whileTrue(
            new ClimbDeploy(s_Pivot, s_Elevator)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Elevator.setHeight(s_Elevator.getHeight()), s_Elevator),
                new InstantCommand(() -> s_Pivot.setPivotAngle(s_Pivot.getCANcoder()), s_Pivot)
            )
        );

        //Climate Retract
        OIConstants.climberRetract.whileTrue(
            new ClimbRetract(s_Elevator)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Elevator.setHeight(s_Elevator.getHeight()), s_Elevator),
                new InstantCommand(() -> s_Pivot.setPivotAngle(s_Pivot.getCANcoder()), s_Pivot)
            )
        );

        /*Driver Mappings */

        //Left Stick Center Button
        OIConstants.homeButton.whileTrue(
            new GoHome(s_Elevator, s_Pivot)
        );

        //Drive Left Trigger
        OIConstants.deployIntake.whileTrue(
            new ParallelRaceGroup(
                m_deployIntake,
                new TrackNote(RobotContainer.this)
            ).finallyDo(
                (interrupted) -> {
                    new SequentialCommandGroup(
                        new WaitCommand(0.3),
                        new InstantCommand(() -> s_Intake.setSpeed(0.0)),
                        new GoHome(s_Elevator, s_Pivot)
                    ).schedule();
                }
            )
        );

        new Trigger(() -> s_Intake.getBeamBreak()).onTrue(
            new SequentialCommandGroup(
                new InstantCommand( () -> s_LedDriver.setColor(LedColor.StrobeWhite), s_LedDriver),
                new WaitCommand(2),
                new InstantCommand(() -> s_LedDriver.setColor(LedColor.Blue), s_LedDriver)
            )
        );

        new Trigger(() -> s_Elevator.getHomeLimitSwitch()).whileTrue(
            new InstantCommand(() -> s_LedDriver.setColor(LedColor.Green), s_LedDriver)
        ).onFalse(
            new InstantCommand(() -> s_LedDriver.setColor(LedColor.Blue))
        );

        OIConstants.deployIntake.onFalse(
            new InstantCommand(() -> setIntakeCentric(false)) 
        );

        //Drive Right Trigger
        OIConstants.shoot.whileTrue(
            new ConditionalCommand(
                m_SpeakerScore,
                    new ConditionalCommand(
                        m_AmpScore,
                        new ConditionalCommand(
                            new MidfieldShot(s_Elevator, s_Intake, s_Pivot, s_Shooter),
                                new ConditionalCommand(
                                    new PodiumSpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter),
                                    new NearSpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter),
                                    () -> m_TargetMode == targetMode.kPodium
                                ),
                            () -> m_TargetMode == targetMode.kMidfield
                        ),
                        () -> m_TargetMode == targetMode.kAmp
                    ),
                () -> m_TargetMode == targetMode.kSpeaker
            )
        ).onFalse(
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Shooter.setShooterPercent(0.0), s_Shooter),
                new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake),
                new GoHome(s_Elevator, s_Pivot)
            )
        );

        //Drive Right Stick Center Button
        OIConstants.resetGyro.onTrue(
            new InstantCommand(() -> s_Swerve.zeroHeading(), s_Swerve)
        );

        OIConstants.snapFront.whileTrue(
            new TurnToAngle(RobotContainer.this, Rotation2d.fromDegrees(0.0), s_Swerve)
        );

         OIConstants.snapBack.whileTrue(
            new TurnToAngle(RobotContainer.this, Rotation2d.fromDegrees(-180), s_Swerve)
        );

         OIConstants.snapLeft.whileTrue(
            new TurnToAngle(RobotContainer.this, Rotation2d.fromDegrees(90), s_Swerve)
        );

         OIConstants.snapRight.whileTrue(
            new TurnToAngle(RobotContainer.this, Rotation2d.fromDegrees(-90), s_Swerve)
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
      }

    public void setAutoAlignTurn(double value) {
        autoAlignTurn = value;
    }
    
    public void setAutoAlignStrafe(double value) {
        autoAlignStrafe = value;
    }

    public void setAutoDrive(double value){
        autoDrive = value;
    }

    public void setIntakeCentric(boolean value) {
        intakeCentric = value;
    }
}