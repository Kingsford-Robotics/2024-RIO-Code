package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private final Elevator s_Elevator;
    private final Intake s_Intake;
    private final Pivot s_Pivot;
    private final Shooter s_Shooter;
    private final Swerve s_Swerve;
    private final CompetitionData s_CompetitionData;

    public enum targetMode {
        kSpeaker,
        kAmp,
        kTrap
    }

    public targetMode m_TargetMode = targetMode.kSpeaker;

    private SequentialCommandGroup m_AmpScore;
    private Command m_deployIntake;
    private SequentialCommandGroup m_SpeakerScore;

    private double autoAlignTurn;       //Supplies a value to control the angle to a setpoint while driving.
    private double autoAlignStrafe;     //Supplies a value to control the side-to-side position while driving.
    private boolean intakeCentric = false;

    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Elevator = new Elevator();
        s_Intake = new Intake();
        s_Pivot = new Pivot();
        s_Shooter = new Shooter();
        s_Swerve = new Swerve();
        s_CompetitionData = new CompetitionData(this, s_Elevator);


        Limelight s_Limelight = new Limelight();

        autoAlignTurn = 0.0;
        autoAlignStrafe = 0.0;

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
                () -> autoAlignStrafe
            )
        );
        
        m_AmpScore = new AmpScore(s_Pivot, s_Elevator, s_Intake, s_Shooter, RobotContainer.this);
        m_deployIntake = new DeployIntake(s_Elevator, s_Pivot, s_Intake, RobotContainer.this);
        m_SpeakerScore = new SpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter, RobotContainer.this);

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

        NamedCommands.registerCommand("highSpeakerScore", new HighSpeakerAuton(s_Elevator, s_Intake, s_Pivot, s_Shooter));
        NamedCommands.registerCommand("lowSpeakerScore", new LowSpeakerScore(s_Elevator, s_Intake, s_Pivot, s_Shooter, s_Swerve));
        NamedCommands.registerCommand("ampScore", new AmpScore(s_Pivot, s_Elevator, s_Intake, s_Shooter, RobotContainer.this));
        NamedCommands.registerCommand("intake", new DeployIntake(s_Elevator, s_Pivot, s_Intake, RobotContainer.this));
        NamedCommands.registerCommand("exitHome", new PowerExitHome(s_Elevator, s_Pivot));
        NamedCommands.registerCommand("home", new GoHome(s_Elevator, s_Pivot));
        NamedCommands.registerCommand("stopIntakeShooter", 
            new ParallelCommandGroup(
                new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake)//,
                //new InstantCommand(() -> s_Shooter.setShooterPercent(0.0), s_Shooter)
            )
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        Shuffleboard.getTab("Competition").add(autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /*Controller Mappings*/
        
        //A Button
        OIConstants.ampTarget.onTrue(new InstantCommand(()-> m_TargetMode = targetMode.kAmp));

        //Y Button
        OIConstants.speakerTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kSpeaker));

        //B Button
        OIConstants.trapTarget.onTrue(new InstantCommand(() -> m_TargetMode = targetMode.kTrap));

        //X Button
        OIConstants.reverseIntake.whileTrue(new RunCommand(() -> s_Intake.setSpeed(-0.25), s_Intake)).
            onFalse(new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake));

        //Left Bumper
        OIConstants.manualActive.whileTrue(
            new ParallelCommandGroup(
                s_Pivot.manualControl(() -> -OIConstants.pivotSpeed.getAsDouble() * 0.2),
                s_Elevator.manualControl(() -> -OIConstants.elevatorSpeed.getAsDouble() * 0.4)
            )
        );

        //Climber Deploy
        OIConstants.climberDeploy.whileTrue(
            new ClimbDeploy(s_Pivot, s_Elevator)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Elevator.setHeight(s_Elevator.getHeight()), s_Elevator),
                new InstantCommand(() -> s_Pivot.setPivotAngle(s_Pivot.getCANcoder()), s_Pivot),
                new InstantCommand(() -> s_Elevator.retractActuator(), s_Elevator)
            )
        );

        //Climate Retract
        OIConstants.climberRetract.whileTrue(
            new ClimbRetract(s_Elevator)
        ).onFalse(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Elevator.setHeight(s_Elevator.getHeight()), s_Elevator),
                new InstantCommand(() -> s_Pivot.setPivotAngle(s_Pivot.getCANcoder()), s_Pivot),
                new InstantCommand(() -> s_Elevator.retractActuator(), s_Elevator)
            )
        );

        /*Driver Mappings */

        //Left Stick Center Button
        OIConstants.homeButton.whileTrue(
            new GoHome(s_Elevator, s_Pivot)
        );

        //Drive Left Trigger
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

        OIConstants.deployIntake.onFalse(
            new InstantCommand(() -> setIntakeCentric(false)) 
        );

        //Drive Right Trigger
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

        //Drive Right Stick Center Button
        OIConstants.resetGyro.onTrue(
            new InstantCommand(() -> s_Swerve.zeroHeading(), s_Swerve)
        );

        OIConstants.snapBack.whileTrue(new RunCommand(() -> s_Elevator.retractActuator(), s_Elevator));

        OIConstants.snapFront.whileTrue(new RunCommand(() -> s_Elevator.deployActuator(), s_Elevator));
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

    public void setIntakeCentric(boolean value) {
        intakeCentric = value;
    }

    public SequentialCommandGroup reset(){
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Elevator.retractActuator(), s_Elevator),
            new InstantCommand(() -> s_Pivot.setPivotAngle(s_Pivot.getCANcoder()), s_Pivot),
            new InstantCommand(() -> s_Elevator.setHeight(s_Elevator.getHeight()), s_Elevator)
        );
    }
}