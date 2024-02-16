package frc.robot;

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
    //private final Intake s_Intake = new Intake();
    private final Jetson s_Jetson = new Jetson();
    private final LedDriver s_LedDriver = new LedDriver();
    private final Limelight s_Limelight = new Limelight();
    private final Pivot s_Pivot = new Pivot();
    //private final Shooter s_Shooter = new Shooter();
    private final Swerve s_Swerve = new Swerve();

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
            new InstantCommand(() -> s_Pivot.setPivotSpeed(-OIConstants.pivotSpeed.getAsDouble() * 0.1), s_Pivot)
        );

        s_Elevator.setDefaultCommand(
            new InstantCommand(() -> s_Elevator.setSpeed(-OIConstants.elevatorSpeed.getAsDouble() * 0.2), s_Elevator)
        );

        //s_Shooter.setDefaultCommand(
        //    new InstantCommand(() -> s_Shooter.setShooterPercent(OIConstants.shooterSpeed.getAsDouble() * 0.2), s_Shooter)
        //);

        //s_Intake.setDefaultCommand(
        //    new InstantCommand(() -> s_Intake.setSpeed(OIConstants.intakeSpeed.getAsDouble()), s_Intake)
        //);

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

        /* Co-Driver Buttons */

        //Reverse Intake
        //OIConstants.reverseIntake.onTrue(new InstantCommand(() -> s_Intake.setSpeed(-0.5), s_Intake)).
        //    onFalse(new InstantCommand(() -> s_Intake.setSpeed(0.0), s_Intake));

        OIConstants.climbDeploy.onTrue(s_Elevator.setHeight(Units.inchesToMeters(6), s_Elevator)).
            onFalse(new InstantCommand(() -> s_Elevator.stop(), s_Elevator)); 
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
