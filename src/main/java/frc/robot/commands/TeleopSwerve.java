package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.OIConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter rotationLimiter;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        //Instantiate slew rate limiter using translation and turn ramp times.
        translationLimiter = new SlewRateLimiter(1/OIConstants.translateRampTime);
        rotationLimiter = new SlewRateLimiter(1/OIConstants.turnRampTime);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        double translationVal = MathUtil.applyDeadband(translationLimiter.calculate(translationSup.getAsDouble()), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(translationLimiter.calculate(strafeSup.getAsDouble()), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationLimiter.calculate(rotationSup.getAsDouble()), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}