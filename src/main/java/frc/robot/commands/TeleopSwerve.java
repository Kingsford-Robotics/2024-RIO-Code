package frc.robot.commands;

import frc.robot.Constants.Constants;
import frc.robot.Constants.OIConstants;
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
    private BooleanSupplier slowModeSup;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter rotationLimiter;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowModeSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowModeSup = slowModeSup;

        //Instantiates slew rate limiters using translation and turn ramp times.
        translationLimiter = new SlewRateLimiter(1/OIConstants.translateRampTime);
        rotationLimiter = new SlewRateLimiter(1/OIConstants.turnRampTime);
    }

    @Override
    public void execute() {
        /* Slew rate limits the inputs and squares them to make the controls more sensitive at lower speeds. */
        double translationVal = translationLimiter.calculate(Math.copySign(Math.pow(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) ,2), MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband)));
        double strafeVal = translationLimiter.calculate(Math.copySign(Math.pow(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) ,2), MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband)));
        double rotationVal = rotationLimiter.calculate(Math.copySign(Math.pow(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) ,2), MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband)));

        /* Slow mode */
        if(slowModeSup.getAsBoolean()) {
            translationVal *= Constants.Swerve.slowModeMultiplier;
            strafeVal *= Constants.Swerve.slowModeMultiplier;
            rotationVal *= Constants.Swerve.slowModeMultiplier;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}