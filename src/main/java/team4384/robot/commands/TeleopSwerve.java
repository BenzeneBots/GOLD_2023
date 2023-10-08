package team4384.robot.commands;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import team4384.robot.constants.RobotMap;
import team4384.robot.constants.SwerveMap;
import team4384.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private JoystickButton Rotate;
    private JoystickButton SlowMode;
    private JoystickButton resetGyro;
    private DoubleSupplier Inverted;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, DoubleSupplier Inverted, BooleanSupplier robotCentricSup, JoystickButton Rotate, JoystickButton SlowMode, JoystickButton resetGyro) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.Rotate = Rotate;
        this.SlowMode = SlowMode;
        this.resetGyro = resetGyro;
        this.Inverted = Inverted;
    }

    public void manuel_execute(double translationVal, double strafeVal, double rotationVal) {
        if (SlowMode.getAsBoolean()) {
            translationVal = -MathUtil.applyDeadband(MathUtil.clamp(translationSup.getAsDouble(), -SwerveMap.maxSlowTeleSpeed, SwerveMap.maxSlowTeleSpeed), RobotMap.stickDeadband);
            strafeVal = -MathUtil.applyDeadband(MathUtil.clamp(strafeSup.getAsDouble(), -SwerveMap.maxSlowTeleSpeed, SwerveMap.maxSlowTeleSpeed), RobotMap.stickDeadband);
            rotationVal = MathUtil.applyDeadband(MathUtil.clamp(rotationSup.getAsDouble(), -SwerveMap.maxRotSpeed , SwerveMap.maxRotSpeed), RobotMap.stickDeadband);

        }
        else  {
            translationVal = MathUtil.applyDeadband(MathUtil.clamp(translationSup.getAsDouble(), -SwerveMap.maxTeleSpeed, SwerveMap.maxTeleSpeed), RobotMap.stickDeadband) * -1;
            strafeVal = MathUtil.applyDeadband(MathUtil.clamp(strafeSup.getAsDouble(), -SwerveMap.maxTeleSpeed, SwerveMap.maxTeleSpeed), RobotMap.stickDeadband) * -1;
            rotationVal = MathUtil.applyDeadband(MathUtil.clamp(rotationSup.getAsDouble(), -SwerveMap.maxRotSpeed, SwerveMap.maxRotSpeed), RobotMap.stickDeadband);
        }

        if (!Rotate.getAsBoolean()) {
            rotationVal = 0;
        }
        else {
            translationVal = 0;
            strafeVal = 0;
        }

        double Invert = Inverted.getAsDouble();

        if (Invert == 1 ||Invert == -1) {
            translationVal *= Invert;
            strafeVal *= Invert;
        }

        if (resetGyro.getAsBoolean()) {
            s_Swerve.zeroGyro();
        }

        strafeVal = 0;
        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveMap.maxSpeed),
                rotationVal * SwerveMap.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
        );
    }
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

        if (SlowMode.getAsBoolean()) {
            translationVal = -MathUtil.applyDeadband(MathUtil.clamp(translationSup.getAsDouble(), -SwerveMap.maxSlowTeleSpeed, SwerveMap.maxSlowTeleSpeed), RobotMap.stickDeadband);
            strafeVal = -MathUtil.applyDeadband(MathUtil.clamp(strafeSup.getAsDouble(), -SwerveMap.maxSlowTeleSpeed, SwerveMap.maxSlowTeleSpeed), RobotMap.stickDeadband);
            rotationVal = MathUtil.applyDeadband(MathUtil.clamp(rotationSup.getAsDouble(), -SwerveMap.maxRotSpeed , SwerveMap.maxRotSpeed), RobotMap.stickDeadband);

        }
        else  {
            translationVal = MathUtil.applyDeadband(MathUtil.clamp(translationSup.getAsDouble(), -SwerveMap.maxTeleSpeed, SwerveMap.maxTeleSpeed), RobotMap.stickDeadband) * -1;
            strafeVal = MathUtil.applyDeadband(MathUtil.clamp(strafeSup.getAsDouble(), -SwerveMap.maxTeleSpeed, SwerveMap.maxTeleSpeed), RobotMap.stickDeadband) * -1;
            rotationVal = MathUtil.applyDeadband(MathUtil.clamp(rotationSup.getAsDouble(), -SwerveMap.maxRotSpeed, SwerveMap.maxRotSpeed), RobotMap.stickDeadband);
        }

        if (!Rotate.getAsBoolean()) {
            rotationVal = 0;
        }
        else {
            translationVal = 0;
            strafeVal = 0;
        }

        double Invert = Inverted.getAsDouble();

        if (Invert == 1 ||Invert == -1) {
            translationVal *= Invert;
            strafeVal *= Invert;
        }

        if (resetGyro.getAsBoolean()) {
            s_Swerve.zeroGyro();
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveMap.maxSpeed), 
            rotationVal * SwerveMap.maxAngularVelocity,
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}