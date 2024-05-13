package frc.robot.Commands.Swerve;

import frc.lib.BobcatLib.Swerve.Swerve;
import frc.robot.Constants.SwerveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve swerve;
    private DoubleSupplier translation;
    private DoubleSupplier fineStrafe;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private BooleanSupplier robotCentric;
    private DoubleSupplier fineTrans;
    private BooleanSupplier autoAlignSupplier;
    private BooleanSupplier aimAssistSupplier;
    private PIDController aimAssistXController = new PIDController(0.2, 0, 0); // TODO tune
    private PIDController aimAssistYController = new PIDController(0.2, 0, 0);
    private Translation2d currTranslation = new Translation2d();

    /**
     * 
     * suppliers are objects that can return a value that will change, for example a
     * method that returns a double can be input as a doubleSupplier
     * 
     * @param swerve        the swerve subsystem
     * @param translation   the value to drive the robot forward and backward
     * @param strafe        value to drive the robot left and right
     * @param rotation      value to rotate the drivetrain
     * @param robotCentric  field-centric if false
     * @param fineStrafe    slow speed control, cancled if translation or strafe is
     *                      in use
     * @param fineTrans     slow speed control, cancled if translation or strafe is
     *                      in use
     * @param snapToAmp     should we automatically rotate to the amp
     * @param snapToSpeaker should we automatically align to the speaker
     * @param pass          align to be facing the amp for passing notes
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe,
            DoubleSupplier rotation, BooleanSupplier robotCentric, DoubleSupplier fineStrafe, DoubleSupplier fineTrans,
            BooleanSupplier aimAssistSupplier, BooleanSupplier autoAlignSupplier) {

        this.swerve = swerve;
        addRequirements(swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.robotCentric = robotCentric;
        this.fineStrafe = fineStrafe;
        this.fineTrans = fineTrans;
        this.aimAssistSupplier = aimAssistSupplier;
        this.autoAlignSupplier = autoAlignSupplier;
        }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConstants.stickDeadband); // from 0 to one

        /*
         * If joysticks not receiving any normal input, use twist values for fine adjust
         */
        if (strafeVal == 0.0) {
            strafeVal = fineStrafe.getAsDouble();
        }
        if (translationVal == 0.0) {
            translationVal = fineTrans.getAsDouble();
        }

        swerve.setAimAssistTranslation(new Translation2d());
        swerve.setAutoAlignAngle(new Rotation2d());

        if (aimAssistSupplier.getAsBoolean()) {
            currTranslation = swerve.getPose().getTranslation();
            translationVal += aimAssistXController.calculate(currTranslation.getX());
            strafeVal += aimAssistYController.calculate(currTranslation.getY());
        }

        /* Drive */
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed),
                rotationVal * SwerveConstants.maxAngularVelocity,
                !robotCentric.getAsBoolean(),
                autoAlignSupplier.getAsBoolean());

    }
}