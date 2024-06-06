package frc.lib.BobcatLib.Swerve.Assists;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import lombok.Getter;
import lombok.Setter;

public class TranslationalAssist {
    @Setter @Getter private Translation2d desiredTranslation;
    private BooleanSupplier shouldRun;
    private Supplier<Translation2d> currTrans;


    /**
     * 
     * @param pose the pose to guide the drivetrain to
     */
    public TranslationalAssist(Translation2d assistTrans, Supplier<Translation2d> translationSupplier, BooleanSupplier shouldAssist){
        desiredTranslation = assistTrans;
        currTrans = translationSupplier;
        shouldRun = shouldAssist;
    }

    public boolean shouldAssist(){
        return shouldRun.getAsBoolean();
    }
    public Translation2d getDistanceToTarget(){
        return getDesiredTranslation().minus(currTrans.get());
    }
    public double xError(){
        return getDistanceToTarget().getX() * Constants.AimAssistConstants.kP_X;
    }
    public double yError(){
        return getDistanceToTarget().getY() * Constants.AimAssistConstants.kP_Y;
    }

}
