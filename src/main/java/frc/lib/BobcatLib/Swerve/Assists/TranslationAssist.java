package frc.lib.BobcatLib.Swerve.Assists;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.BobcatLib.Swerve.SwerveConstants.AimAssistConstants;

public class TranslationAssist {
    private Supplier<Translation2d> translationalSupplier;
    private BooleanSupplier shouldRun;
    private Supplier<Translation2d> currPose;
    private BooleanSupplier override;
    

    /**
     * 
     * @param pose the pose to guide the drivetrain to
     */
    public TranslationAssist(Supplier<Translation2d> assistPoseSupplier, Supplier<Translation2d> poseSupplier, BooleanSupplier shouldAssist, BooleanSupplier override){
        translationalSupplier = assistPoseSupplier;
        currPose = poseSupplier;
        shouldRun = shouldAssist;
        this.override = override;
    }

    public boolean shouldAssist(){
        return shouldRun.getAsBoolean() && !override.getAsBoolean();
    }
    public Translation2d getDesiredPose(){
        return translationalSupplier.get();
    }
    public Translation2d getDistanceToTarget(){
        return getDesiredPose().minus(currPose.get());
    }
    public double xError(){
        return getDistanceToTarget().getX() * AimAssistConstants.kP_X;
    }
    public double yError(){
        return getDistanceToTarget().getY() * AimAssistConstants.kP_Theta;
    }



}
