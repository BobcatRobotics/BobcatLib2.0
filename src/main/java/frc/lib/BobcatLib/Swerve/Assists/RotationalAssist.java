package frc.lib.BobcatLib.Swerve.Assists;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;
import lombok.Setter;

public class RotationalAssist {
    @Setter @Getter private Rotation2d desiredRotation;
    private BooleanSupplier shouldRun;
    private Supplier<Rotation2d> currRot;


    /**
     * 
     * @param pose the pose to guide the drivetrain to
     */
    public RotationalAssist(Rotation2d assistRotationSupplier, Supplier<Rotation2d> rotationSupplier, BooleanSupplier shouldAssist){
        desiredRotation = assistRotationSupplier;
        currRot = rotationSupplier;
        shouldRun = shouldAssist;
    }

    public boolean shouldAssist(){
        return shouldRun.getAsBoolean();
    }
    public Rotation2d getDistanceToTarget(){
        return getDesiredRotation().minus(currRot.get());
    }
    public double getErrorRad(){
        return getDistanceToTarget().getRadians();
    }


}
