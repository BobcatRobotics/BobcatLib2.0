package frc.lib.Team254;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final String moduleName;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public ModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, String moduleName) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.moduleName = moduleName;
    }
}
