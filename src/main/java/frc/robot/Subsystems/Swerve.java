// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.lib.BobcatLib.Swerve.GyroIO;
import frc.lib.BobcatLib.Swerve.SwerveBase;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIO;
import frc.lib.BobcatLib.Vision.Vision;

/** Add your docs here. */
public class Swerve extends SwerveBase{
    
    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO, Vision... cameras){
        super(gyroIO, flIO, frIO, blIO, brIO, cameras);
        
    }
}
