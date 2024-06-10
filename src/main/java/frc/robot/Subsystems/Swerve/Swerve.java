// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import frc.lib.BobcatLib.Swerve.GyroIO;
import frc.lib.BobcatLib.Swerve.SwerveBase;
import frc.lib.BobcatLib.Swerve.SwerveModule.SwerveModuleIO;
import frc.lib.BobcatLib.Vision.Vision;
import frc.robot.Constants;

/** Add your docs here. */
public class Swerve extends SwerveBase{
    
    public Swerve(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO, Vision... cameras){
        super(gyroIO, flIO, frIO, blIO, brIO, Constants.loopPeriodSecs, cameras);
    }
    

    /*  add season specific methods here
     *  for example, Devin's shoot-on-the-fly
     *  logic would go here, since it only applies
     *  to Crescendo. The autoalign and aim assist
     *  functionality, however, would go in the
     *  SwerveBase class, since you use it in 
     *  nearly every game.
     */
    
}
