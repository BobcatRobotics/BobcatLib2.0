// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import lib.BobcatLib.Team177.Swerve.PhoenixOdometryThread;
import lib.BobcatLib.Team177.Swerve.SwerveBase;
import lib.BobcatLib.Team177.Swerve.Constants.SwerveConstants;
import lib.BobcatLib.Team177.Swerve.StandardDeviations.SwerveStdDevs;
import lib.BobcatLib.Team177.Vision.Vision;

/** Add your docs here. */
public class Swerve extends SwerveBase{
    
    public Swerve(SwerveConstants constants,
      int[] filterTags,
      SwerveStdDevs standardDeviations,
      PhoenixOdometryThread threadInstance,
      Vision... cameras){
        super(constants, filterTags, standardDeviations, threadInstance, cameras);
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
