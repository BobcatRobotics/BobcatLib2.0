// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;


import BobcatLib.Team177.Swerve.SwerveBase;
import BobcatLib.Team177.Swerve.Constants.SwerveConstants;
import BobcatLib.Team177.Vision.Vision;
import edu.wpi.first.math.Matrix;

/** Add your docs here. */
public class Swerve extends SwerveBase{
    
    public Swerve(SwerveConstants constants, int[] filterTags, Matrix[] visionStdDevs, Vision... cameras){
        super(constants, filterTags, visionStdDevs, cameras);
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
