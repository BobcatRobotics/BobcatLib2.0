package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;


public class Amp extends SubsystemBase {
    private final AmpIO io;
    private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

    public Amp(AmpIO io){
        this.io = io;
    }
    public void period(){
        io.updateInputs(inputs);
        Logger.processInputs("Amp", inputs);
    }
    

    public void setPos(double degrees){
        io.setPos(degrees/360); // degrees -> rotationsss
    }

    public void deploy(){
        setPos(AmpConstants.deployValue);
    }
    public void retract(){
        setPos(AmpConstants.retractValue);
    }

    public boolean beyondCrashThreshold(){
        return inputs.motorposition > AmpConstants.crashThreshold; //TODO not sure if this should be > or <
    }

    public void setPercentOut(double percent){
        io.setPercent(percent);
    }

    public void stop(){
        io.stop();
    }
} 