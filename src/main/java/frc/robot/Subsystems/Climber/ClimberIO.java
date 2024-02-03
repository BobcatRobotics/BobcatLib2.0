package frc.robot.Subsystems.Climber;
import org.littletonrobotics.junction.AutoLog;



public interface ClimberIO {
    @AutoLog 
    public static class ClimberIOInputs {

        public double climberPosition = 0.0;
        public double climberMotorPercentOut = 0.0;
        public double climberMotorStatorCurrent = 0.0;
        public double climberMotorVelocityRPS = 0.0;
        public double climberMotorPosition = 0.0;

        public double motionmagicAcceleration = 0.0;
        public double motionmagicCruiseVelocity = 0.0;
        public double motionmagicJerk = 0.0;

        }


    public default double getClimberPosition(int climberEncoderValue){
        return 0.0;
    }

    public default void updateInputs(){
        updateInputs();
    }

    public default void stop(){
        stop();
    }

    public default void run(double rotationAmount){
        run(rotationAmount);
    }

    public default void inverseDirection(){ 
        inverseDirection();
    }

}