package frc.robot.Subsystems.Superstructure;

import frc.robot.Subsystems.Superstructure.Arm.Arm;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Superstructure {
    
    public enum IntakeState{
        IDLE,
        INTAKE,
        OUTTAKE,
        SHOOT
    }

    @RequiredArgsConstructor
    @Getter
    public enum SuperstructureGoal{
        HIGH(IntakeState.OUTTAKE),
        MID(IntakeState.OUTTAKE),
        LOW(IntakeState.OUTTAKE),
        BABY_BIRD(IntakeState.INTAKE),
        GROUND_INTAKE(IntakeState.INTAKE),
        STOW(IntakeState.IDLE),
        CUBE_SHOOT(IntakeState.SHOOT);

        private final IntakeState intake;
    }

    @Getter @Setter private SuperstructureGoal currentGoal = SuperstructureGoal.STOW; 
    private final Arm arm;

    public Superstructure(Arm arm){
        this.arm = arm;
    }

    public void setState(SuperstructureGoal goal){
        switch (goal) {
            case STOW:
                break;
        
            default:
                break;
        }
    }

    
}
