package frc.lib.BobcatLib.CANdle;

import frc.lib.BobcatLib.Annotations.SeasonSpecific;



/**
 * add season-specific states here
 */
@SeasonSpecific
public enum CANdleState {
    OFF,
    INTAKESTALL,
    RESETPOSE,
    RESETGYRO,
    OUTAKE,
    ALIGNING,
    ALIGNED
    
}
