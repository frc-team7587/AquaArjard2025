package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ClimberIO climberIO;

    public Climber(ClimberIO climberIO) {
        this.climberIO = climberIO;
    }
    
}
