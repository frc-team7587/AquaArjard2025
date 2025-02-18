package frc.robot.subsystems.coralIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

public class CoralIntakeSparkMax implements CoralIntakeIO {
    SparkMax coralIndexMotor;
    SparkMax coralPivotMotor;

    private final DigitalInput breakBeamSensor;

    public CoralIntakeSparkMax() {
        coralIndexMotor = new SparkMax(CoralIntakeConstants.kCoralIndexerMotorID, MotorType.kBrushless);
        coralPivotMotor = new SparkMax(CoralIntakeConstants.kCoralPivotMotorID, MotorType.kBrushless);

        breakBeamSensor = new DigitalInput(CoralIntakeConstants.kbreakLightSensorID);
    }

    @Override
    public void setIndexerSpeed(double speed) {
        coralIndexMotor.set(speed);
    }
    
    @Override
    public void setPivotSpeed(double speed) {
        coralPivotMotor.set(speed);
    }

    @Override
    public void stop() {
        coralIndexMotor.set(0);
    }

    @Override
    public boolean getBreakBeamSensorValue() {
        return !breakBeamSensor.get();
    }
}
