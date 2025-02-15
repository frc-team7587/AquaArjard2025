package frc.robot.subsystems.coralOutake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralOutakeSparkMax implements CoralOutakeIO {
    SparkMax coralIndexMotor;
    SparkMax coralPivotMotor;

    public CoralOutakeSparkMax() {
        coralIndexMotor = new SparkMax(CoralOutakeConstants.kCoralIndexerMotorID, MotorType.kBrushless);
        coralPivotMotor = new SparkMax(CoralOutakeConstants.kCoralPivotMotorID, MotorType.kBrushless);
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
}
