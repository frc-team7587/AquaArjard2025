package frc.robot.subsystems.coralIntake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralIntakeSparkMax implements CoralIntakeIO {
    SparkMax coralIndexMotor;
    SparkMax coralPivotMotor;

    public CoralIntakeSparkMax() {
        coralIndexMotor = new SparkMax(CoralIntakeConstants.kCoralIndexerMotorID, MotorType.kBrushless);
        coralPivotMotor = new SparkMax(CoralIntakeConstants.kCoralPivotMotorID, MotorType.kBrushless);
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
