package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralSparkMax implements CoralIO {
    SparkMax coralIndexMotor;
    SparkMax coralPivotMotor;

    public CoralSparkMax() {
        coralIndexMotor = new SparkMax(CoralConstants.kCoralIndexerMotorID, MotorType.kBrushless);
        coralPivotMotor = new SparkMax(CoralConstants.kCoralPivotMotorID, MotorType.kBrushless);
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
