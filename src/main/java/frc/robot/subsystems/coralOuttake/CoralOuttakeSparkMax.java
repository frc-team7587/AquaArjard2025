package frc.robot.subsystems.coralOuttake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Configs.AlgaeIntakeConfig;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;

public class CoralOuttakeSparkMax implements CoralOuttakeIO {
    SparkMax coralIndexMotor;
    SparkMax coralPivotMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    private final DigitalInput breakBeamSensor;

    public CoralOuttakeSparkMax() {
        coralPivotMotor = new SparkMax(AlgaeIntakeConstants.kAlgaeMotorLeaderID, MotorType.kBrushless);
        coralIndexMotor = new SparkMax(AlgaeIntakeConstants.kAlgaeMotorFollowerID, MotorType.kBrushless);
    
        pivotEncoder = coralPivotMotor.getEncoder();
        pivotController = coralIndexMotor.getClosedLoopController();
        coralPivotMotor.configure(AlgaeIntakeConfig.pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        coralIndexMotor.configure(AlgaeIntakeConfig.indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    public void setPivotPosition(double position) {
        pivotController.setReference(position, ControlType.kPosition);
    }

    @Override
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }
    
    @Override
    public boolean getBreakBeamSensorValue() {
        return !breakBeamSensor.get();
    }

    @Override
    public void reset() {
        pivotEncoder.setPosition(0);
    }

    @Override
    public void stop() {
        coralIndexMotor.set(0);
    }
}
