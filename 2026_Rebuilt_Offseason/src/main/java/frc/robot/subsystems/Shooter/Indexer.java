package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer extends SubsystemBase {
    private SparkMax indexerMoter = new SparkMax(4, MotorType.kBrushless);

    public Indexer() {
        final SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();

        indexerMotorConfig.idleMode(IdleMode.kBrake);

        indexerMoter.configure(indexerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command indexCommand() {
        return this.run(() -> {
            indexerMoter.set(0.1);
        }).finallyDo(() -> {
            indexerMoter.stopMotor();
        });
    }
}
