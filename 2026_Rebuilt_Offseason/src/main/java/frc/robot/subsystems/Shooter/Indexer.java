package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.Shooter.ShooterConstants.IndexerConstants;

public class Indexer extends SubsystemBase {
    private SparkMax indexerMoter = new SparkMax(IndexerConstants.indexerMoterID, MotorType.kBrushless);
    private SparkMax kickupMotor = new SparkMax(IndexerConstants.kickupMoterID, MotorType.kBrushless);

    public Indexer() {
        final SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
        final SparkMaxConfig kickupMotorConfig = new SparkMaxConfig();

        indexerMotorConfig.idleMode(IdleMode.kBrake);
        kickupMotorConfig.idleMode(IdleMode.kBrake);

        indexerMoter.configure(indexerMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        kickupMotor.configure(kickupMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command indexCommand() {
        return this.run(() -> {
            indexerMoter.set(IndexerConstants.indexerMoterSpeed);
        }).finallyDo(() -> {
            indexerMoter.stopMotor();
        });
    }

    public Command kickupCommand() {
        return this.run(() -> {
            kickupMotor.set(IndexerConstants.kickupMoterSpeed);
        }).finallyDo(() -> {
            kickupMotor.stopMotor();
        });
    }
}
