package frc.robot.subsystems.RollerFloor;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerFloor extends SubsystemBase {
    private SparkMax rollerFloorMotor = new SparkMax(RollerFloorConstants.rollerFloorMotorID, MotorType.kBrushless);

    public RollerFloor() {
        final SparkMaxConfig rollerFloorMotorConfig = new SparkMaxConfig();

        rollerFloorMotorConfig.idleMode(IdleMode.kBrake);

        rollerFloorMotor.configure(rollerFloorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command feedCommand() {
        return this.run(() -> {
            rollerFloorMotor.set(RollerFloorConstants.rollerFloorSpeed);
        }).finallyDo(() -> {
            rollerFloorMotor.stopMotor();
        });
    }
}
