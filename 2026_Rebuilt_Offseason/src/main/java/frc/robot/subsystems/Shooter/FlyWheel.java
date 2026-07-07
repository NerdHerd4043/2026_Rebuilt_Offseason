package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.Shooter.ShooterConstants.FlyWheelConstants;

public class FlyWheel extends SubsystemBase {
    private SparkFlex leftFlyWheelMotor = new SparkFlex(FlyWheelConstants.leftFlyWheelMotorID, MotorType.kBrushless);
    private SparkFlex rightFlyWheelMotor = new SparkFlex(FlyWheelConstants.rightFlyWheelMotorID, MotorType.kBrushless);

    private SparkClosedLoopController pidController = rightFlyWheelMotor.getClosedLoopController();

    public FlyWheel() {
        final SparkFlexConfig leftFlyWheelMotorConfig = new SparkFlexConfig();
        final SparkFlexConfig rightFlyWheelMotorConfig = new SparkFlexConfig();

        // Assigning PID values
        rightFlyWheelMotorConfig.closedLoop
                .p(FlyWheelConstants.P)
                .i(FlyWheelConstants.I)
                .d(FlyWheelConstants.D);

        // Assigning feedforward values
        rightFlyWheelMotorConfig.closedLoop.feedForward
                .kS(FlyWheelConstants.kS)
                .kV(FlyWheelConstants.kV);

        rightFlyWheelMotorConfig.idleMode(IdleMode.kCoast);

        leftFlyWheelMotorConfig.follow(rightFlyWheelMotor, true);

        leftFlyWheelMotor.configure(leftFlyWheelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightFlyWheelMotor.configure(rightFlyWheelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // public Command runCommand() {
    // return this.run(() -> {
    // rightFlyWheelMotor.set(1);
    // }).finallyDo(() -> {
    // rightFlyWheelMotor.stopMotor();
    // });
    // }

    public Command runFlyWheel() {
        return this.run(() -> {
            pidController.setSetpoint(FlyWheelConstants.flyWheelSpeed, ControlType.kVelocity);
        }).finallyDo(() -> {
            pidController.setSetpoint(0, ControlType.kVoltage);
        });
    }
}
