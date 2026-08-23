package frc.robot.subsystems.Shooter;

public final class ShooterConstants {
    public static class FlyWheelConstants {
        public static final int leftFlyWheelMotorID = 16;
        public static final int rightFlyWheelMotorID = 17;

        public static final Double P = 0.000;
        public static final Double I = 0.0;
        public static final Double D = 0.0;

        public static final Double kS = 0.0;
        public static final Double kV = 0.004025;

        public static final double flyWheelSpeed = 500;

    }

    public static class IndexerConstants {
        public static final int indexerMoterID = 18;
        public static final int kickupMoterID = 19;

        public static final Double indexerMoterSpeed = 0.75;
        public static final Double kickupMoterSpeed = 0.5;

    }
}
