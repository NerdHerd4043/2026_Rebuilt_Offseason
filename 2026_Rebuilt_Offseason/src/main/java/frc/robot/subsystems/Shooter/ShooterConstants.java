package frc.robot.subsystems.Shooter;

public final class ShooterConstants {
    public static class FlyWheelConstants {
        public static final int leftFlyWheelMotorID = 16;
        public static final int rightFlyWheelMotorID = 17;

        public static final int P = 0;
        public static final int I = 0;
        public static final int D = 0;

        public static final int kS = 0;
        public static final int kV = 0;

        public static final double flyWheelSpeed = 2000.0; // In RPM

    }

    public static class IndexerConstants {
        public static final int indexerMoterID = 18;

        public static final Double indexerMoterSpeed = 0.2;

    }
}
