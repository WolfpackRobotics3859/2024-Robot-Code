package frc.robot.constants;

public class Positions 
{
    public static final class STOW
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 4.3;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 0;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 0;
        public static final double SHOOTER_FEEDER_VOLTAGE = 0;
        public static final double SHOOTER_WRIST_ANGLE = 0.5;
        public static final double ELEVATOR_POSITION = 0.0725;
    }

    public static final class INTAKING
    {
        public static final double INTAKE_ROLLER_VOLTAGE = -6;
        public static final double INTAKE_WRIST_POSITION = 0.23;
        public static final double SHOOTER_ROLLER_1_VELOCITY = -30;
        public static final double SHOOTER_ROLLER_2_VELOCITY = -30;
        public static final double SHOOTER_FEEDER_VOLTAGE = -3;
        public static final double SHOOTER_WRIST_ANGLE = 0.53;
        public static final double ELEVATOR_POSITION = 0.0725;
    }

    public static final class AMP
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 3.0;
        public static final double SHOOTER_ROLLER_1_VELOCITY = -35;
        public static final double SHOOTER_ROLLER_2_VELOCITY = -35;
        public static final double SHOOTER_FEEDER_VOLTAGE = -11;
        public static final double SHOOTER_WRIST_ANGLE = 0.53;
        public static final double ELEVATOR_POSITION = 0.805;
    }

    public static final class LOW_SHOT
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 2.7;
        public static final double SHOOTER_FEEDER_VOLTAGE = 10;
        public static final double ELEVATOR_POSITION = 0.2;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 70;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 70;
    }

    public static final class LOW_BUMPER_SHOT
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 2.7;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 40;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 40;
        public static final double SHOOTER_FEEDER_VOLTAGE = 10;
        public static final double SHOOTER_WRIST_ANGLE = 0.665;
        public static final double ELEVATOR_POSITION = 0.2;
    }

    public static final class DEFENSE_SHOT
    {
        public static final double INTAKE_ROLLER_VOLTAGE = 0;
        public static final double INTAKE_WRIST_POSITION = 3.0;
        public static final double SHOOTER_ROLLER_1_VELOCITY = 30;
        public static final double SHOOTER_ROLLER_2_VELOCITY = 30;
        public static final double SHOOTER_FEEDER_VOLTAGE = 10;
        public static final double SHOOTER_WRIST_ANGLE = 0.57;
        public static final double ELEVATOR_POSITION = 0.8; 
    }
}
