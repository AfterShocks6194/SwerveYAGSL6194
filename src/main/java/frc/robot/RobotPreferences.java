package frc.robot;

import com.frcteam3255.preferences.SN_BooleanPreference;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.preferences.SN_ZeroDoublePreference;

import edu.wpi.first.math.util.Units;

public class RobotPreferences {

    public static final boolean useNetworkTables = true;

    // order of subsystems (and adjacent classes) shall be:
    // controllers, drivetrain, arm, intake, collector, charger (if it exists),
    // vision, leds

    public static final class prefDrivetrain {

     
     
    }

    public static final class prefArm {
        public static final SN_DoublePreference shoulderP = new SN_DoublePreference("shoulderP", 0.1);
        public static final SN_DoublePreference shoulderI = new SN_DoublePreference("shoulderI", 0);
        public static final SN_DoublePreference shoulderD = new SN_DoublePreference("shoulderD", 1.5);
        // falcon encoder counts per second
        public static final SN_DoublePreference shoulderMaxSpeed = new SN_DoublePreference("shoulderMaxSpeed", 18000);
        // falcon encoder counts per second per second
        public static final SN_DoublePreference shoulderMaxAccel = new SN_DoublePreference("shoulderMaxAccel", 10000);
        // degrees
        public static final SN_DoublePreference shoulderTolerance = new SN_DoublePreference("shoulderTolerance", 0.5);
        public static final SN_DoublePreference shoulderClosedLoopPeakOutput = new SN_DoublePreference(
                "shoulderClosedLoopPeakOutput", 0.6);

        public static final SN_DoublePreference elbowP = new SN_DoublePreference("elbowP", 0.1);
        public static final SN_DoublePreference elbowI = new SN_DoublePreference("elbowI", 0);
        public static final SN_DoublePreference elbowD = new SN_DoublePreference("elbowD", 1.5);
        // falcon encoder counts per second
        public static final SN_DoublePreference elbowMaxSpeed = new SN_DoublePreference("elbowMaxSpeed", 18000);
        // falcon encoder counts per second per second
        public static final SN_DoublePreference elbowMaxAccel = new SN_DoublePreference("elbowMaxAccel", 10000);
        // degrees
        public static final SN_DoublePreference elbowTolerance = new SN_DoublePreference("elbowTolerance", 0.5);
        public static final SN_DoublePreference elbowClosedLoopPeakOutput = new SN_DoublePreference(
                "elbowClosedLoopPeakOutput", 0.6);

        public static final SN_DoublePreference shoulderAdjustRange = new SN_DoublePreference("shoulderAdjustRange", 0);
        public static final SN_DoublePreference elbowAdjustRange = new SN_DoublePreference("elbowAdjustRange", 0);

        // preset to intake cube from collector
        public static final SN_DoublePreference armPresetCollectorShoulderAngle = new SN_DoublePreference(
                "armPresetCollectorShoulderAngle", 25);
        public static final SN_DoublePreference armPresetCollectorElbowAngle = new SN_DoublePreference(
                "armPresetCollectorElbowAngle", -92.5);

        // preset to stow arm within frame
        public static final SN_DoublePreference armPresetStowShoulderAngle = new SN_DoublePreference(
                "armPresetStowShoulderAngle", -90);
        public static final SN_DoublePreference armPresetStowElbowAngle = new SN_DoublePreference(
                "armPresetStowElbowAngle", 66);

        // preset to score cone AND cube in hybrid node
        public static final SN_DoublePreference armPresetLowShoulderAngle = new SN_DoublePreference(
                "armPresetLowShoulderAngle", -90);
        public static final SN_DoublePreference armPresetLowElbowAngle = new SN_DoublePreference(
                "armPresetLowElbowAngle", 0);

        // preset to intake cone from ground
        public static final SN_DoublePreference armPresetConeShoulderAngle = new SN_DoublePreference(
                "armPresetConeShoulderAngle", -90);
        public static final SN_DoublePreference armPresetConeElbowAngle = new SN_DoublePreference(
                "armPresetConeElbowAngle", -15);

        // preset to score cone on mid node
        public static final SN_DoublePreference armPresetConeMidShoulderAngle = new SN_DoublePreference(
                "armPresetMidShoulderAngle", -59);
        public static final SN_DoublePreference armPresetConeMidElbowAngle = new SN_DoublePreference(
                "armPresetMidElbowAngle", 40);

        // preset to score cone on high node
        public static final SN_DoublePreference armPresetConeHighShoulderAngle = new SN_DoublePreference(
                "armPresetHighShoulderAngle", -11);
        public static final SN_DoublePreference armPresetConeHighElbowAngle = new SN_DoublePreference(
                "armPresetHighElbowAngle", 25);

        // preset to score cube on mid node
        public static final SN_DoublePreference armPresetCubeMidShoulderAngle = new SN_DoublePreference(
                "armPresetCubeMidShoulderAngle", -95.5);
        public static final SN_DoublePreference armPresetCubeMidElbowAngle = new SN_DoublePreference(
                "armPresetCubeMidElbowAngle", 22);

        // preset to score cube on high node
        public static final SN_DoublePreference armPresetCubeHighShoulderAngle = new SN_DoublePreference(
                "armPresetCubeHighShoulderAngle", 37.5);
        public static final SN_DoublePreference armPresetCubeHighElbowAngle = new SN_DoublePreference(
                "armPresetCubeHighElbowAngle", -30.5);

        // preset to collect cone (or cube) from shelf
        public static final SN_DoublePreference armPresetShoulderShelf = new SN_DoublePreference(
                "armPresetShoulderShelf", -20);
        public static final SN_DoublePreference armPresetElbowShelf = new SN_DoublePreference(
                "armPresetElbowShelf", 10);

        // amount to lower joints by when scoring
        public static final SN_DoublePreference armShoulderLoweringAngle = new SN_DoublePreference(
                "armShoulderLoweringAngle", 0);
        public static final SN_DoublePreference armElbowLoweringAngle = new SN_DoublePreference(
                "armElbowLoweringAngle", 20);

        // position shoulder goes to when transitioning from collector preset
        public static final SN_DoublePreference armPresetPostCollectorShoulderAngle = new SN_DoublePreference(
                "armPresetPostCollectorShoulderAngle", 45);

        // preset to stick arm straight out
        public static final SN_ZeroDoublePreference armPresetStraightShoulderAngle = new SN_ZeroDoublePreference();
        public static final SN_ZeroDoublePreference armPresetStraightElbowAngle = new SN_ZeroDoublePreference();

        public static final SN_DoublePreference armShootCubeHighShoulderAngle = new SN_DoublePreference(
                "armShootCubeHighShoulderAngle", -99);
        public static final SN_DoublePreference armShootCubeHighElbowAngle = new SN_DoublePreference(
                "armShootCubeHighElbowAngle", 32);

        public static final SN_DoublePreference armToleranceFudgeFactor = new SN_DoublePreference(
                "armToleranceFudgeFactor", 5);
    }


   


   
}
