package org.firstinspires.ftc.teamcode.mech;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class config { ;
    public static double kpTurretLL = 0.01; //.01 // proportional gain from the limelight (how strongly it reacts to the current error
    public static double kS = 0.05; //represents amt of voltage needed
    public static double kffGyro = .00475; // .003 // feedforward gain from the IMU
    public static double dbDeg = 3.5; // "deadband" is the range (degrees) in which no action occurs (prevents unnecessary power output & oscillation)
    public static long maxStaleMs = 50; //"staleness", how old the data is
}
