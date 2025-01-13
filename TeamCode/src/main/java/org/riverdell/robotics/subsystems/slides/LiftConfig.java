package org.riverdell.robotics.subsystems.slides;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LiftConfig {
    public static int MAX_EXTENSION = 2450;
    public static double kP = 0.012;
    public static double kI = 0.0;
    public static double kD = 0.0005;
}
