// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Central robot state: shoot mode, climb mode, and selected scoring target.
 * Subsystems and commands read this to coordinate behavior.
 */
public final class RobotState {

    private RobotState() {}

    public enum ShootMode {
        /** Not shooting; index/tower can be off or run for intake. */
        IDLE,
        /** Preparing: turret/flywheel/hood moving to setpoints. */
        PREPARING,
        /** At setpoint; feeding ball (index/tower on). */
        SHOOTING
    }

    public enum ClimbMode {
        /** Normal teleop; climber disabled. */
        DISABLED,
        /** Climber enabled; operator can run climb/retract. */
        ENABLED
    }

    public enum VisionCollectionMode {
        COLLECT,
        IGNORE
    }

    public static VisionCollectionMode s_ShootAuth = VisionCollectionMode.IGNORE;

    /** When true, TurretAim acts as if the aim button is pressed (used by auto commands). */
    public static volatile boolean s_autoAimActive = false;

    private static ShootMode s_shootMode = ShootMode.IDLE;
    private static ClimbMode s_climbMode = ClimbMode.DISABLED;
    /** Name of selected target (e.g. "HUB_BLUE", "HUB_RED"). Used by turret and shot calc. */
    private static volatile String s_selectedTargetName = "HUB_BLUE";

    public static ShootMode getShootMode() {
        return s_shootMode;
    }

    public static void setShootMode(ShootMode mode) {
        s_shootMode = mode;
    }

    public static ClimbMode getClimbMode() {
        return s_climbMode;
    }

    public static void setClimbMode(ClimbMode mode) {
        s_climbMode = mode;
    }

    public static String getSelectedTargetName() {
        return s_selectedTargetName;
    }

    public static void setSelectedTargetName(String name) {
        s_selectedTargetName = name != null ? name : "HUB_BLUE";
    }

    public static boolean isShootModeActive() {
        return s_shootMode == ShootMode.PREPARING || s_shootMode == ShootMode.SHOOTING;
    }

    /** True only when operator/driver is holding shoot trigger (feeding). Use for drive speed limit. */
    public static boolean isShooting() {
        return s_shootMode == ShootMode.SHOOTING;
    }

    public static boolean isClimbModeActive() {
        return s_climbMode == ClimbMode.ENABLED;
    }
}
