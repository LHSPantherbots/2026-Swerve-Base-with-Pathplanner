// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;



public final class Constants {
    
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class CanIdConstants{
    public static final int centerDriveCanId = 14;
    public static final int intakePivotCanId = 4;
    public static final int intakeRollerCanId = 5;
    public static final int hopperCanId = 8;
    public static final int feederCanId = 9;
    public static final int launcherLeftCanId = 7;
    public static final int launcherRightCanId = 6;
    public static final int hoodLeftCanId = 12;
    public static final int hoodRightCanId = 11;
    public static final int climberCanId = 13;
  }

  public static final class LEDs{
    public static final int purple_Red = 148;
    public static final int purple_Green = 0;
    public static final int purple_Blue = 211;
    public static final int purple_Hue = 282;
    public static final int purple_Sat = 100;
    public static final int purple_Val = 83;

    public static final int yellow_Red = 255;
    public static final int yellow_Green = 90;
    public static final int yellow_Blue = 0;
    public static final int yellow_Hue = 21;
    public static final int yellow_Sat = 100;
    public static final int yellow_Val = 100;

    public static final int orange_Red = 251;
    public static final int orange_Green = 51;
    public static final int orange_Blue = 0;
    public static final int orange_Hue = 12;
    public static final int orange_Sat = 100;
    public static final int orange_Val = 98;

    public static final int red_Hue = 0;
    public static final int red_Sat = 100;
    public static final int red_Val = 100;

    public static final int blue_Hue = 240;
    public static final int blue_Sat = 100;
    public static final int blue_Val = 100;

    public static final int green_Hue = 240;
    public static final int green_Sat = 100;
    public static final int green_Val = 100;

    public static final int Red = 255;
    public static final int Green = 255;
    public static final int Blue = 255;
    public static final int white_Hue = 0;
    public static final int white_Sat = 0;
    public static final int white_Val = 100;
  }

}
