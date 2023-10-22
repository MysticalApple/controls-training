// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class BallsSubsystem extends SubsystemBase {

  private final CANSparkMax flywheel;
  private final CANSparkMax flywheel2;
  private final WPI_TalonSRX belts;


  private final AnalogPotentiometer lower;
  private final AnalogPotentiometer center;
  private final AnalogPotentiometer upper;

  private final XboxController controller;

  private boolean started = false;
  private Timer debugTimer;

  private boolean storing = false;
  private Timer storingTimer;

  private boolean shooting = false;
  private Timer shootingTimer;

  private boolean[] ballStorage;

  /* Constants */
  /* Sensor values were obtained from testing and are mostly accurate, though
   * the lower sensor sometimes gives the same value when no ball is in front
   * of it and when a ball is too close to it.
   */
  private final double lowerSensorCutoff = 0.38;
  private final double centerSensorCutoff = 0.25;
  private final double upperSensorCutoff = 0.3;

  /* Speeds and timings need to be tested. They are currently set to what I
   * imagine might work.
   */
  private final double storingSpeed = -0.4;
  private final double flywheelShootingSpeed = 0.7;
  private final double beltsShootingSpeed = -0.1;

  private final double timeToNextSensor = 0.5;
  private final double timeToShoot = 0.5;
  

  /** Creates a new ExampleSubsystem. */
  public BallsSubsystem() {
    flywheel = new CANSparkMax(3, MotorType.kBrushless);
    flywheel2 = new CANSparkMax(16, MotorType.kBrushless);
    flywheel2.follow(flywheel); /* Both motors connect to the same flywheel. */
    belts = new WPI_TalonSRX(8);

    lower = new AnalogPotentiometer(0);
    center = new AnalogPotentiometer(1);
    upper = new AnalogPotentiometer(2);

    controller = new XboxController(0);

    debugTimer = new Timer();
    storingTimer = new Timer();
    shootingTimer = new Timer();
    
    ballStorage = new boolean[]{false, false, false};
  }


  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */
    if (!started) {
      started = true;
      debugTimer.start();
      return;
    }


    /* Update Ball Storage - If a sensor outputs above a certain value, there
     * is likely a ball in front of it. There exists a space between the lower
     * and center sensors in which a ball cannot be detected by either sensor,
     * but the correct value for timeToNextSensor should help avoid issues with
     * it.
     */
    ballStorage[0] = upper.get() > upperSensorCutoff;
    ballStorage[1] = center.get() > centerSensorCutoff;
    ballStorage[2] = lower.get() > lowerSensorCutoff;


    /* Storage Logic - Tries to advance a ball through the storage unless there
     * is already a ball directly in front of it. The timeToNextSensor is
     * currently the same for both intervals, and two diffferent times might be
     * more effective, but this is simple enough, and more testing is required
     * anyway.
     */
    if (!storing && ((!ballStorage[0] && ballStorage[1]) ||
                     (!ballStorage[1] && ballStorage[2]))) {
      storing = true;
      belts.set(storingSpeed);
      storingTimer.start();
    }

    if (storing && storingTimer.hasElapsed(timeToNextSensor)) {
      belts.set(0);
      storingTimer.stop();
      storingTimer.reset();
    }

    /* Shooting - Completely untested, but I think this system should work. The
     * conveyor belt being active is probably usually not required to shoot,
     * but it could be useful in case the ball at the top sensor isn't quite
     * fully in contact with the flywheel yet. The values set for storing times
     * and speeds will probably affect the shooting as well and need to be
     * tested. The current implementation uses a constant flywheel speed,
     * but allowing the user to adjust the speed could allow shooting balls at
     * different distances.
     */
    if (!shooting && controller.getAButton()) {
      shooting = true;
      flywheel.set(flywheelShootingSpeed);
      belts.set(beltsShootingSpeed);
      shootingTimer.start();
    }

    if (shooting && shootingTimer.hasElapsed(timeToShoot)) {
      flywheel.set(0);
      belts.set(0);
      shootingTimer.stop();
      shootingTimer.reset();
    }



    /* Debug Info - There's probably some cooler way to present this info so
     * it's actually legible but this works:tm:.
     */
    if (debugTimer.hasElapsed(1)) {
      System.out.println("----------------------");
      System.out.println("Lower: " + lower.get());
      System.out.println("Center: " + center.get());
      System.out.println("Upper: " + upper.get());
      System.out.println("Balls: " + Arrays.toString(ballStorage));

      debugTimer.restart();
    }
  }

  /* Everything below is unused, but I don't feel like hunting down mentions
   * of these functions in other files just to be able to remove their useless
   * definitions.
   */
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
