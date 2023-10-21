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

  private final double lowerSensorCutoff = 0.38;
  private final double centerSensorCutoff = 0.25;
  private final double upperSensorCutoff = 0.3;

  private final double storingSpeed = -0.4;
  private final double shootingSpeed = 0.7;

  private final double timeToNextSensor = 0.5;
  private final double timeToShoot = 1;

  private boolean started = false;
  private Timer debugTimer;

  private boolean storing = false;
  private Timer storingTimer;

  private boolean shooting = false;
  private Timer shootingTimer;

  private boolean[] ballStorage;
  
  /**
   * Shoots a ball from the storage
   */

  /** Creates a new ExampleSubsystem. */
  public BallsSubsystem() {
    flywheel = new CANSparkMax(3, MotorType.kBrushless);
    flywheel2 = new CANSparkMax(16, MotorType.kBrushless);
    flywheel2.follow(flywheel);
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
    // This method will be called once per scheduler run
    if (!started) {
      started = true;
      debugTimer.start();
      return;
    }


    // Update Ball Storage
    ballStorage[0] = upper.get() > upperSensorCutoff;
    ballStorage[1] = center.get() > centerSensorCutoff;
    ballStorage[2] = lower.get() > lowerSensorCutoff;


    // Storage Logic
    // Tries to advance a ball throughout the storage unless there is already a ball directly in front of it
    if (!storing && ((!ballStorage[1] && ballStorage[1]) || (!ballStorage[1] && ballStorage[2]))) {
      storing = true;
      belts.set(storingSpeed);
      storingTimer.start();
    }

    if (storing && storingTimer.hasElapsed(timeToNextSensor)) {
      belts.set(0);
      storingTimer.stop();
      storingTimer.reset();
    }

    // Shooting
    if (!shooting && controller.getAButton()) {
      shooting = true;
      flywheel.set(shootingSpeed);
      shootingTimer.start();
    }

    if (shooting && shootingTimer.hasElapsed(timeToShoot)) {
      flywheel.set(0);
      shootingTimer.stop();
      shootingTimer.reset();
    }



    
    if (debugTimer.hasElapsed(0.1)) {
      System.out.println("----------------------");
      System.out.println("Lower: " + lower.get());
      System.out.println("Center: " + center.get());
      System.out.println("Upper: " + upper.get());
      System.out.println("Balls: " + Arrays.toString(ballStorage));

      debugTimer.restart();
    }
  }


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
