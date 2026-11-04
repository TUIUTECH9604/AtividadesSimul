// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SimElevator;
import frc.robot.subsystems.SimHand;
import frc.robot.subsystems.SimIntake;

public class RobotContainer {

  private final TankSubsystem tank = new TankSubsystem();

  private final CommandXboxController driver = new CommandXboxController(0);
  private final XboxController driverget = new XboxController(0);

  private SimIntake mSimIntake = new SimIntake(20, 1.5, 3);
  private SimElevator mSimElevator = new SimElevator(400, 1.5);
  private SimHand mSimHand = new SimHand(-180, 180, 1.5, 3);


  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    autoChooser.setDefaultOption("Nenhum", Commands.none());
    autoChooser.addOption("Quadrado", Autos.quadrado(tank));
    autoChooser.addOption("Reto", Autos.reto(tank));
    autoChooser.addOption("autoZigZag", Autos.autoZigZag(tank));

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();

    tank.setDefaultCommand(tank.run(() -> {

      double leftX = driver.getLeftX();
      double leftY = driver.getLeftY();

      double joystickMagnitude = Math.hypot(leftX, leftY);

      // Joystick esquerdo sendo utilizado
      if (joystickMagnitude > 0.08) {

        tank.driveWithAngleControl(getLeftJoystickAngle(), driver.getRawAxis(3));

        // Tank normal
      } else {

        double forward = driver.getRawAxis(3);
        double backward = driver.getRawAxis(2);
        double rotation = driver.getRightX();

        tank.drive(forward, -backward, -rotation);
      }
    }));
  }

  private void configureBindings() {

    driver.start().onTrue(tank.runOnce(() -> tank.resetPose(new Pose2d(2.0, 4.0, new Rotation2d()))));

    driver.povUp().onTrue(mSimElevator.CMDsetPosition(400, 10)); // "400"
    driver.povDown().onTrue(mSimElevator.CMDsetPosition(0, 3));

    // driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3))); // "45"
    // driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)));
    driver.a().onTrue(Commands.runOnce(() -> mSimHand.setAngle(135, 3)).onlyIf(() -> SimElevator.getRawPosition() > 0.30));
    driver.b().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-135, 3)).onlyIf(() -> SimElevator.getRawPosition() > 0.30)); // "135"
    driver.povRight().onTrue(Commands.runOnce(() -> mSimHand.setAngle(-45, 3)).onlyIf(() -> SimElevator.getRawPosition() > 0.30));
    driver.povLeft().onTrue(Commands.runOnce(() -> mSimHand.setAngle(45, 3)).onlyIf(() -> SimElevator.getRawPosition() > 0.30));

    //  && SimHand.getRawAngle() > 0
    // new Trigger(tank::isFuelInsideRobot).onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)));
    // new Trigger(tank::isFuelInsideRobot).onTrue(Commands.runOnce(() -> mSimElevator.setPosition(0, 3)));

    driver.back().onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3)).onlyIf(() -> SimElevator.getRawPosition() > 0.10));

    coletar_Ground();

    new Trigger(tank::isFuelInsideRobot).onTrue(
    Commands.sequence(
        mSimElevator.CMDsetPosition(400, 3),
        Commands.waitUntil(() -> Math.abs(SimElevator.getRawPosition() - 0.40) < 0.005),

        Commands.runOnce(() -> mSimHand.setAngle(0, 3)),
        Commands.waitUntil(() -> Math.abs(SimHand.getRawAngle()) < 0.01),

        mSimElevator.CMDsetPosition(0, 3),
        Commands.waitUntil(() -> Math.abs(SimElevator.getRawPosition()) < 0.005)
    )
);

    // new Trigger(tank::isFuelInsideRobot).onTrue(
    // Commands.sequence(
    //     Commands.runOnce(() -> mSimElevator.CMDsetPosition(400, 3),
    //     Commands.waitUntil(() -> mSimElevator.getRawPosition() >= 400.0),

    //     Commands.runOnce(() -> mSimHand.setAngle(0, 3)),
    //     Commands.waitUntil(() -> mSimHand.getRawAngle() <= 0.01),

    //     Commands.runOnce(() -> mSimElevator.CMDsetPosition(0, 3))

    //     )
    //   ));

    // Commands.sequence(
    //   new Trigger(tank::isFuelInsideRobot).onTrue(Commands.runOnce(() -> mSimElevator.CMDsetPosition(400, 3))),
    //   new Trigger(tank::isFuelInsideRobot).onTrue(Commands.runOnce(() -> mSimHand.setAngle(0, 3))),
    //   new Trigger(tank::isFuelInsideRobot).onTrue(Commands.runOnce(() -> mSimElevator.CMDsetPosition(0, 3)))

    // );

    // new Trigger(tank::isFuelInsideRobot)
    // .onTrue(
    //     Commands.sequence(
    //         mSimElevator.CMDsetPosition(400, 3),
    //         Commands.runOnce(() -> mSimHand.setAngle(0, 3)),
    //         mSimElevator.CMDsetPosition(0, 3)
    //     )
    // );

    // new Trigger(tank::isFuelInsideRobot).onTrue(Commands.sequence(Commands.run(() -> mSimHand.setAngle(0, 3)).until(() -> mSimHand.getAngle() == 0), Commands.runOnce(() -> mSimElevator.setPosition(0, 3))));
    // new Trigger(tank::isFuelInsideRobot).onTrue(Commands.sequence(Commands.runOnce(() -> mSimElevator.CMDsetPosition(400, 3)), null).andThen(Commands.runOnce(() -> mSimHand.setAngle(0, 3))).andThen(Commands.runOnce(() -> mSimElevator.setPosition(0, 3))));
    // new Trigger(tank::isFuelInsideRobot).onTrue(Commands.sequence(mSimElevator.CMDsetPosition(400, 3), Commands.waitUntil(() -> mSimElevator.getRawPosition() <= 40.0), Commands.runOnce(() -> mSimHand.setAngle(0, 3)), Commands.waitUntil(() -> mSimHand.getRawAngle() <= 0.01), Commands.runOnce(() -> mSimElevator.setPosition(0, 3))));
    // new Trigger(tank::isFuelInsideRobot).onTrue(Commands.sequence(Commands.runOnce(() -> mSimElevator.CMDsetPosition(400, 3)), Commands.waitUntil(() -> mSimElevator.getRawPosition() <= 40.0), Commands.runOnce(() -> mSimHand.setAngle(0, 3)), Commands.waitUntil(() -> mSimHand.getRawAngle() <= 0.01), Commands.runOnce(() -> mSimElevator.setPosition(0, 3))));
    // deposit_Pose();.andThen(Commands.waitUntil(() ->mSimElevator.getRawPosition() <= 40.0))  .andThen(Commands.waitUntil(() -> mSimHand.getRawAngle() <= 0.01))
    

    driver.y().onTrue(tank.depositFuelCommand());

  }

  void coletar_Ground() {
    driver.leftBumper().whileTrue(mSimIntake.CMDintakeOn().onlyIf(() -> !tank.isFuelInsideRobot()));
    driver.leftBumper().whileFalse(mSimIntake.CMDintakeOff());
    new Trigger(tank::isFuelInsideRobot).onTrue(mSimIntake.CMDintakeOff());
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public double getLeftJoystickAngle() {

    double x = -driverget.getLeftX();
    double y = -driverget.getLeftY();

    if (Math.hypot(x, y) < 0.1) {
      return 0.0;
    }

    return Math.toDegrees(Math.atan2(x, y));
  }

  /**
   * Ativa o comando alvo, quando a condição é verdadeira.
   * 
   * @param null
   */
  private void onCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).onTrue(command);
  }

  /**
   * Matém um comando ativo, enquanto a condição é verdade.
   * 
   * @param null
   */
  private void whileCommandCondition(BooleanSupplier condition, Command command) {
    new Trigger(condition).whileTrue(command);
  }

  Command deposit() {
    return Commands.sequence(
        Commands.runOnce(() -> mSimHand.setVelocity(1)),
        tank.depositFuelCommand(),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> mSimHand.setVelocity(0)));
  }

}
