// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.AnalogGyro; 

public class Robot extends TimedRobot {
  
  // Lista de pasos para bailar payaso de rodeo
  private enum AutoState {
    GIRAR_A_DERECHA,    // Gira +90° respecto a su orientación inicial
    AVANZAR_DERECHA,    // Camina 1.8s
    GIRAR_180,          // Gira 180°
    AVANZAR_IZQUIERDA,  // Camina 1.8s
    GIRAR_AL_FRENTE,    // Vuelve a alinearse a 0°
    AVANZAR_ATRAS,      // Retrocede 1.8s
    AVANZAR_ADELANTE,   // Avanza 1.8s
    GIRAR_IZQUIERDA_90, // Giro intermedio indicado en la lista ("girar a la izquierda")
    VERIFICAR_REPETICION,// Controla el bucle de "repetir 4 veces"
    GIRO_360,           // Vuelta completa al terminar las 4 repeticiones
    REINICIAR_CICLO     // Bucle infinito: "repetir por siempre todo lo anterior"
  }

  private AutoState currentStep;
  private final Timer stateTimer = new Timer(); 
  private AnalogGyro gyro; 

  // Contadores para controlar los bucles de la imagen
  private int repeticionesBloque = 0; // Lleva la cuenta del "repetir 4"
  private double anguloReferencia = 0.0; // Almacena el ángulo del robot al iniciar un giro

  // Motores de la transmisión
  private final Spark leftMotor1 = new Spark(0);
  private final Spark leftMotor2 = new Spark(1);
  private final Spark rightMotor1 = new Spark(2);
  private final Spark rightMotor2 = new Spark(3);

  public Robot() {}

  @Override
  public void robotInit() {
    gyro = new AnalogGyro(0); 
    gyro.calibrate(); 

    leftMotor1.setInverted(false);
    leftMotor2.setInverted(false);
    rightMotor1.setInverted(true); 
    rightMotor2.setInverted(true);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    gyro.reset(); 
    repeticionesBloque = 0;
    anguloReferencia = 0.0;
    
    currentStep = AutoState.GIRAR_A_DERECHA;
    
    stateTimer.reset();
    stateTimer.start();
  }

  @Override
  public void autonomousPeriodic() {
    double currentAngle = gyro.getAngle();

    switch (currentStep) {
      
      /// --- PASO 1: 2 pasos a la derecha (Girar 90° a la derecha y avanzar 1.8s) --- ///
      
      case GIRAR_A_DERECHA:
        // Buscamos que el ángulo disminuya 90° respecto a donde empezamos (Giro rápido a potencia 1.0)
        if (currentAngle > anguloReferencia - 90.0) {
          setDriveSpeeds(1.0, -1.0); 
        } else {
          stopMotors();
          stateTimer.reset();
          currentStep = AutoState.AVANZAR_DERECHA;
        }
        break;

      case AVANZAR_DERECHA:
        if (stateTimer.get() < 1.8) {
          setDriveSpeeds(0.4, 0.4);
        } else {
          stopMotors();
          anguloReferencia = currentAngle; // Guardamos el ángulo actual (+90° aprox)
          currentStep = AutoState.GIRAR_180; // Saltamos directo al nuevo paso
        }
        break;

       /// --- PASO 2: 2 pasos a la izquierda (Girar 180° y avanzar 1.8 otra vez) --- ///

       case GIRAR_180:
        // Como estamos mirando a la derecha, sumamos 180° completos para terminar mirando a la izquierda
        if (currentAngle < anguloReferencia + 180.0) {
          setDriveSpeeds(-1.0, 1.0); // Giro rápido a la izquierda
        } else {
          stopMotors();
          stateTimer.reset();
          currentStep = AutoState.AVANZAR_IZQUIERDA;
        }
        break;


      case AVANZAR_IZQUIERDA:
        if (stateTimer.get() < 1.8) {
          setDriveSpeeds(0.4, 0.4);
        } else {
          stopMotors();
          anguloReferencia = currentAngle;
          currentStep = AutoState.GIRAR_AL_FRENTE;
        }
        break;

      /// --- PASO 3: 2 pasos para atrás (Alinearse al centro y retroceder 1.8s) --- ///
      
      case GIRAR_AL_FRENTE:
        // Deshace el giro izquierdo regresando 90° a la derecha
        if (currentAngle > anguloReferencia - 90.0) {
          setDriveSpeeds(1.0, -1.0);
        } else {
          stopMotors();
          stateTimer.reset();
          currentStep = AutoState.AVANZAR_ATRAS;
        }
        break;

      case AVANZAR_ATRAS:
        if (stateTimer.get() < 1.8) {
          setDriveSpeeds(-0.4, -0.4); // Potencia negativa = reversa
        } else {
          stopMotors();
          stateTimer.reset();
          currentStep = AutoState.AVANZAR_ADELANTE;
        }
        break;

      /// --- PASO 4: 2 pasos para delante (Avanzar 1.8s) --- ///
      case AVANZAR_ADELANTE:
        if (stateTimer.get() < 1.8) {
          setDriveSpeeds(0.4, 0.4);
        } else {
          stopMotors();
          anguloReferencia = currentAngle;
          currentStep = AutoState.GIRAR_IZQUIERDA_90;
        }
        break;

      /// --- PASO 5: Girar a la izquierda --- ///
      case GIRAR_IZQUIERDA_90:
        if (currentAngle < anguloReferencia + 90.0) {
          setDriveSpeeds(-1.0, 1.0);
        } else {
          stopMotors();
          currentStep = AutoState.VERIFICAR_REPETICION;
        }
        break;

      /// --- Repetir 4 veces --- ///

      case VERIFICAR_REPETICION:
        repeticionesBloque++;
        if (repeticionesBloque < 4) {
          // Si no van 4 veces, actualizamos la referencia y volvemos a empezar la secuencia del bloque
          anguloReferencia = currentAngle;
          currentStep = AutoState.GIRAR_A_DERECHA;
        } else {
          // Si ya completó las 4 repeticiones, pasa al giro de 360
          repeticionesBloque = 0; // Limpiamos contador
          anguloReferencia = currentAngle;
          currentStep = AutoState.GIRO_360;
        }
        break;

      // --- PASO 6: 360 ---
      case GIRO_360:
        // Da una vuelta completa rápida hacia la izquierda sumando 360° al rumbo actual
        if (currentAngle < anguloReferencia + 360.0) {
          setDriveSpeeds(-1.0, 1.0);
        } else {
          stopMotors();
          currentStep = AutoState.REINICIAR_CICLO;
        }
        break;

      // --- Repetir por siempre todo lo anterior --- ///

      case REINICIAR_CICLO:
        // Reinicia la máquina al estado inicial recreando un bucle infinito continuo
        anguloReferencia = currentAngle;
        currentStep = AutoState.GIRAR_A_DERECHA;
        break;

      default:
        stopMotors();
        break;
    }
  }

  private void setDriveSpeeds(double leftSpeed, double rightSpeed) {
    leftMotor1.set(leftSpeed);
    leftMotor2.set(leftSpeed);
    rightMotor1.set(rightSpeed);
    rightMotor2.set(rightSpeed);
  }

  private void stopMotors() {
    setDriveSpeeds(0.0, 0.0);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    stateTimer.stop();
  }

  @Override
  public void disabledPeriodic() {}
}
