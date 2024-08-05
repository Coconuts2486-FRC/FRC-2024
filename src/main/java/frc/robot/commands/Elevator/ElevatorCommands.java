// package frc.robot.commands.Elevator;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.elevator.Elevator;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// public class ElevatorCommands {
//   private ElevatorCommands() {}

//   // negitive is up
//   public static Command manualElevator(
//       Elevator elevator,
//       DoubleSupplier upAxis,
//       DoubleSupplier downAxis,
//       BooleanSupplier bottomLimit,
//       BooleanSupplier topLimit) {
//     return Commands.run(
//         () -> {
//           if (bottomLimit.getAsBoolean()) {
//             if (upAxis.getAsDouble() > .1) {
//               elevator.runDutyCycle(-upAxis.getAsDouble());
//             } else {
//               elevator.runDutyCycle(0);
//             }
//           } else if (topLimit.getAsBoolean()) {

//             if (downAxis.getAsDouble() > .1) {
//               elevator.runDutyCycle(downAxis.getAsDouble());
//             } else {
//               elevator.runDutyCycle(0);
//             }
//           } else {
//             elevator.runDutyCycle(downAxis.getAsDouble() - upAxis.getAsDouble());
//           }
//         },
//         elevator);
//   }
// }
