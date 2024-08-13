// // Copyright (c) 2024 FRC 6328
// // http://github.com/Mechanical-Advantage
// // Copyright (c) 2024 FRC 2486
// // http://github.com/Coconuts2486-FRC
// //
// // This program is free software; you can redistribute it and/or
// // modify it under the terms of the GNU General Public License
// // version 3 as published by the Free Software Foundation or
// // available in the root directory of this project.
// //
// // This program is distributed in the hope that it will be useful,
// // but WITHOUT ANY WARRANTY; without even the implied warranty of
// // MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// // GNU General Public License for more details.

// package frc.robot;

// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import lombok.Builder;

// /** Fudge factors for critical field locations which can be tuned per alliance at events. */
// public final class FudgeFactors {
//   @Builder
//   public static class FudgedTransform {
//     @Builder.Default private final Transform2d red = new Transform2d();

//     @Builder.Default private final Transform2d blue = new Transform2d();

//     public Transform2d getTransform() {
//       return DriverStation.getAlliance()
//           .map(alliance -> alliance == DriverStation.Alliance.Red ? red : blue)
//           .orElseGet(Transform2d::new);
//     }
//   }

//   private FudgeFactors() {}

//   public static final FudgedTransform speaker = FudgedTransform.builder().build();

//   public static final FudgedTransform amp = FudgedTransform.builder().build();

//   public static final FudgedTransform centerPodiumAmpChain = FudgedTransform.builder().build();

//   public static final FudgedTransform centerAmpSourceChain = FudgedTransform.builder().build();

//   public static final FudgedTransform centerSourcePodiumChain =
// FudgedTransform.builder().build();
// }
