// package frc.robot;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import frc.robot.util.GeomUtil;
// import frc.robot.util.VirtualSubsystem;
// import lombok.experimental.ExtensionMethod;
// @ExtensionMethod({GeomUtil.class})
// public class LED extends VirtualSubsystem{

//   public static double flashDelay = .2;
//   public static int flashIndex = 0;
//   public static AddressableLED ledStrip = new AddressableLED(9);
//   public static AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(40);
//   public static int[] rgb = {0, 0, 255};
//   static int topA = 159;
//   static int bottomA = 150;
//   static int topB = 149;
//   static int bottomB = 140;
//   static boolean positive = false;
//   static int pixelHue = 0;

//   public void periodic(){
//     //led
//        for (int i = 0; i < ledBuffer.getLength(); i++) {
//       ledBuffer.setRGB(i, 255, 0, 0);
//       if (i >= bottomA && i <= topA) {

//         ledBuffer.setRGB(i, 255, 255, 0);
//       }
//       if (i >= bottomB && i <= topB) {

//         ledBuffer.setRGB(i, 255, 255, 0);
//       }

//       //  if (i > 6){
//       //     ledBuffer.setRGB(i-6,255,0,0);
//       //  }

//     }
//     ledStrip.setData(ledBuffer);
//     if (positive == false) {
//       topA = topA - 3;
//       bottomA = bottomA - 3;
//       topB = topB + 3;
//       bottomB = bottomB + 3;
//     } else {
//       topA = topA + 3;
//       bottomA = bottomA + 3;
//       topB = topB - 3;
//       bottomB = bottomB - 3;
//     }

//     if (topA < 5) {
//       positive = true;
//     }
//     if (bottomA > 295) {
//       positive = false;
//     }
//     // }
//   }

//   public static void gotNote(boolean lightStop) {
//     ledBuffer.setRGB(100, 255, 255);
//   }
// }

// //   public static void Animate() {
// //     // if (Map.lightStop.get()){
// //     //     if (flashIndex == 0 ){
// //     //          for (int i = 0; i < ledBuffer.getLength(); i++){
// //     //      ledBuffer.setRGB(i,135,31,135);
// //     //         }
// //     //         if (Timer.getFPGATimestamp()-flashDelay > .1){
// //     //             flashDelay = Timer.getFPGATimestamp();
// //     //             flashIndex = 1;
// //     //         }

// //     //      } else if (flashIndex == 1){
// //     //          for (int i = 0; i < ledBuffer.getLength(); i++){
// //     //      ledBuffer.setRGB(i,0,0,0);
// //     //         }
// //     //         if (Timer.getFPGATimestamp()-flashDelay > .1){
// //     //             flashDelay = Timer.getFPGATimestamp();
// //     //             flashIndex = 0;
// //     //         }

// //     //      }

// //     //     ledStrip.setData(ledBuffer);
// //     // }

// //     //  else {

// //     for (int i = 0; i < ledBuffer.getLength(); i++) {
// //       ledBuffer.setRGB(i, 255, 0, 0);
// //       if (i >= bottomA && i <= topA) {

// //         ledBuffer.setRGB(i, 255, 255, 0);
// //       }
// //       if (i >= bottomB && i <= topB) {

// //         ledBuffer.setRGB(i, 255, 255, 0);
// //       }

// //       //  if (i > 6){
// //       //     ledBuffer.setRGB(i-6,255,0,0);
// //       //  }

// //     }
// //     ledStrip.setData(ledBuffer);
// //     if (positive == false) {
// //       topA = topA - 3;
// //       bottomA = bottomA - 3;
// //       topB = topB + 3;
// //       bottomB = bottomB + 3;
// //     } else {
// //       topA = topA + 3;
// //       bottomA = bottomA + 3;
// //       topB = topB - 3;
// //       bottomB = bottomB - 3;
// //     }

// //     if (topA < 5) {
// //       positive = true;
// //     }
// //     if (bottomA > 295) {
// //       positive = false;
// //     }
// //     // }
// //   }

// //   public static void gotNote(boolean lightStop) {
// //     ledBuffer.setRGB(100, 255, 255);
// //   }
// // }
