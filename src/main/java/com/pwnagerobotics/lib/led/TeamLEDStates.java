// package com.pwnagerobotics.lib.led;

// import com.pwnagerobotics.lib.led.jobs.*;
// import com.pwnagerobotics.pwnage2022.subsystems.LEDManager;
// import com.pwnagerobotics.pwnage2022.subsystems.LEDManager.LEDState;

// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;

// public class TeamLEDStates {
    
//     private static SendableChooser<LEDState> mLedChooser = new SendableChooser<LEDState>();
//     private final static String kDataKey = "Endgame LEDState Chooser";

//     public static LEDState
//     ALLIANCE, // Current Alliance Colora
//     PWNAGE, // 2451
//     MUKWONAGO_BEARS, // 930
//     WILDSTANG, // 111
//     ARGOS, // 1756
//     WINNOVATION; // 1625

//     public TeamLEDStates() {
//         ALLIANCE = new LEDState(
//             LEDManager.allMainLEDStrips(
//                 new BreatheJob(
//                     new Color[] {
//                         // AllianceColorUpdater.getAllianceColor(Color.kWhite);
//                         Color.kPurple
//                     },
//                     0.85
//                 )
//             )
//         );
//         mLedChooser.setDefaultOption("Alliance", ALLIANCE);

//         PWNAGE = new LEDState(
//             LEDManager.allMainLEDStrips(
//                 new PwnageJob(
//                     new Color[] {}
//                     ,
//                     0.5f
//                 )
//             )
//         );
//         mLedChooser.addOption("Pwnage", PWNAGE);

//         WILDSTANG = new LEDState(
//             LEDManager.allMainLEDStrips(
//             new RainbowJob(
//                 25, 
//                 0.05f
//             )
//         ));
//         mLedChooser.addOption("Wildstang (111)", WILDSTANG);

//         MUKWONAGO_BEARS = new LEDState(
//             LEDManager.allMainLEDStrips(
//                 new ChaseJob(
//                     new Color[] {
//                         Color.kBlue,
//                         Color.kBlue,
//                         Color.kBlue,
//                         Color.kBlack,
//                         Color.kBlack,
//                         Color.kBlack,
//                     }, 
//                     0.08f
//                 )
//             )
//         );
//         mLedChooser.addOption("Mukwonago Bears (930)", MUKWONAGO_BEARS);

//         ARGOS = new LEDState(
//             LEDManager.allMainLEDStrips(
//                 new FlashJob(
//                     new Color[] {
//                         Color.kGold,
//                         Color.kOrange,
//                         Color.kYellow
//                     }, 
//                     0.8f
//                 )
//             )
//         );
//         mLedChooser.addOption("Argos (1756)", ARGOS);

//         WINNOVATION = new LEDState(
//             LEDManager.allMainLEDStrips(
//                 new ChaseJob(
//                     new Color[] {
//                         ColorFactory.fromRGB(1, 0.2, 0),
//                         ColorFactory.fromRGB(1, 0.2, 0),
//                         Color.kDarkBlue,
//                         Color.kDarkBlue
//                     },
//                     0.3f
//                 )
//             )
//         );
//         mLedChooser.addOption("Winnovation (1625)", WINNOVATION);
//         SmartDashboard.putData(kDataKey, mLedChooser);
//     }

//     public static void showEndgameState() {
//         if (mLedChooser.getSelected() == null) {
//             ALLIANCE.show();
//             return;
//         }
//         mLedChooser.getSelected().show();
//     }

//     private static TeamLEDStates mInstance = null;

//     public synchronized static TeamLEDStates getInstance() {
//         if (mInstance == null) {
//             mInstance = new TeamLEDStates();
//         }
//         return mInstance;
//     }
// }
