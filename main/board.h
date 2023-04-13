#pragma once

// RGB SPI interface
#define LCD_SPI_DATA0     9  /*!< for 1-line SPI, this also refered as MOSI */
#define LCD_SPI_CLK       10
#define LCD_SPI_CS        0
#define LCD_SPI_DC        -1
#define LCD_SPI_RST       -1

// RGB interface
#define LCD_PCLK_GPIO     (14)
#define LCD_VSYNC_GPIO    (12)
#define LCD_HSYNC_GPIO    (11)
#define LCD_DE_GPIO       (13)

#define RGB_B0       (10)  // B0
#define RGB_B1       (9)  // B1
#define RGB_B2       (40)   // B2
#define RGB_B3       (20)  // B3
#define RGB_B4       (19)  // B4
#define RGB_G0       (41)   // G0
#define RGB_G1       (46)  // G1
#define RGB_G2       (3)  // G2
#define RGB_G3       (42)  // G3
#define RGB_G4       (8)  // G4
#define RGB_G5      (18)  // G5
#define RGB_R0      (2)  // R0
#define RGB_R1      (17)  // R1
#define RGB_R2      (16)   // R2
#define RGB_R3      (1)  // R3
#define RGB_R4      (15)  // R4

#define LCD_DATA0_GPIO    (RGB_B0)   // B0
#define LCD_DATA1_GPIO    (RGB_B1)   // B1
#define LCD_DATA2_GPIO    (RGB_B2)   // B2
#define LCD_DATA3_GPIO    (RGB_B3)   // B3
#define LCD_DATA4_GPIO    (RGB_B4)   // B4
#define LCD_DATA5_GPIO    (RGB_G0)   // G0
#define LCD_DATA6_GPIO    (RGB_G1)    // G1
#define LCD_DATA7_GPIO    (RGB_G2)   // G2
#define LCD_DATA8_GPIO    (RGB_G3)   // G3
#define LCD_DATA9_GPIO    (RGB_G4)   // G4
#define LCD_DATA10_GPIO   (RGB_G5)   // G5
#define LCD_DATA11_GPIO   (RGB_R0)   // R0
#define LCD_DATA12_GPIO   (RGB_R1)    // R1
#define LCD_DATA13_GPIO   (RGB_R2)    // R2
#define LCD_DATA14_GPIO   (RGB_R3)    // R3
#define LCD_DATA15_GPIO   (RGB_R4)    // R4
#define LCD_DISP_EN_GPIO  (-1)

#define LCD_PIN_BK_LIGHT       45

#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

