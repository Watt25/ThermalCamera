
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"


#define TFT_CS   15 
#define TFT_DC    2
#define TFT_MOSI 13
#define TFT_CLK  14
#define TFT_RST  26
#define TFT_MISO 12
#define TFT_LED  27
#define INTERPOLATED_COLS 96
#define INTERPOLATED_ROWS 72

   float get_point (float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
   void set_point (float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
   void get_adjacents_1d (float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
   void get_adjacents_2d (float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
   float cubicInterpolate (float p[], float x);
   float bicubicInterpolate (float p[], float x, float y);
   void interpolate_image (float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);
   float dest_2d [INTERPOLATED_COLS * INTERPOLATED_ROWS];


// color LUT
   const uint16_t camColors[] = {0x480F,                                                       
   0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
   0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
   0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
   0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
   0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
   0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
   0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
   0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
   0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
   0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
   0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
   0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
   0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
   0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
   0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
   0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
   0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
   0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
   0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
   0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
   0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
   0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
   0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
   0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
   0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
   0xF080,0xF060,0xF040,0xF020,0xF800,};


  

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

const byte MLX90640_address = 0x33; // default address of the MLX90640

#define TA_SHIFT 8 // open air  shift 

static float mlx90640To[768];
static float mlx90640To1[768];
static float mlx90640To2[768];
static float mlx90640To3[768];
static float mlx90640To4[768];
paramsMLX90640 mlx90640;

int xPos, yPos;                                // position
int R_colour, G_colour, B_colour;              // RGB
int i, j;                                      
float T_max, T_min;                            // max min T 
float T_center;                                // center T
int make_int;


void setup()
   {
    Serial.begin(115200);
    
    Wire.begin();
    Wire.setClock(400000); // I2C 400kHz

    while (!Serial); 
    
    Serial.println("MLX90640");

    if (isConnected() == false)
       {
        Serial.println("MLX90640 not found");
        while (1);
       }
     
    Serial.println("MLX90640 OK");

  
    int status;
    uint16_t eeMLX90640[832];
    
    status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  
    if (status != 0)
       Serial.println("Failed...");

    status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  
    if (status != 0)
       {
        Serial.println("Parameter failed...");
        Serial.print(" status = ");
        Serial.println(status);
       }

  

    MLX90640_I2CWrite(0x33, 0x800D, 6401); 
    MLX90640_SetRefreshRate(MLX90640_address, 0x02); //1Hz refresh 
    //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //4Hz refresh 
       
    pinMode(TFT_LED, OUTPUT);
    digitalWrite(TFT_LED, HIGH);

    tft.begin();

    tft.setRotation(3);

    tft.fillScreen(ILI9341_BLACK);
 

    tft.drawLine(250, 210 - 0, 258, 210 - 0, tft.color565(255, 255, 255));
    tft.drawLine(250, 210 - 30, 258, 210 - 30, tft.color565(255, 255, 255));
    tft.drawLine(250, 210 - 60, 258, 210 - 60, tft.color565(255, 255, 255));
    tft.drawLine(250, 210 - 90, 258, 210 - 90, tft.color565(255, 255, 255));
    tft.drawLine(250, 210 - 120, 258, 210 - 120, tft.color565(255, 255, 255));
    tft.drawLine(250, 210 - 150, 258, 210 - 150, tft.color565(255, 255, 255));
    tft.drawLine(250, 210 - 180, 258, 210 - 180, tft.color565(255, 255, 255));

    tft.setCursor(220, 220);
    tft.setTextColor(ILI9341_WHITE, tft.color565(0, 0, 0));
    tft.print("T = ");    


    // scale vs T
    
 
    for (i = 0; i < 181; i++)
       {
        tft.drawLine(240, 210 - i, 250, 210 - i, camColors[i+255-180]);
       } 

   } 



void loop()
   {
    for (byte x = 0 ; x < 2 ; x++) //Read
       {
        uint16_t mlx90640Frame[834];
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    
        if (status < 0)
           {
            Serial.print("Get Frame Error: ");
            Serial.println(status);
           }

        float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

        float tr = Ta - TA_SHIFT; //T shift
        float emissivity = 0.95;

        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To1);
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To2);
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To3);
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To4);
       }

       for (i=0; i<768; i++)  mlx90640To[i]= ( mlx90640To1[i] + mlx90640To2[i] + mlx90640To3[i] + mlx90640To4[i] ) / 4;
    // T_min T_max

    mlx90640To[1*32 + 21] = 0.5 * (mlx90640To[1*32 + 20] + mlx90640To[1*32 + 22]);    // error-delete
    mlx90640To[4*32 + 30] = 0.5 * (mlx90640To[4*32 + 29] + mlx90640To[4*32 + 31]);   
    
    T_min = mlx90640To[0];
    T_max = mlx90640To[0];

    for (i = 1; i < 768; i++)
       {
        if((mlx90640To[i] > -41) && (mlx90640To[i] < 301))
           {
            if(mlx90640To[i] < T_min)
               {
                T_min = mlx90640To[i];
               }

            if(mlx90640To[i] > T_max)
               {
                T_max = mlx90640To[i];
               }
           }
        else if(i > 0)   // T out of range
           {
            mlx90640To[i] = mlx90640To[i-1];
           }
        else
           {
            mlx90640To[i] = mlx90640To[i+1];
           }
       }


   
  

    T_center = mlx90640To[11* 32 + 15];     // T_center

    // drawing
   
tft.setRotation(1);
    for (i = 0 ; i < 24 ; i++)
       {
        for (j = 0; j < 32; j++)
           {
            mlx90640To[i*32 + j] = 180.0 * (mlx90640To[i*32 + j] - T_min) / (T_max - T_min);
           }
       }
/*
for (i = 0 ; i < 24 ; i++)
       {
        for (j = 0; j < 32; j++)
           {
            make_int = mlx90640To[i*32 + j];
            tft.fillRect(300 - j * 7, 50 + i * 7, 7, 7, camColors[make_int+255-180]);

           }
       }

*/
   
    interpolate_image (mlx90640To, 24, 32, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS); 

   for (i=0; i<(INTERPOLATED_ROWS * INTERPOLATED_COLS); i++)
   {
    if (dest_2d[i]<0) dest_2d[i]=0;
    if (dest_2d[i]>180) dest_2d[i]=180;  
   }


    for (i = 0 ; i < INTERPOLATED_ROWS ; i++)
       {
        for (j = 0; j < INTERPOLATED_COLS; j++)
           {
            make_int = dest_2d[i*INTERPOLATED_COLS + j];
            tft.fillRect(300 - j * 2, 50 + i * 2, 2, 2, camColors[make_int+255-180]);
           
           }
       }

        tft.drawLine(204, 112, 204, 132, tft.color565(255, 255, 255));
        tft.drawLine(194, 122, 214, 122, tft.color565(255, 255, 255));
   
      tft.setRotation(3);
     
   
 
    tft.fillRect(260, 25, 37, 10, tft.color565(0, 0, 0));
    tft.fillRect(260, 205, 37, 10, tft.color565(0, 0, 0));    
   

    tft.setTextColor(ILI9341_WHITE, tft.color565(0, 0, 0));
    tft.setCursor(265, 25);
    tft.print(T_max, 1);
    tft.setCursor(265, 205);
    tft.print(T_min, 1);
    tft.setCursor(245, 220);
    tft.print(T_center, 1);

    tft.setCursor(300, 25);
    tft.print("C");
    tft.setCursor(300, 205);
    tft.print("C");
    tft.setCursor(265, 220);
    tft.print("C");
    
   
   }
   


   
   
// MLX90640 detected
boolean isConnected()
   {
    Wire.beginTransmission((uint8_t)MLX90640_address);
    if (Wire.endTransmission() != 0)
    return (false); 
    return (true);
   }   
   
   
   
   
   
   
   
   
   
   
   
