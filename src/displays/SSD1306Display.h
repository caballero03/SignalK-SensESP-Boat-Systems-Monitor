#ifndef SSD1306_DISPLAY_H_
#define SSD1306_DISPLAY_H_

#include "sensesp/transforms/transform.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display width and height, in pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// OLED display rotation
#define DISPLAY_ROTATION 2 // 2 = upside-down (0,2 are landscape, 1,3 are portrait)

// Display data cells 128x64 pixels with font size 1 (6x8 chars) makes 4x8 data cell matrix
// Could be 16 rows of 10 char cells when rotated 90 degrees
#define NUM_DISPLAY_COLS 4
#define NUM_DISPLAY_ROWS 8

// Eight rows of four data cells
#define DATA_ROW_0 0
#define DATA_ROW_1 1
#define DATA_ROW_2 2
#define DATA_ROW_3 3
#define DATA_ROW_4 4
#define DATA_ROW_5 5
#define DATA_ROW_6 6
#define DATA_ROW_7 7

// Four data columns across the 128 pixels of the dsplay making 5-1/3 chars at font size 1 each
#define DATA_COL_0 0
#define DATA_COL_1 1
#define DATA_COL_2 2
#define DATA_COL_3 3

struct DisplayCell {
    float value;
    int fontSize;
    String unit;
    bool hasData;
};

namespace sensesp {

class SSD1306Display  : public FloatTransform {
 public:
    SSD1306Display( Adafruit_SSD1306* disp,
                    int8_t displayRow,
                    int8_t displayCol,
                    String unit = "",
                    uint8_t fontSize = 1);

    virtual void set(const float& new_value) override;

 protected:
    void updateDisplay();

private:
    static Adafruit_SSD1306* display;

    uint8_t cur_row;
    uint8_t cur_col;
    static DisplayCell row_col_data[NUM_DISPLAY_COLS][NUM_DISPLAY_ROWS];
    static bool firstRun;
};


} // End namespace sensesp
#endif