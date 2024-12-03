#include "sensesp_app.h"
#include "SSD1306Display.h"


//////////////////////////////////////////////////////////////
// Declare the static member variables storage 
// 

// Yeah, the rows and columns are flipped... Sue me.
DisplayCell sensesp::SSD1306Display::row_col_data[NUM_DISPLAY_COLS][NUM_DISPLAY_ROWS]; 

// A flag to make the display data init only happen once
bool sensesp::SSD1306Display::firstRun = false;

// There is only one display for all the data cells
Adafruit_SSD1306* sensesp::SSD1306Display::display = NULL;


//////////////////////////////////////////////////////////////
// CONSTRUCTOR
//

sensesp::SSD1306Display::SSD1306Display(Adafruit_SSD1306* disp, 
                                        int8_t displayRow, 
                                        int8_t displayCol,
                                        String unit,
                                        uint8_t fontSize) {

    // Check to see if the display class has ever been instantiated before this.
    // If not, start a periodic timer on the sensesp app to update the dislay.
    if( ! firstRun ) {
        // Do this stuff only once for all instances
        firstRun = true;
        display = disp;

        // Initialize the display cell array
        for(int col=0; col < NUM_DISPLAY_COLS; col++) {
            for(int row=0; row < NUM_DISPLAY_ROWS; row++) {
                this->row_col_data[col][row].value = 0.0;
                this->row_col_data[col][row].hasData = false;
                this->row_col_data[col][row].unit = "";
                this->row_col_data[col][row].fontSize = 1;
            }
        }

        // Start a periodic display update to show new data from all data cells
        SensESPBaseApp::get_event_loop()->onRepeat(750, [this]() { this->updateDisplay(); });
    }

    // Per this data cell info
    cur_row = displayRow;
    cur_col = displayCol;
    this->row_col_data[displayCol][displayRow].unit = unit;
    this->row_col_data[displayCol][displayRow].fontSize = fontSize;
}

// This is called by a sensesp producer when this display instance is connected to it
void sensesp::SSD1306Display::set(const float& new_value) {
    // TODO: What do we need to do to display this data?
    //  -- Just change an array of the data and periodically update it?

    // Yep.
    row_col_data[cur_col][cur_row].value = new_value;

    // This is idempotent; it is assumed that when the cell has data once it needs updates always
    row_col_data[cur_col][cur_row].hasData = true;

    // Pass the value through to the next transform in the chain (if any)
    this->emit(new_value);
}

void sensesp::SSD1306Display::updateDisplay() {
    display->setRotation(DISPLAY_ROTATION);
    display->clearDisplay(); // Hopefully, this won't blink or anything weird. Shouldn't...
    display->setTextSize(1); // Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
    display->setTextColor(SSD1306_WHITE);
    
    // Run through the data cells and draw values into the corresponding display cells
    for(int col=0; col < NUM_DISPLAY_COLS; col++) {
        for(int row=0; row < NUM_DISPLAY_ROWS; row++) {
            // Only activate a data cell for writing only if it has data from the set() function
            if(this->row_col_data[col][row].hasData) {
                display->setCursor( col*(SCREEN_WIDTH/NUM_DISPLAY_COLS), 
                                    row*(SCREEN_HEIGHT/NUM_DISPLAY_ROWS));
                display->setTextSize(this->row_col_data[col][row].fontSize);
                display->printf("%0.1f%s",  this->row_col_data[col][row].value, 
                                            this->row_col_data[col][row].unit);
            }
        }
    }

    // Apply the display updates
    display->display();
}
