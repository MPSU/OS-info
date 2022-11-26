#include "lcd.h"
#include "gdfontg.h"

#define FONT_SIZE_X 9
#define FONT_SIZE_Y 15
#define STRING_MARGIN 5
#define STRING_LENGTH (int)(LCD_WIDTH / FONT_SIZE_X - 1)

// Demo text

void demo_text(char* text) {

    gdImagePtr im = gdImageCreateTrueColor(LCD_WIDTH, LCD_HEIGHT);
    int white = gdImageColorAllocate(im, 255, 255, 255);

    char buff[STRING_LENGTH + 1];
    int str_ptr = 0;
    int num_str = 0;
    for(int i = 0; i < strlen(text); i++) {
        if(text[i] == '\n' || str_ptr == STRING_LENGTH || text[i+1] == '\0') {
            strncpy(buff, text + i - str_ptr, (text[i] == '\n') ? str_ptr : str_ptr + 1);
            buff[str_ptr + ((text[i] == '\n') ? 0 : 1)] = '\0';
            gdImageString(im, gdFontGetGiant(), 0, num_str * FONT_SIZE_Y + STRING_MARGIN, buff, white);
            str_ptr = 0;
            num_str++;
        }
        else str_ptr++;
    }

    display_gd_image(im);
    gdImageDestroy(im);
}

void help(char* prog) {
    printf("Usage: sudo %s <text>\n", prog);
}

int main(int argc, char* argv[]) {
    lcd_init();

    if(argc != 2) {
        help(argv[0]);
    }
    else {
        demo_text(argv[1]);
    }

    lcd_deinit();
    
    return EXIT_SUCCESS;
}