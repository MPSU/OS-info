#include "lcd.h"
#include "gdfontg.h"

// Demo text

void demo_text(char* text) {

    gdImagePtr im = gdImageCreateTrueColor(LCD_WIDTH, LCD_HEIGHT);
    int white = gdImageColorAllocate(im, 255, 255, 255);

    gdImageString(im, gdFontGetGiant(), 0, 0, text, white);

    display_gd_image(im);
    gdImageDestroy(im);
}

void help(char* prog) {
    printf("Usage: sudo %s <text_file>\n", prog);
}

int main(int argc, char* argv[]) {
    lcd_init();

    if(argc != 2) {
        help(argv[0]);
    }
    else {
        FILE *f = fopen(argv[1], "r");
        if(f == NULL) {
            printf("Could not open file %s\n", argv[1]);
            help(argv[0]);
            exit(EXIT_FAILURE);
        }

        // Get file size
        fseek(f, 0, SEEK_END); 
        int size = ftell(f);

        if(!size) {
            printf("File is empty %s\n", argv[1]);
            exit(EXIT_FAILURE);
        }

        fseek(f, 0, SEEK_SET); 
        char* text = (char*)malloc(size);
        fread(text, 1, size - 1, f);
        fclose(f);

        demo_text(text);
    }

    lcd_deinit();
    
    return EXIT_SUCCESS;
}