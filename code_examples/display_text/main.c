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
    int red = gdImageColorAllocate(im, 255, 0, 0);

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
    printf("Usage:\nPrint from inputs from file: sudo %s -f <text_file>\nPrint text from command line: sudo %s -i '<text>'\n", prog, prog);
}

void exit_fail(char* prog) {
    help(prog);
    lcd_deinit();
    exit(EXIT_FAILURE);
}

int main(int argc, char* argv[]) {
    lcd_init();

    if(argc != 3) {
        exit_fail(argv[0]);
    }
    if (!strcmp(argv[1], "-f"))
    {
        FILE *f = fopen(argv[2], "r");
        if(f == NULL) {
            printf("Could not open file %s\n", argv[2]);
            exit_fail(argv[0]);
        }

        // Get file size
        fseek(f, 0, SEEK_END); 
        int size = ftell(f);

        if(!size) {
            printf("File is empty %s\n", argv[2]);
            exit_fail(argv[0]);
        }

        fseek(f, 0, SEEK_SET); 
        char* text = (char*)malloc(size);
        fread(text, 1, size, f);
        fclose(f);

        demo_text(text);
    }
    else {
        if (!strcmp(argv[1], "-i"))
            demo_text(argv[2]);
        else
            exit_fail(argv[0]);
    }

    lcd_deinit();
    
    return EXIT_SUCCESS;
}