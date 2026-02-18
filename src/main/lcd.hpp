#include <string>
#include <stdint.h>

void lcd_init();
void lcd_write_bits(bool rs, uint8_t b);
void lcd_write_text(std::string text);
