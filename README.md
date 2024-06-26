# This is a library for 16 by 2 LCD screen based on HD44780 driver.
   The repo also includes a simple example projest made for STM32F302R8.
   Library was written for STM32F3xx microcontrollers. It requires TIM and I2C peripherals. It allows user to connect up to 8 displays per I2C peripheral.<br />


# User Manual:
   **IOC setup**:<br />
     1.Initialize High Speed Clock<br />
     2.Initialize I2C peripheral in Standard Mode<br />
     3.Initialize Timer with internal clock source<br />
  
  **Initializing the display**:<br />
    1. include "hd44780_lcd_i2c.h" header file<br />
    2. create your display structure<br />
      MODULE_ADDRESS is defined in hd44780_lcd_i2c.h file. It is a default address for this I2C driver.<br />
      Default address is required when driver pins A0, A1, A2 dont have pull-down resistors<br />
    ```
       LCD_HandleTypeDef my_display = {&hi2c, &htim, MODULE_ADDRESS, current_col, current_row, number_of_line};
    ```<br />
    2. Initialize the display with LCD_ibit function. Pass address to your display struccture <br />
    ```
    LCD_init(&my_display);
    ```<br />
    <br />
    **List of functions and examples of their use:**<br />
    LCD_init - initializes display and library's internal delay function<br />
    example:<br />
    ```
    LCD_init(&my_display);
    ```<br />
    <br />
    LCD_putchar - prints a single ascii character to the display.<br />
    example:<br />
    ```
    char c = 'x';
    ```<br />
    ```
    LCD_putchar(&my_display, c);
    ```<br />
    <br />
    LCD_printf - prints a string to the display<br />
    example:<br />
    ```
    char data[] = "HD44780";
    ```<br />
    ```
    LCD_printf(&my_display, data);
    ```<br />
    LCD_printf_align  - prints a string aligned that can be aligned to the middle or the right side of the display. ALIGN_RIGHT and ALIGN_MIDDLE constants are defined in hd44780_lcd_i2c.h file <br />
    example:<br />
        ```
    char data[] = "HD44780";
    ```<br />
    ```
    LCD_printf_align(&my_display, data, ALIGN_RIGHT);
    ```<br />
    <br />
    LCD_set_position - sets position of the cursor<br />
    example: sets position to column:0 and row:1<br />
    ```
    LCD_set_position(&my_display, 0, 1);
    ```<br />
    <br />
    LCD_clear - clears the display<br />
    example:<br />
    ```
    LCD_clear(&my_display);
    ```<br />
    <br />
    LCD_reset_position - sets cursor position to (0,0)<br />
    example:<br />
    ```
    LCD_reset_position(&my_display);
    ```<br />
    <br />
    
    

    
