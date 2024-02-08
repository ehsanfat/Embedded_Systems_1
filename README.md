# Embedded_Systems_1
## Programming the PIC controller- dsPIC30f

# UART2 Communication Project



This project is designed to simulate **UART2** communication and display received characters on an **LCD** screen, while also providing functionalities triggered by **button** presses.


## Overview
The project aims to achieve the following requirements:

There are two types of synchronization and they can complement each other:
1.  Simulate an algorithm that requires 7 ms for execution and operates at 100 Hz.
2.  Read characters from UART2 and display them on the first row of the LCD.
3.  Clear the first row and restart writing when the end of the row is reached.
4.  Clear the first row upon receiving CR ('\r') or LF ('\n') characters.
5.  Display the number of characters received from UART2 on the second row.
6.  Send the current number of characters received to UART2 upon pressing button S5.
7.  Clear the first row and reset the characters received counter upon pressing button S6.

## Usage

To use this project:

1.  **Hardware Setup:** Connect the UART2 and LCD components to your microcontroller according to the hardware specifications.
2.  **Software Setup:** Upload the provided code to your microcontroller using your preferred development environment.
3.  **Operation:**
    -   Press button S5 to send the current number of received characters to UART2.
    -   Press button S6 to clear the first row and reset the characters received counter.
## Implementation Details

-   **Execution Time:** The algorithm simulates a real-world scenario by requiring 7 ms for execution and operating at 100 Hz.
-   **UART2 Communication:** Characters received from UART2 are displayed on the LCD's first row. Upon receiving CR or LF characters, the first row is cleared.
-   **Character Counter:** The number of characters received from UART2 is displayed on the second row.
-   **Button Functionality:** Button S5 triggers the transmission of the character count to UART2, while button S6 clears the first row and resets the character counter.

## Additional Notes

-   **Buffer Overflow:** Ensure that the LCD buffer size is sufficient to handle the maximum expected input.
-   **Interrupt Handling:** Implement interrupt routines for button presses and UART2 communication as necessary.
-   **Error Handling:** Include error handling mechanisms for cases such as buffer overflow or communication errors.
via a Handlebars template (on a blog for example).
