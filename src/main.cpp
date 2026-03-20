#include "DataLogger.h"
#include "GPA.h"
#include "IO_handler.h"
#include "mbed.h"
#include "realtime_thread.h"
#include "uart_comm_thread_receive.h"
#include "uart_comm_thread_send.h"

// IMPORTANT:
// - Do NOT set the BufferedSerial to non-blocking mode, as this will break proper communication!

// Changelog (unsorted):
// - Eigen is now working for the Nucleo L432KC
// - All operators in IIR_Filter and LinearCharacteristics were removed, you have to use .evaluate and .apply now
// - Encoder now replaces EncoderCounterIndex and Enc_unwrap
// - DebounceIn is now used for the button
// - Extended EncoderCounter so it will also work with Mbed CE
// - Simplified logic, removed statemachine class and moved everything to realtime_thread
// - Added GPA and GUI
// - Updated GUI
// - Added AvgFilter for vizualization of set values in GUI
// - Replaced IIR_Filter with a sligthly more future proof implementation
// - Removed PID_CNtrl, integrator is now part of IIR_Filter

float Ts = 1.0f / 1.0e3f;
GPA myGPA(1.0f, 245.0f, 30, 0.1f, 0.2f, Ts); // setup here does not affect the actual used parameters, they are set via
                                             // the UART communication via MATLAB
DataLogger myDataLogger(1);

int main()
{
    // Input/Output handling
    IO_handler io_handler(Ts);

    // Communication
    BufferedSerial uart_serial(USBTX, USBRX, 115200);                     // leave this blocking!
    uart_comm_thread_send uart_com_send(&io_handler, &uart_serial, .01f); // send communication thread
    uart_comm_thread_receive uart_com_receive(&uart_serial, .01f);        // receive communication thread

    // Real-Time Thread
    realtime_thread rt_thread(&io_handler, Ts);

    // Start the three threads
    uart_com_receive.start_uart();
    uart_com_send.start_uart();
    rt_thread.start_loop();

    while (true) {
        ThisThread::sleep_for(500ms);
    }
}
