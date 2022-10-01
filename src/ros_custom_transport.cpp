#include "pros/apix.h"

__FILE* rosFile;

extern "C" bool v5_serial_open(struct uxrCustomTransport * transport){
    pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    rosFile = fopen("/ser/sout", "r+");
    pros::c::fdctl(fileno(rosFile), SERCTL_DEACTIVATE, NULL);

    return true;
}

extern "C" bool v5_serial_close(struct uxrCustomTransport * transport){
    if (fclose(rosFile) == 0) {
        return true;
    }

    return false;
}

extern "C" size_t v5_serial_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    int wrote = 0;
    for (wrote = 0; wrote < len; wrote++) {
        fputc(buf[wrote], rosFile);
    }
    fflush(rosFile);

    return wrote;
}

extern "C" size_t v5_serial_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    int read = 0;
    for (read = 0; read < len; read++) {
        buf[read] = (uint8_t) fgetc(rosFile);
    }

    return read;
}
