#include <Arduino.h>

class RingBuffer {
private:
    byte* mem;
    uint write_p;
    uint read_p;
    uint n;
    uint len;
    bool full;
    uint max;

public:
    RingBuffer(uint len, uint n): n(n), len(len) {
        mem = (byte*) malloc(len * n);
        write_p = 0;
        read_p = 0;
        full = false;
        max = len * n;
    }

    ~RingBuffer() {
        free(mem);
    }

    byte* read() {
        if (read_p == write_p && !full) {
            return NULL;
        } else {
            full = false;
            read_p = (read_p + len) % max;
            return &mem[read_p];
        }
    }

    void write(byte* data) {
        // Block until a spot is available
        while (read_p == write_p && full) {
            yield();
        }

        memcpy(&mem[write_p], data, len);

        write_p = (write_p + len) % max;
        if (write_p == read_p) {
            full = true;
        }
    }
};