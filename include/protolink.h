//
// Created by tom on 12/28/19.
//

#ifndef PROTOLINK_PROTOLINK_H
#define PROTOLINK_PROTOLINK_H

#include <pb_decode.h>
#include "protolink.pb.h"

#include <functional>

#if not defined ESP8266 && not defined ESP32
typedef unsigned int uint;
    #include <cstdint>
    #include <string>
    #include <algorithm>

    #define YIELD
    #define String std::string
    #define to_str(p) case(p): return std::string(#p);

    int min(int a, int b) {
        return a < b ? a : b;
    }

#endif
#if defined ESP8266 || defined ESP32
#define YIELD yield();
    #define String String
    #define to_str(p) case(p): return #p;
    #define min min
#endif

#define LEN_N_BYTES 4


template <class Connection>
class ProtoLink {
public:
    enum class State {
        NO_CONN,
        RECV_LEN,
        RECV_MSG,
        ERR
    };

    // Initial state
    State state = State::NO_CONN;

    Connection* con;
    uint8_t* fb;
    uint fb_size;

    // Error message to be set when something bad happens
    char const* error_msg;

private:
    // Keep track of how many bytes to recieve next
    uint to_recv = 0;

    // Connection wrapper adapted from
    // https://github.com/eric-wieser/nanopb-arduino/blob/master/src/pb_arduino.cpp
    static bool pb_conn_read(pb_istream_t *stream, pb_byte_t *buf, size_t count) {
        auto *c = reinterpret_cast<Connection *>(stream->state);
        size_t written = c->readBytes(buf, count);
        return written == count;
    };

    pb_istream_s as_pb_istream(Connection* c, size_t count) {
        return {pb_conn_read, c, count};
    }

    // Nanopb decoding callback to write the incoming
    // framebuffer without first making a copy
    static bool write_fb(pb_istream_t *stream, const pb_field_t *field, void **arg) {
        auto* p = (ProtoLink<Connection>*) arg[0];
        if (stream->bytes_left > p->fb_size) {
            p->state = State::ERR;
            p->error_msg = "New framebuffer data exceeds framebuffer size";
            return false;
        }

        if (!pb_read(stream, p->fb, stream->bytes_left)) {
            //Serial.println("Dropped bad fb");
            // p->state = State::ERR;
            // p->error_msg = "Error writing framebuffer into memory";
            // return false;
        }

        return true;
    }

    // Nanopb decoding for oneof messages
    // https://github.com/nanopb/nanopb/blob/master/tests/oneof_callback/decode_oneof.c
    static bool command_decode_callback(pb_istream_t *stream, const pb_field_t *field, void **arg)
    {
        auto* p = (ProtoLink<Connection>*) arg[0];

        auto msg = (Command*)field->message;

        switch (field->tag) {
            case Command_updatefb_tag:
                auto submsg = (UpdateFB*)field->pData;
                submsg->fb.funcs.decode = write_fb;
                submsg->fb.arg = arg[0];
        };

        return true;
    }

    // Dummy function so we don't blow up if we forget to set a function pointer
    static void dummy() {
    }

    std::function<void(void)> _onFBReceived = &dummy;

public:
    ProtoLink(uint8_t* fbp, uint fbs){
        fb = fbp;
        fb_size = fbs;
    }

    void onFBReceived(std::function<void(void)> cb) {
        _onFBReceived = cb;
    }

    State init(Connection* c) {
        con = c;
        state = State::RECV_LEN;
        return state;
    }

    State run() {
        switch(state) {
            case State::NO_CONN:    break;
            case State::RECV_LEN:   run_RECV_LEN(); break;
            case State::RECV_MSG:    run_RECV_MSG();  break;
        }
        return state;
    }

    String stateToString() {
        switch (state) {
            to_str(State::NO_CONN);
            to_str(State::RECV_LEN);
            to_str(State::RECV_MSG);
            to_str(State::ERR);
        }
        return "Unknown"; // Unreachable, yet clang complains
    }

private:
    // State methods
    void run_RECV_LEN() {
        // We are using length prefix framing, so first recieve LEN_N_BYTES bytes
        // describing the length of the actual message
        uint len = 0;

        uint8_t lenbuf[LEN_N_BYTES];
        uint b_read = con->readBytes(lenbuf, LEN_N_BYTES);

        // If we timed out, do nothing
        if (b_read == 0) {
            return;
        }

        // Assemble bytes into int
        for (uint i = 0; i < LEN_N_BYTES; i++) {
            len |= (uint)lenbuf[LEN_N_BYTES - 1 - i] << i * 8;
        }

        // Set n bytes to recieve and start waiting for the message
        to_recv = len;
        state = State::RECV_MSG;
    }

    void run_RECV_MSG() {
        auto istream = (pb_istream_t)as_pb_istream(con, to_recv);

        Command msg = (Command)Command_init_zero;

        msg.cb_c.funcs.decode = command_decode_callback;
        msg.cb_c.arg = this;

        // Attempt decode
        if (!pb_decode(&istream, Command_fields, &msg))
        {
            state = State::ERR;
            if (error_msg == nullptr) {
                error_msg = PB_GET_ERROR(&istream);
            }
            return;
        }

        switch(msg.which_c) {
            case Command_updatefb_tag:
                _onFBReceived();
                break; // handled in decoding callback
        }

        // Go back to initial state to wait for next command
        state = State::RECV_LEN;
    }
};

#endif //PROTOLINK_PROTOLINK_H
