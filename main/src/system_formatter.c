#include "system_formatter.h"
#include <string.h>

uint8_t create_request_body(const char* keys[], const char* values[], uint8_t key_value_len,  char* out, size_t out_len) {
    memset(out, 0, out_len);

    snprintf(out, out_len, "{");

    for(size_t i = 0; i < key_value_len; i++) {
        strncat(out, "\"", out_len - strlen(out) - 1);
        strncat(out, keys[i], out_len - strlen(out) - 1);
        strncat(out, "\":\"", out_len - strlen(out) - 1);
        strncat(out, values[i], out_len - strlen(out) - 1);
        strncat(out, "\"", out_len - strlen(out) - 1);
        if(i < key_value_len - 1) {
            strncat(out, ",", out_len - strlen(out) - 1);
        }
    }

    strncat(out, "}", out_len - strlen(out) - 1);

    return 0;
}