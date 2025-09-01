#ifndef SYSTEM_FORMATTER_H_
#define SYSTEM_FORMATTER_H_

uint8_t create_request_body(const char* keys[], const char* values[], uint8_t key_value_len,  char* out, size_t out_len);

#endif