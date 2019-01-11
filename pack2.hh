#pragma once

unsigned int pack(unsigned char *buf, const char *format, ...);
void unpack(unsigned char *buf, const char *format, ...);
