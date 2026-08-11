#pragma once
#include "type.h"

namespace VRT
{
    PointCloudData processDecodeData(const PulseData& pulse,const int gpsWeek, const int secondInWeek, const int timeRes);
}

namespace Decode
{
    void rw_probe_decode(const unsigned char* buffer, unsigned short* outdata);

    void reorder_to_colmajor(const unsigned short* data_in, unsigned short* data_out);

    void flip180_colmajor64x64(const unsigned short* data_in, unsigned short* data_out);

    void rw_probe_decode3(const unsigned char* buffer, unsigned short* outdata);
}