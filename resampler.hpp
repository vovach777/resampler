#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <cmath>
#include <utility>
#include <cassert>
#include <algorithm>
#include <vector>

inline float linear(float x, float width, float x0, float x1)
{

    return (x1-x0) / width * x + x0;

}


class LowPassFilter {
public:
    static constexpr float calculate_alpha( float sampleRate, float cutoffFrequency)
    {
        float RC = float(1.0 / (cutoffFrequency * 2 * 3.14159265358979323846));
        float dt = 1.0f / sampleRate;
        return dt / (RC + dt);
    }

    LowPassFilter(float alpha) : alpha(alpha), previousOutput(0.0) {}
    LowPassFilter(float sampleRate, float cutoffFrequency) : LowPassFilter(LowPassFilter::calculate_alpha(sampleRate,cutoffFrequency)) {}

    float process(float input) {
        float output = alpha * input + (1 - alpha) * previousOutput;
        previousOutput = output;
        return output;
    }

private:
    float alpha;
    float previousOutput;
};

class Resampler {
public:
    Resampler() {}
    Resampler(int src_rate, int dst_rate) : src_rate(src_rate), dst_rate(dst_rate), lpf(LowPassFilter::calculate_alpha(src_rate,dst_rate/2)) {

        if (  dst_rate > src_rate )
        {

            upscale_factor = dst_rate * 2 / src_rate;
            lpf = LowPassFilter(LowPassFilter::calculate_alpha(src_rate*upscale_factor,dst_rate/2));

        } else {
            upscale_factor = 1;
        }
     }


    size_t set_input( float sample ) {


        if (dst_rate == src_rate && upscale_factor == 1) {
            output.push_back(sample);
            dst_time++;
            src_time++;
            return output.size();
        }

        const auto cur_sample = sample;
        for ( int i = 0; i < upscale_factor; ++i)
        {

            if (upscale_factor > 1)   {
                sample = linear(i,upscale_factor, prev_sample, cur_sample);
            }
            src_time++;

            while ( dst_time * (src_rate*upscale_factor) / dst_rate   <  src_time  ) {
                output.push_back( lpf.process(sample) );
                dst_time++;
            }

        }
        prev_sample = sample;

        return output.size();
    }

    std::vector<float> get_output() {
        std::vector<float> out;
        out.swap(output);
        return out;
    }

    inline int get_source_sr() {
        return src_rate;
    }
    inline int get_dest_sr() {
        return dst_rate;
    }
private:
    int src_rate{44100};
    int dst_rate{44100};
    uint64_t dst_time{0};
    uint64_t src_time{0};
    float prev_sample{0};
    int upscale_factor{0};
    std::vector<float> output;
    LowPassFilter lpf{1};
};
