#ifndef __AUDIOTX_H__
#define __AUDIOTX_H__

#include "libavutil/opt.h"
#include "libavutil/common.h"
#include "libavutil/avutil.h"
#include "libavutil/imgutils.h"
#include "libavformat/avformat.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"
#include "libavformat/avio.h"
#include "libavutil/opt.h"
#include "libavutil/fifo.h"


typedef struct aud_info{
    int audindex;
    AVCodecContext *in_codec_context;
    AVCodec *in_codec;
    AVCodecContext *out_codec_context;
    AVCodec *out_codec;
    AVFrame *decoded_frame;
    int output_codec_opened;
    int bytes_per_sample;
    int bytes_per_frame;
    AVFifoBuffer* fifo;
    uint8_t *buffer;
    AVFrame *temp_frame;
} aud_context;

extern aud_context audctx;
#endif
