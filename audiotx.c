/*
 * copyright (c) 2001 Fabrice Bellard
 *
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifdef HAVE_AV_CONFIG_H
#undef HAVE_AV_CONFIG_H
#endif


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
#include "audiotx.h"

typedef struct SampleFmtInfo {
char name[8];
int bits;
int planar;
enum AVSampleFormat altform; ///< planar<->packed alternative form
} SampleFmtInfo;

/** this table gives more information about formats */
static const SampleFmtInfo sample_fmt_info[AV_SAMPLE_FMT_NB] = {
[AV_SAMPLE_FMT_U8]   = { .name =   "u8", .bits =  8, .planar = 0, .altform = AV_SAMPLE_FMT_U8P  },
[AV_SAMPLE_FMT_S16]  = { .name =  "s16", .bits = 16, .planar = 0, .altform = AV_SAMPLE_FMT_S16P },
[AV_SAMPLE_FMT_S32]  = { .name =  "s32", .bits = 32, .planar = 0, .altform = AV_SAMPLE_FMT_S32P },
[AV_SAMPLE_FMT_FLT]  = { .name =  "flt", .bits = 32, .planar = 0, .altform = AV_SAMPLE_FMT_FLTP },
[AV_SAMPLE_FMT_DBL]  = { .name =  "dbl", .bits = 64, .planar = 0, .altform = AV_SAMPLE_FMT_DBLP },
[AV_SAMPLE_FMT_U8P]  = { .name =  "u8p", .bits =  8, .planar = 1, .altform = AV_SAMPLE_FMT_U8   },
[AV_SAMPLE_FMT_S16P] = { .name = "s16p", .bits = 16, .planar = 1, .altform = AV_SAMPLE_FMT_S16  },
[AV_SAMPLE_FMT_S32P] = { .name = "s32p", .bits = 32, .planar = 1, .altform = AV_SAMPLE_FMT_S32  },
[AV_SAMPLE_FMT_FLTP] = { .name = "fltp", .bits = 32, .planar = 1, .altform = AV_SAMPLE_FMT_FLT  },
[AV_SAMPLE_FMT_DBLP] = { .name = "dblp", .bits = 64, .planar = 1, .altform = AV_SAMPLE_FMT_DBL  },
};

aud_context audctx;

int audio_init(AVFormatContext *aud_stream)
{

   printf("In audio_init\n"); 
   audctx.output_codec_opened = 0; 
   audctx.decoded_frame = NULL;
   if (!audctx.decoded_frame) {
         if (!(audctx.decoded_frame = avcodec_alloc_frame())) {
             fprintf(stderr, "Could not allocate audio frame\n");
             exit(1);
         }
    }    

    audctx.audindex = av_find_best_stream(aud_stream, AVMEDIA_TYPE_AUDIO, -1, -1,
                NULL, 0);
    
    audctx.in_codec_context=malloc(sizeof(AVCodecContext)); 
    memcpy(audctx.in_codec_context,aud_stream->streams[audctx.audindex]->codec,sizeof(AVCodecContext));
    audctx.in_codec = avcodec_find_decoder(audctx.in_codec_context->codec_id);
    if (!avcodec_open2(audctx.in_codec_context, audctx.in_codec,NULL) < 0) {
        die("Could not open the in codec");
    }
  
    audctx.out_codec = avcodec_find_encoder(CODEC_ID_AAC);
    if (!audctx.out_codec) {
        fprintf(stderr, "out codec not found\n");
        exit(1);
    }

    audctx.out_codec_context =  avcodec_alloc_context3(audctx.out_codec);
    audctx.out_codec_context->channels       = audctx.in_codec_context->channels;
    audctx.out_codec_context->channel_layout = audctx.in_codec_context->channel_layout;
    audctx.out_codec_context->sample_rate    = audctx.in_codec_context->sample_rate;
    audctx.out_codec_context->sample_fmt     = audctx.in_codec_context->sample_fmt;
    audctx.out_codec_context->bit_rate       = 48000;   
    if (!avcodec_open2(audctx.out_codec_context, audctx.out_codec,NULL) < 0) {
	die("Could not open the out codec");
    }
    //setup fifo 
    audctx.bytes_per_sample = (audctx.out_codec_context->sample_fmt< 0 || audctx.out_codec_context->sample_fmt >= AV_SAMPLE_FMT_NB)?0:(sample_fmt_info[audctx.out_codec_context->sample_fmt].bits >> 3); 
    audctx.bytes_per_frame  = audctx.out_codec_context->frame_size * audctx.bytes_per_sample * audctx.out_codec_context->channels; 
    audctx.fifo = av_fifo_alloc(2 * 128*1024); 
    audctx.buffer = (uint8_t*) av_malloc(
			2 * 128*1024);
    audctx.temp_frame = avcodec_alloc_frame(); 
    avcodec_get_frame_defaults(audctx.temp_frame); 
    audctx.temp_frame->nb_samples = audctx.bytes_per_frame / (audctx.out_codec_context->channels * 
						audctx.bytes_per_sample);
    return audctx.audindex; 
}

int audio_transcode(AVPacket *pkt, AVPacket *out_pkt)
{
//decode
    int got_frame = 0;
    /* the codec gives us the frame size, in samples */
    int err= avcodec_decode_audio4(audctx.in_codec_context, audctx.decoded_frame, &got_frame, pkt);
    if (err < 0){
	printf ("ERROR decoding audio\n");
	return err;
    }
    printf("DECODED AUDIO FRAME got_frame %d\n",got_frame);
//encode
    uint8_t *buf = audctx.decoded_frame->data[0]; 
    int buf_size = audctx.decoded_frame->linesize[0];
    
    av_fifo_generic_write(audctx.fifo, buf, buf_size, NULL); 
    got_frame = 0;
    err = -1; 
    while (av_fifo_size(audctx.fifo) >= audctx.bytes_per_frame) {
        av_fifo_generic_read(audctx.fifo, audctx.buffer, audctx.bytes_per_frame, NULL);
	err  = avcodec_fill_audio_frame(audctx.temp_frame,audctx.out_codec_context->channels, 
                                 audctx.out_codec_context->sample_fmt, 
                                 audctx.buffer, audctx.bytes_per_frame, 1);

	err = avcodec_encode_audio2(audctx.out_codec_context, out_pkt, audctx.temp_frame, &got_frame);
        printf("encoded audio pkt size %d got frame %d\n", out_pkt->size, got_frame);
        if (err < 0){
		printf("ERROR encoding audio %d\n",err);
		return err;
	}
	printf("ENCODED AUDIO FRAME\n");
    }
    return err;
}
