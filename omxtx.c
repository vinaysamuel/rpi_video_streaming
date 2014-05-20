/* omxtx.c
 *
 * (c) 2012, 2013 Dickon Hood <dickon@fluff.org>
 * Timing fixups by Adam Charrett <adam@dvbstreamer.org>
 *
 * A trivial OpenMAX transcoder for the Pi.
 *
 * Very much a work-in-progress, and as such is noisy, doesn't produce
 * particularly pretty output, and is probably buggier than a swamp in
 * summer.  Beware of memory leaks.
 *
 * Usage: ./omxtx [-b bitrate] [-r size] [-x] [-d] [-m] input.foo output.mp4
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* To do:
 *
 *  *  Move to an event-driven setup, rather than waiting.
 */

#define _BSD_SOURCE
#define FF_API_CODEC_ID 1

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "bcm_host.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavformat/avio.h"
#include <error.h>

#include "OMX_Video.h"
#include "OMX_Types.h"
#include "OMX_Component.h"
#include "OMX_Core.h"
#include "OMX_Broadcom.h"

#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/queue.h>
#include <fcntl.h>

#include <time.h>
#include <errno.h>

#include <unistd.h>

/* For htonl(): */
#include <arpa/inet.h>
#include "audiotx.h"
extern int audio_transcode(AVPacket *in_pkt, AVPacket *out_pkt); 
extern int audio_init(AVFormatContext *aud_stream); 

static OMX_VERSIONTYPE SpecificationVersion = {
	.s.nVersionMajor = 1,
	.s.nVersionMinor = 1,
	.s.nRevision     = 2,
	.s.nStep         = 0
};

/* Hateful things: */
#define MAKEME(y, x)	do {						\
				y = calloc(1, sizeof(x));		\
				y->nSize = sizeof(x);			\
				y->nVersion = SpecificationVersion;	\
			} while (0)


#define OERR(cmd)	do {						\
				OMX_ERRORTYPE oerr = cmd;		\
				if (oerr != OMX_ErrorNone) {		\
					fprintf(stderr, #cmd		\
						" failed on line %d: %x\n", \
						__LINE__, oerr);	\
					exit(1);			\
				} else {				\
					fprintf(stderr, #cmd		\
						" completed at %d.\n",	\
						__LINE__);		\
				}					\
			} while (0)

#define OERRq(cmd)	do {	oerr = cmd;				\
				if (oerr != OMX_ErrorNone) {		\
					fprintf(stderr, #cmd		\
						" failed: %x\n", oerr);	\
					exit(1);			\
				}					\
			} while (0)
/* ... but damn useful.*/

#define V_ALWAYS	0
#define V_INFO		1
#define V_LOTS		2

/* Not vprintf(); there's already one of those in libc... */
#define logprintf(v, ...) \
    do { \
        if ((v) <= ctx.verbosity) { \
            printf( __VA_ARGS__); \
        } \
    } while (0)


/* Hardware component names: */
#define ENCNAME "OMX.broadcom.video_encode"
#define DECNAME "OMX.broadcom.video_decode"
#define RSZNAME "OMX.broadcom.resize"
#define VIDNAME "OMX.broadcom.video_render"
#define SPLNAME "OMX.broadcom.video_splitter"
#define DEINAME	"OMX.broadcom.image_fx"

/*
 * Portbase for the modules, could also be queried, but as the components
 * are broadcom/raspberry specific anyway...
 */
#define PORT_RSZ  60
#define PORT_VID  90
#define PORT_DEC 130
#define PORT_DEI 190
#define PORT_ENC 200
#define PORT_SPL 250

enum states {
	DECINIT,
	DECTUNNELSETUP,
	DECRUNNING,
	DECFLUSH,
	DECDONE,
	DECFAILED,
	ENCPREINIT,
	ENCINIT,
	ENCGOTBUF,
	ENCDONE,
	ENCSPSPPS,
	ENCRUNNING,
};



struct packetentry {
	TAILQ_ENTRY(packetentry) link;
	AVPacket packet;
};

TAILQ_HEAD(packetqueue, packetentry);

static struct packetqueue packetq;



static struct context {
	AVFormatContext *ic;
	AVFormatContext *oc;
	int		nextin;
	int		nextout;
	int		incount;
	int		outcount;
	volatile int	fps;
	volatile int	framecount;
	int		done;
	volatile int	flags;
	OMX_BUFFERHEADERTYPE *encbufs, *bufhead;
	volatile enum states	decstate;
	volatile enum states	encstate;
	int		vidindex;
	int		audindex;
	int		*outindex;
	OMX_HANDLETYPE	dec, enc, rsz, dei;
	pthread_mutex_t	filelock;
	pthread_mutex_t	lock;
	AVBitStreamFilterContext *bsfc;
	int		bitrate;
	char		*resize;
	char		*oname;
	double		frameduration;
	int		verbosity;
	int64_t		ptsoff;
	int		num_video_frames;
        int 		num_audio_frames;
} ctx;
#define FLAGS_VERBOSE		(1<<0)
#define FLAGS_DECEMPTIEDBUF	(1<<1)
#define FLAGS_MONITOR		(1<<2)
#define FLAGS_DEINTERLACE	(1<<3)
#define FLAGS_RAW		(1<<4)
#define FLAGS_NOSUBS		(1<<5)



static OMX_BUFFERHEADERTYPE *allocbufs(OMX_HANDLETYPE h, int port, int enable);


/* Print some useful information about the state of the port: */
static void dumpport(OMX_HANDLETYPE handle, int port)
{
	OMX_VIDEO_PORTDEFINITIONTYPE	*viddef;
	OMX_PARAM_PORTDEFINITIONTYPE	*portdef;

	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	portdef->nPortIndex = port;
	OERR(OMX_GetParameter(handle, OMX_IndexParamPortDefinition, portdef));

	printf("Port %d is %s, %s\n", portdef->nPortIndex,
		(portdef->eDir == 0 ? "input" : "output"),
		(portdef->bEnabled == 0 ? "disabled" : "enabled"));
	printf("Wants %d bufs, needs %d, size %d, enabled: %d, pop: %d, "
		"aligned %d\n", portdef->nBufferCountActual,
		portdef->nBufferCountMin, portdef->nBufferSize,
		portdef->bEnabled, portdef->bPopulated,
		portdef->nBufferAlignment);
	viddef = &portdef->format.video;

	switch (portdef->eDomain) {
	case OMX_PortDomainVideo:
		printf("Video type is currently:\n"
			"\tMIME:\t\t%s\n"
			"\tNative:\t\t%p\n"
			"\tWidth:\t\t%d\n"
			"\tHeight:\t\t%d\n"
			"\tStride:\t\t%d\n"
			"\tSliceHeight:\t%d\n"
			"\tBitrate:\t%d\n"
			"\tFramerate:\t%d (%x); (%f)\n"
			"\tError hiding:\t%d\n"
			"\tCodec:\t\t%d\n"
			"\tColour:\t\t%d\n",
			viddef->cMIMEType, viddef->pNativeRender,
			viddef->nFrameWidth, viddef->nFrameHeight,
			viddef->nStride, viddef->nSliceHeight,
			viddef->nBitrate,
			viddef->xFramerate, viddef->xFramerate,
			((float)viddef->xFramerate/(float)65536),
			viddef->bFlagErrorConcealment,
			viddef->eCompressionFormat, viddef->eColorFormat);
		break;
	case OMX_PortDomainImage:
		printf("Image type is currently:\n"
			"\tMIME:\t\t%s\n"
			"\tNative:\t\t%p\n"
			"\tWidth:\t\t%d\n"
			"\tHeight:\t\t%d\n"
			"\tStride:\t\t%d\n"
			"\tSliceHeight:\t%d\n"
			"\tError hiding:\t%d\n"
			"\tCodec:\t\t%d\n"
			"\tColour:\t\t%d\n",
			portdef->format.image.cMIMEType,
			portdef->format.image.pNativeRender,
			portdef->format.image.nFrameWidth,
			portdef->format.image.nFrameHeight,
			portdef->format.image.nStride,
			portdef->format.image.nSliceHeight,
			portdef->format.image.bFlagErrorConcealment,
			portdef->format.image.eCompressionFormat,
			portdef->format.image.eColorFormat); 		
		break;
/* Feel free to add others. */
	default:
		break;
	}

	free(portdef);
}



static int mapcodec(enum CodecID id)
{
	printf("Mapping codec ID %d (%x)\n", id, id);
	switch (id) {
		case	CODEC_ID_MPEG2VIDEO:
		case	CODEC_ID_MPEG2VIDEO_XVMC:
			return OMX_VIDEO_CodingMPEG2;
		case	CODEC_ID_H264:
			return OMX_VIDEO_CodingAVC;
		case	8:
			return OMX_VIDEO_CodingMJPEG;
		case	13:
			return OMX_VIDEO_CodingMPEG4;
		default:
			return -1;
	}

	return -1;
}



static void dumpportstate(void)
{
	enum OMX_STATETYPE		state;

	printf("\n\nIn exit handler, after %d frames:\n", ctx.framecount);
	dumpport(ctx.dec, PORT_DEC);
	dumpport(ctx.dec, PORT_DEC+1);
	dumpport(ctx.enc, PORT_ENC);
	dumpport(ctx.enc, PORT_ENC+1);

	OMX_GetState(ctx.dec, &state);
	printf("Decoder state: %d\n", state);
	OMX_GetState(ctx.enc, &state);
	printf("Encoder state: %d\n", state);
}



static const char *mapcomponent(struct context *ctx, OMX_HANDLETYPE h)
{
	if (h == ctx->dec)
		return "Decoder";
	if (h == ctx->enc)
		return "Encoder";
	if (h == ctx->rsz)
		return "Resizer";
	return "Unknown!";
}



static AVFormatContext *makeoutputcontext(AVFormatContext *ic,
	const char *oname, int vididx, int audidx, const OMX_PARAM_PORTDEFINITIONTYPE *prt)
{
	AVFormatContext			*oc;
	AVOutputFormat			*fmt;
	int				i;
	AVStream			*iflow, *oflow;
	AVCodec				*c;
	AVCodecContext			*cc;
	const OMX_VIDEO_PORTDEFINITIONTYPE	*viddef;
	int				streamindex = 0;

	viddef = &prt->format.video;

	fmt = av_guess_format(NULL, oname, NULL);
	if (!fmt) {
		fprintf(stderr, "Can't guess format for %s; defaulting to "
			"MPEG\n",
			oname);
		fmt = av_guess_format(NULL, "MPEG", NULL);
	}
	if (!fmt) {
		fprintf(stderr, "Failed even that.  Bye bye.\n");
		exit(1);
	}

	oc = avformat_alloc_context();
	if (!oc) {
		fprintf(stderr, "Failed to alloc outputcontext\n");
		exit(1);
	}
	oc->oformat = fmt;
	snprintf(oc->filename, sizeof(oc->filename), "%s", oname);
	oc->debug = 1;
	oc->start_time_realtime = ic->start_time;
	oc->start_time = ic->start_time;
	oc->duration = 0;
	ctx.outindex = calloc(ic->nb_streams, sizeof(int));
	oc->bit_rate = 0;

#define ETB(x) x.num, x.den
	for (i = 0; i < ic->nb_streams; i++) {
		iflow = ic->streams[i];
		if (i == vididx) {	/* My new H.264 stream. */
			c = avcodec_find_encoder(CODEC_ID_H264);
			ctx.outindex[i] = streamindex++;
printf("Found a codec at %p\n", c);
			oflow = avformat_new_stream(oc, c);
printf("Defaults: output stream: %d/%d, input stream: %d/%d, input codec: %d/%d, output codec: %d/%d, output framerate: %d/%d, input framerate: %d/%d, ticks: %d; %d %lld/%lld\n", ETB(oflow->time_base), ETB(iflow->time_base), ETB(iflow->codec->time_base), ETB(oflow->codec->time_base), ETB(oflow->r_frame_rate), ETB(iflow->r_frame_rate), oflow->codec->ticks_per_frame, iflow->codec->ticks_per_frame, oc->start_time_realtime, ic->start_time_realtime);
			cc = oflow->codec;
			cc->width = viddef->nFrameWidth;
			cc->height = viddef->nFrameHeight;
			cc->codec_id = CODEC_ID_H264;
			cc->codec_type = AVMEDIA_TYPE_VIDEO;
			cc->bit_rate = ctx.bitrate;
			cc->profile = FF_PROFILE_H264_HIGH;
			cc->level = 41;
			cc->time_base = iflow->codec->time_base;

			oflow->avg_frame_rate = iflow->avg_frame_rate;
			oflow->r_frame_rate = iflow->r_frame_rate;
			oflow->start_time = AV_NOPTS_VALUE;

			if (!ctx.resize) {
				cc->sample_aspect_ratio.num =
				    iflow->codec->sample_aspect_ratio.num;
				cc->sample_aspect_ratio.den =
				    iflow->codec->sample_aspect_ratio.den;
				oflow->sample_aspect_ratio.num =
				    iflow->codec->sample_aspect_ratio.num;
				oflow->sample_aspect_ratio.den =
				    iflow->codec->sample_aspect_ratio.den;
			}

printf("Defaults: output stream: %d/%d, input stream: %d/%d, input codec: %d/%d, output codec: %d/%d, output framerate: %d/%d, input framerate: %d/%d\n", ETB(oflow->time_base), ETB(iflow->time_base), ETB(iflow->codec->time_base), ETB(oflow->codec->time_base), ETB(oflow->r_frame_rate), ETB(iflow->r_frame_rate));
printf("Time base: %d/%d, fps %d/%d\n", oflow->time_base.num, oflow->time_base.den, oflow->r_frame_rate.num, oflow->r_frame_rate.den);
		} 
                else if (i==audidx){
			ctx.outindex[i] = streamindex++;
			oflow = avformat_new_stream(oc, audctx.out_codec);	
			cc = oflow->codec;
			cc->codec_id = CODEC_ID_AAC;
			cc->codec_type = AVMEDIA_TYPE_AUDIO;
			cc->channels = audctx.out_codec_context->channels;
			cc->channel_layout = audctx.out_codec_context->channel_layout;
			cc->sample_rate = audctx.out_codec_context->sample_rate;
			cc->sample_fmt = audctx.out_codec_context->sample_fmt;
			cc->bit_rate = audctx.out_codec_context->bit_rate;
		        cc->frame_size =  audctx.out_codec_context->frame_size; 
			printf("created new stream for audio\n");	
		}
		else { 	/* Something pre-existing. */
			if ((ctx.flags & FLAGS_NOSUBS) &&
				iflow->codec->codec_type !=
					AVMEDIA_TYPE_AUDIO) {
				ctx.outindex[i] = -1;
				continue;
			}
			ctx.outindex[i] = streamindex++;
			c = avcodec_find_encoder(iflow->codec->codec_id);
			oflow = avformat_new_stream(oc, c);
			avcodec_copy_context(oflow->codec, iflow->codec);
/* Apparently fixes a crash on .mkvs with attachments: */
			av_dict_copy(&oflow->metadata, iflow->metadata, 0);
/* Reset the codec tag so as not to cause problems with output format */
			oflow->codec->codec_tag = 0; 
		}
	}
	for (i = 0; i < oc->nb_streams; i++) {
		if (oc->oformat->flags & AVFMT_GLOBALHEADER)
			oc->streams[i]->codec->flags
				|= CODEC_FLAG_GLOBAL_HEADER;
		if (oc->streams[i]->codec->sample_rate == 0)
			oc->streams[i]->codec->sample_rate = 48000; /* ish */
	}

printf("\n\n\nInput:\n");
	av_dump_format(ic, 0, ic->filename, 0);
printf("\n\n\nOutput:\n");
	av_dump_format(oc, 0, oname, 1);

	return oc;
}



static void writenonvideopacket(AVPacket *in_pkt)
{
        int err;
	int index = in_pkt->stream_index;
	int outindex = ctx.outindex[index];
        AVPacket *pkt;
        AVPacket pkt_obj;

        if (index == ctx.audindex){
	        av_init_packet(&pkt_obj);
		pkt_obj.data = NULL;
		pkt_obj.size = 0;
		pkt_obj.stream_index = outindex;	
		err = audio_transcode(in_pkt,&pkt_obj); 
		if (err <0){
	        	outindex = -1;	
			printf("ERROR transcoding audio\n");	
			av_free_packet(&pkt_obj);
                        pkt = in_pkt;
		}
		else{	
			pkt = &pkt_obj; 
		}	
	}
	else{
		pkt = in_pkt;
        }
	if (outindex != -1) {
		pkt->stream_index = outindex;
		if (pkt->pts != AV_NOPTS_VALUE) {
			pkt->pts =
			    av_rescale_q(pkt->pts,
					 ctx.ic->streams[index]->time_base,
					 ctx.oc->streams[outindex]->time_base);
			pkt->pts -= ctx.ptsoff;
		}

		if (pkt->dts != AV_NOPTS_VALUE) {
			pkt->dts =
			    av_rescale_q(pkt->dts,
					 ctx.ic->streams[index]->time_base,
					 ctx.oc->streams[outindex]->time_base);
			pkt->dts -= ctx.ptsoff;
		}
		logprintf(V_LOTS, "Other PTS %lld\n", pkt->pts);
		int r = av_interleaved_write_frame(ctx.oc, pkt);
		if (r != 0) {
			char err[256];
			av_strerror(r, err, sizeof(err));
			printf("Failed to write an audio frame: %s (%lld, %llx) \n", err, pkt->pts, pkt->pts);
		} else if (index == ctx.audindex){
			ctx.num_audio_frames++;
                        printf("Wrote an audio frame #%d of size %d @index %d! %lld (%llx)\n", ctx.num_audio_frames,pkt->size,pkt->stream_index,pkt->pts, pkt->pts);
		}
	
		if (index==ctx.audindex)
			av_free_packet(pkt);
	} else {
		av_free_packet(pkt);
	}
	
}



static void openoutput(struct context *ctx)
{
	int i;
	int r;
	int outindex;
	struct packetentry *packet, *next;
	AVFormatContext *oc;
	char err[256];

	oc = ctx->oc;
/* At some point they changed the API: */
#ifndef URL_WRONLY
#define URL_WRONLY AVIO_FLAG_WRITE
#endif
	avio_open(&oc->pb, ctx->oname, URL_WRONLY);

	r = avformat_write_header(oc, NULL);
	if (r < 0) {
		av_strerror(r, err, sizeof(err));
		fprintf(stderr, "Failed to write header: %s\n", err);
		exit(1);
	}

	printf("Writing initial frame buffer contents out...");

	outindex = ctx->outindex[ctx->vidindex];
	ctx->ptsoff =
	    av_rescale_q(ctx->ic->start_time, AV_TIME_BASE_Q,
			 oc->streams[outindex]->time_base);

	for (i = 0, packet = TAILQ_FIRST(&packetq); packet; packet = next) {
		next = TAILQ_NEXT(packet, link);

		if (packet->packet.stream_index == ctx->vidindex) {
			av_free_packet(&packet->packet);
		} else {
			writenonvideopacket(&packet->packet);
			i++;
		}
		TAILQ_REMOVE(&packetq, packet, link);
		free(packet);
	}

	printf(" ...done.  Wrote %d frames.\n\n", i);

	return;
}



OMX_ERRORTYPE genericeventhandler(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_EVENTTYPE event,
				OMX_U32 data1,
				OMX_U32 data2,
				OMX_PTR eventdata)
{
	switch (event) {
	case OMX_EventError:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("%s %p has errored: %x\n", mapcomponent(ctx, component),
			component, data1);
		return data1;
		break;
	case OMX_EventCmdComplete:
	if (ctx->flags & FLAGS_VERBOSE)
		printf("%s %p has completed the last command.\n",
			mapcomponent(ctx, component), component);
		break;
	case OMX_EventPortSettingsChanged: {
//	if (ctx->flags & FLAGS_VERBOSE)
		printf("%s %p port %d settings changed.\n",
			mapcomponent(ctx, component), component, data1);
		dumpport(component, data1);
	}
		break;
	default:
		if (ctx->flags & FLAGS_VERBOSE)
			printf("Got an event of type %x on %s %p "
				"(d1: %x, d2 %x)\n", event,
				mapcomponent(ctx, component), component,
				data1, data2);
	}
	
	return OMX_ErrorNone;
}


OMX_ERRORTYPE deceventhandler(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_EVENTTYPE event, 
				OMX_U32 data1,
				OMX_U32 data2,
				OMX_PTR eventdata)
{
	if (event == OMX_EventPortSettingsChanged) {
		ctx->decstate = DECTUNNELSETUP;
	}
	return genericeventhandler(component, ctx, event, data1, data2,
		eventdata);
}



OMX_ERRORTYPE enceventhandler(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_EVENTTYPE event, 
				OMX_U32 data1,
				OMX_U32 data2,
				OMX_PTR eventdata)
{
	return genericeventhandler(component, ctx, event, data1, data2,
		eventdata);
}



OMX_ERRORTYPE rszeventhandler(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_EVENTTYPE event,
				OMX_U32 data1,
				OMX_U32 data2,
				OMX_PTR eventdata)
{
	return genericeventhandler(component, ctx, event, data1, data2,
		eventdata);
}



OMX_ERRORTYPE emptied(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_BUFFERHEADERTYPE *buf)
{
	if (ctx->flags & FLAGS_VERBOSE)
		printf("Got a buffer emptied event on %s %p, buf %p\n",
			mapcomponent(ctx, component), component, buf);
	buf->nFilledLen = 0;
	ctx->flags |= FLAGS_DECEMPTIEDBUF;
	return OMX_ErrorNone;
}



OMX_ERRORTYPE filled(OMX_HANDLETYPE component,
				struct context *ctx,
				OMX_BUFFERHEADERTYPE *buf)
{
	OMX_BUFFERHEADERTYPE *spare;

	if (ctx->flags & FLAGS_VERBOSE)
		printf("Got buffer %p filled (len %d)\n", buf,
			buf->nFilledLen);

/*
 * Don't call OMX_FillThisBuffer() here, as the hardware craps out after
 * a short while.  I don't know why.  Reentrancy, or the like, I suspect.
 * Queue the packet(s) and deal with them in main().
 *
 * It only ever seems to ask for the one buffer, but better safe than sorry...
 */

	pthread_mutex_lock(&ctx->lock);
	if (ctx->bufhead == NULL) {
		buf->pAppPrivate = NULL;
		ctx->bufhead = buf;
		pthread_mutex_unlock(&ctx->lock);
		return OMX_ErrorNone;
	}

	spare = ctx->bufhead;
	while (spare->pAppPrivate != NULL)
		spare = spare->pAppPrivate;

	spare->pAppPrivate = buf;
	buf->pAppPrivate = NULL;
	pthread_mutex_unlock(&ctx->lock);

	return OMX_ErrorNone;
}


OMX_CALLBACKTYPE encevents = {
	(void (*)) enceventhandler,
	(void (*)) emptied,
	(void (*)) filled
};

OMX_CALLBACKTYPE decevents = {
	(void (*)) deceventhandler,
	(void (*)) emptied,
	(void (*)) filled
};

OMX_CALLBACKTYPE rszevents = {
	(void (*)) rszeventhandler,
	(void (*)) emptied,
	(void (*)) filled
};

OMX_CALLBACKTYPE genevents = {
	(void (*)) genericeventhandler,
	(void (*)) emptied,
	(void (*)) filled
};



static void *fps(void *p)
{
	enum OMX_STATETYPE		state;
	int				lastframe;

	while (1) {
		lastframe = ctx.framecount;
		sleep(1);
		printf("Frame %6d (%5ds).  Frames last second: %d     \r",
			ctx.framecount, ctx.framecount/25,
				ctx.framecount-lastframe);
		fflush(stdout);
		if (0 && ctx.fps == 0) {
			printf("In fps thread, after %d frames:\n",
				ctx.framecount);
			dumpport(ctx.dec, PORT_DEC);
			dumpport(ctx.dec, PORT_DEC+1);
			dumpport(ctx.enc, PORT_ENC);
			dumpport(ctx.enc, PORT_ENC+1);

			OMX_GetState(ctx.dec, &state);
			printf("Decoder state: %d\n", state);
			OMX_GetState(ctx.enc, &state);
			printf("Encoder state: %d\n", state);
		}
	}
	return NULL;
}



static OMX_BUFFERHEADERTYPE *allocbufs(OMX_HANDLETYPE h, int port, int enable)
{
	int i;
	OMX_BUFFERHEADERTYPE *list = NULL, **end = &list;
	OMX_PARAM_PORTDEFINITIONTYPE *portdef;

	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	portdef->nPortIndex = port;
	OERR(OMX_GetParameter(h, OMX_IndexParamPortDefinition, portdef));

	if (enable)
		OERR(OMX_SendCommand(h, OMX_CommandPortEnable, port, NULL));

	for (i = 0; i < portdef->nBufferCountActual; i++) {
		OMX_U8 *buf;

		buf = vcos_malloc_aligned(portdef->nBufferSize,
			portdef->nBufferAlignment, "buffer");
		printf("Allocated a buffer of %d bytes\n",
			portdef->nBufferSize);
		OERR(OMX_UseBuffer(h, end, port, NULL, portdef->nBufferSize,
			buf));
		end = (OMX_BUFFERHEADERTYPE **) &((*end)->pAppPrivate);
	}

	free(portdef);

	return list;
}



static AVBitStreamFilterContext *dofiltertest(AVPacket *rp)
{
	AVBitStreamFilterContext *bsfc;
	bsfc = NULL;

	if (!(rp->data[0] == 0x00 && rp->data[1] == 0x00 &&
		rp->data[2] == 0x00 && rp->data[3] == 0x01)) {
		bsfc = av_bitstream_filter_init("h264_mp4toannexb");
		if (!bsfc) {
			printf("Failed to open filter.  This is bad.\n");
		} else {
			printf("Have a filter at %p\n", bsfc);
		}
	} else
		printf("No need for a filter.\n");

	return bsfc;
}



static AVPacket *filter(struct context *ctx, AVPacket *rp)
{
	AVPacket *p;
	AVPacket *fp;
	int rc;

	fp = calloc(1, sizeof(AVPacket));

	if (ctx->bsfc) {
		rc = av_bitstream_filter_filter(ctx->bsfc,
				ctx->ic->streams[ctx->vidindex]->codec,
				NULL, &(fp->data), &(fp->size),
				rp->data, rp->size,
				rp->flags & AV_PKT_FLAG_KEY);
		if (rc > 0) {
			av_free_packet(rp);
			fp->destruct = av_destruct_packet;
			p = fp;
		} else {
			char err[256];
#if 0
			printf("Failed to filter frame: "
				"%d (%x): %s\n", rc, rc,
				av_make_error_string(err, sizeof(err), rc));
#else
			printf("Failed to filter frame: "
				"%d (%x)\n", rc, rc);
#endif
			p = rp;
		}
	} else
		p = rp;

	return p;
}



static void configure(struct context *ctx)
{
	pthread_t			fpst;
	pthread_attr_t			fpsa;
	OMX_CONFIG_FRAMERATETYPE	*framerate;
	OMX_VIDEO_PARAM_PROFILELEVELTYPE *level;
	OMX_VIDEO_PARAM_BITRATETYPE	*bitrate;
	OMX_BUFFERHEADERTYPE		*encbufs;
	OMX_PARAM_PORTDEFINITIONTYPE	*portdef;
	OMX_VIDEO_PORTDEFINITIONTYPE	*viddef;
	OMX_PARAM_PORTDEFINITIONTYPE	*imgportdef;
	OMX_IMAGE_PORTDEFINITIONTYPE	*imgdef;
	OMX_VIDEO_PARAM_PORTFORMATTYPE	*pfmt;
	OMX_CONFIG_POINTTYPE		*pixaspect;
	OMX_HANDLETYPE			dec, enc, rsz, spl, vid, dei;
	OMX_HANDLETYPE			prev;
	int				pp;
	int				x, y;
	int				i;
/* omxplayer does this: */
	OMX_PARAM_U32TYPE		*extra_buffers;
	OMX_CONFIG_IMAGEFILTERPARAMSTYPE *image_filter;

	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	MAKEME(imgportdef, OMX_PARAM_PORTDEFINITIONTYPE);
	MAKEME(pixaspect, OMX_CONFIG_POINTTYPE);
	MAKEME(extra_buffers, OMX_PARAM_U32TYPE);
	MAKEME(image_filter, OMX_CONFIG_IMAGEFILTERPARAMSTYPE);

/* These just save a bit of typing.  No other reason. */
	dec = ctx->dec;
	enc = ctx->enc;
	rsz = ctx->rsz;
	dei = ctx->dei;
	viddef = &portdef->format.video;
	imgdef = &imgportdef->format.image;

	prev = dec;
	pp = PORT_DEC+1;

	printf("Decoder has changed settings.  Setting up encoder.\n");

	if (ctx->flags & FLAGS_MONITOR) {
		OERR(OMX_GetHandle(&spl, SPLNAME, &ctx, &genevents));
		OERR(OMX_GetHandle(&vid, VIDNAME, &ctx, &genevents));
		for (i = 0; i < 5; i++)
			OERR(OMX_SendCommand(spl, OMX_CommandPortDisable,
				PORT_SPL+i, NULL));
		OERR(OMX_SendCommand(vid, OMX_CommandPortDisable,
			PORT_VID, NULL));
	}

/* Get the decoder port state...: */
	portdef->nPortIndex = PORT_DEC+1;
	OERR(OMX_GetParameter(dec, OMX_IndexParamPortDefinition, portdef));

/* Deinterlacer: */
	if (ctx->flags & FLAGS_DEINTERLACE) {
		OERR(OMX_SendCommand(dei, OMX_CommandStateSet, OMX_StateIdle,
			NULL));
		portdef->nPortIndex = PORT_DEI;
		OERR(OMX_SendCommand(dei, OMX_CommandPortDisable,
				PORT_DEI, NULL));
		OERR(OMX_SendCommand(dei, OMX_CommandPortDisable,
				PORT_DEI+1, NULL));
		OERR(OMX_SetParameter(dei, OMX_IndexParamPortDefinition,
			portdef));
		portdef->nPortIndex = PORT_DEI+1;
		OERR(OMX_SetParameter(dei, OMX_IndexParamPortDefinition,
			portdef));

/* omxplayer does this: */
		extra_buffers->nU32 = 3;
		extra_buffers->nPortIndex = pp;
		OERR(OMX_SetParameter(prev, OMX_IndexParamBrcmExtraBuffers,
			extra_buffers));
		image_filter->nPortIndex = PORT_DEI+1;
		image_filter->nNumParams = 1;
		image_filter->nParams[0] = 3;
		image_filter->eImageFilter = OMX_ImageFilterDeInterlaceAdvanced;
		OERR(OMX_SetConfig(dei,
			OMX_IndexConfigCommonImageFilterParameters,
			image_filter));

		imgportdef->nPortIndex = PORT_DEI;
		OERR(OMX_GetParameter(dei, OMX_IndexParamPortDefinition,
			imgportdef));
		portdef->nPortIndex = PORT_ENC;
		viddef->nFrameWidth = imgdef->nFrameWidth;
		viddef->nFrameHeight = imgdef->nFrameHeight;
		viddef->nStride = imgdef->nStride;
		viddef->nSliceHeight = imgdef->nSliceHeight;
		viddef->bFlagErrorConcealment = imgdef->bFlagErrorConcealment;
		viddef->eCompressionFormat = imgdef->eCompressionFormat;
		viddef->eColorFormat = imgdef->eColorFormat;
		viddef->pNativeWindow = imgdef->pNativeWindow;

		dumpport(dei, 190);
		dumpport(dei, 191);
//		allocbufs(dei, PORT_DEI+1, 0);

		OERR(OMX_SetupTunnel(prev, pp, dei, PORT_DEI));
		prev = dei;
		pp = PORT_DEI + 1;
	}

	if (ctx->resize) {
		imgportdef->nPortIndex = PORT_RSZ;
		OERR(OMX_GetParameter(rsz, OMX_IndexParamPortDefinition,
			imgportdef));
/*
 * The least they could have done was to make *all* the common elements appear
 * in the same place in the structures, but no...
 */
		imgdef->nFrameWidth = viddef->nFrameWidth;
		imgdef->nFrameHeight = viddef->nFrameHeight;
		imgdef->nStride = viddef->nStride;
		imgdef->nSliceHeight = viddef->nSliceHeight;
		imgdef->bFlagErrorConcealment = viddef->bFlagErrorConcealment;
		imgdef->eCompressionFormat = viddef->eCompressionFormat;
		imgdef->eColorFormat = viddef->eColorFormat;
		imgdef->pNativeWindow = viddef->pNativeWindow;
		OERR(OMX_SetParameter(rsz, OMX_IndexParamPortDefinition,
			imgportdef));
		if (sscanf(ctx->resize, "%dx%d", &x, &y) == 2) {
			x += 0x0f;
			x &= ~0x0f;
			y += 0x0f;
			y &= ~0x0f;
			imgdef->nFrameWidth = x;
			imgdef->nFrameHeight = y;
		} else {
			imgdef->nFrameWidth *= x;
			imgdef->nFrameWidth /= 100;
			imgdef->nFrameWidth += 0x0f;
			imgdef->nFrameWidth &= ~0x0f;
			imgdef->nFrameHeight *= x;
			imgdef->nFrameHeight /= 100;
			imgdef->nFrameHeight += 0x0f;
			imgdef->nFrameHeight &= ~0x0f;
		}
		imgdef->nStride = 0;
		imgdef->nSliceHeight = 0;
		printf("Frame size: %dx%d, scale factor %d\n",
			imgdef->nFrameWidth, imgdef->nFrameHeight, x);
		imgportdef->nPortIndex = PORT_RSZ+1;
		OERR(OMX_SetParameter(rsz, OMX_IndexParamPortDefinition,
			imgportdef));
		usleep(40);
		OERR(OMX_GetParameter(rsz, OMX_IndexParamPortDefinition,
			imgportdef));
		portdef->nPortIndex = PORT_ENC;
		viddef->nFrameWidth = imgdef->nFrameWidth;
		viddef->nFrameHeight = imgdef->nFrameHeight;
		viddef->nStride = imgdef->nStride;
		viddef->nSliceHeight = imgdef->nSliceHeight;
		viddef->bFlagErrorConcealment = imgdef->bFlagErrorConcealment;
		viddef->eCompressionFormat = imgdef->eCompressionFormat;
		viddef->eColorFormat = imgdef->eColorFormat;
		viddef->pNativeWindow = imgdef->pNativeWindow;
	}

/* ... and feed it to the encoder: */
	portdef->nPortIndex = PORT_ENC;
	OERR(OMX_SetParameter(enc, OMX_IndexParamPortDefinition, portdef));

/* Setup the tunnel(s): */
	if (ctx->flags & FLAGS_MONITOR) {
		OERR(OMX_SendCommand(vid, OMX_CommandStateSet, OMX_StateIdle,
			NULL));
		OERR(OMX_SendCommand(spl, OMX_CommandStateSet, OMX_StateIdle,
			NULL));
		portdef->nPortIndex = PORT_VID;
//		OERR(OMX_SetParameter(vid, OMX_IndexParamPortDefinition, portdef));
dumpport(spl, PORT_SPL);
		portdef->nPortIndex = PORT_SPL;
		OERR(OMX_SetParameter(spl, OMX_IndexParamPortDefinition,
			portdef));
		portdef->nPortIndex = PORT_SPL+1;
		OERR(OMX_SetParameter(spl, OMX_IndexParamPortDefinition,
			portdef));
		portdef->nPortIndex = PORT_SPL+2;
		OERR(OMX_SetParameter(spl, OMX_IndexParamPortDefinition,
			portdef));
		OERR(OMX_SetupTunnel(prev, pp, spl, PORT_SPL));
		OERR(OMX_SetupTunnel(spl, PORT_SPL+2, vid, PORT_VID));
		prev = spl;
		pp = PORT_SPL+1;
	}
	if (ctx->resize) {
		OERR(OMX_SetupTunnel(prev, pp, rsz, PORT_RSZ));
		prev = rsz;
		pp = PORT_RSZ+1;
	}
	OERR(OMX_SendCommand(enc, OMX_CommandStateSet, OMX_StateIdle, NULL));
	dumpport(prev, pp);
	dumpport(enc, PORT_ENC);
	OERR(OMX_SetupTunnel(prev, pp, enc, PORT_ENC));

	if (ctx->bitrate == 0) {
		viddef->nBitrate *= 3;
		viddef->nBitrate /= 4;
	} else {
		if (ctx->bitrate == 0)
			viddef->nBitrate = (2*1024*1024);
		else
			viddef->nBitrate = ctx->bitrate;
		ctx->bitrate = viddef->nBitrate;
	}

	viddef->eCompressionFormat = OMX_VIDEO_CodingAVC;
	viddef->nStride = viddef->nSliceHeight = viddef->eColorFormat = 0;
	portdef->nPortIndex = PORT_ENC+1;
	OERR(OMX_SetParameter(enc, OMX_IndexParamPortDefinition, portdef));

	MAKEME(bitrate, OMX_VIDEO_PARAM_BITRATETYPE);
	bitrate->nPortIndex = PORT_ENC+1;
	bitrate->eControlRate = OMX_Video_ControlRateVariable;
	bitrate->nTargetBitrate = viddef->nBitrate;
	OERR(OMX_SetParameter(enc, OMX_IndexParamVideoBitrate, bitrate));

	MAKEME(pfmt, OMX_VIDEO_PARAM_PORTFORMATTYPE);
	pfmt->nPortIndex = PORT_ENC+1;
	pfmt->nIndex = 0;
	pfmt->eCompressionFormat = OMX_VIDEO_CodingAVC;
	pfmt->eColorFormat = OMX_COLOR_FormatYUV420PackedPlanar;
	pfmt->xFramerate = viddef->xFramerate;

	pixaspect->nPortIndex = PORT_ENC+1;
	pixaspect->nX = 64;
	pixaspect->nY = 45;
//	OERR(OMX_SetParameter(dec, OMX_IndexParamBrcmPixelAspectRatio, pixaspect));

//		DUMPPORT(enc, PORT_ENC+1); exit(0);

	pfmt->nPortIndex = PORT_ENC+1;
	pfmt->nIndex = 1;
	pfmt->eCompressionFormat = OMX_VIDEO_CodingAVC;
	pfmt->eColorFormat = 0;
	pfmt->xFramerate = 0; //viddef->xFramerate;
	OERR(OMX_SetParameter(enc, OMX_IndexParamVideoPortFormat,
		pfmt));

	MAKEME(framerate, OMX_CONFIG_FRAMERATETYPE);
	framerate->nPortIndex = PORT_ENC+1;
	framerate->xEncodeFramerate = viddef->xFramerate;
	OERR(OMX_SetParameter(enc, OMX_IndexConfigVideoFramerate, framerate));

#if 0 /* Doesn't seem to apply to video? */
printf("Interlacing: %d\n", ic->streams[vidindex]->codec->field_order);
	if (0 || ic->streams[vidindex]->codec->field_order == AV_FIELD_TT) {
		interlace->nPortIndex = PORT_ENC+1;
		interlace->eMode = OMX_InterlaceFieldsInterleavedUpperFirst;
		interlace->bRepeatFirstField = 0;
		OERR(OMX_SetParameter(enc, OMX_IndexConfigCommonInterlace,
			interlace));
	}
#endif

	MAKEME(level, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
	level->nPortIndex = PORT_ENC+1;
	OERR(OMX_GetParameter(enc, OMX_IndexParamVideoProfileLevelCurrent,
		level));
	printf("Current level:\t\t%d\nCurrent profile:\t%d\n",
		level->eLevel, level->eProfile);
	OERR(OMX_SetParameter(enc, OMX_IndexParamVideoProfileLevelCurrent,
		level));

	ctx->encbufs = encbufs = allocbufs(enc, PORT_ENC+1, 1);
	OERR(OMX_SendCommand(dec, OMX_CommandPortEnable, PORT_DEC+1, NULL));
	if (ctx->resize) {
		OERR(OMX_SendCommand(rsz, OMX_CommandPortEnable, PORT_RSZ,
			NULL));
		OERR(OMX_SendCommand(rsz, OMX_CommandPortEnable, PORT_RSZ+1,
			NULL));
	}

	OERR(OMX_SendCommand(enc, OMX_CommandPortEnable, PORT_ENC, NULL));

	if (ctx->flags & FLAGS_MONITOR) {
		OERR(OMX_SendCommand(vid, OMX_CommandPortEnable, PORT_VID,
			NULL));
		OERR(OMX_SendCommand(spl, OMX_CommandPortEnable, PORT_SPL,
			NULL));
		OERR(OMX_SendCommand(spl, OMX_CommandPortEnable, PORT_SPL+1,
			NULL));
		OERR(OMX_SendCommand(spl, OMX_CommandPortEnable, PORT_SPL+2,
			NULL));
// sleep(1); printf("\n\n\n\n\n"); for (i = 0; i < 5; i++) dumpport(spl, PORT_SPL+i); sleep(1);
printf("Pre-sleep.\n"); fflush(stdout);
		sleep(1);	/* Should probably wait for an event instead */
printf("Post-sleep.\n"); fflush(stdout);
		OERR(OMX_SendCommand(vid, OMX_CommandStateSet,
			OMX_StateExecuting, NULL));
		OERR(OMX_SendCommand(spl, OMX_CommandStateSet,
			OMX_StateExecuting, NULL));
	}

	if (ctx->flags & FLAGS_DEINTERLACE) {
		OERR(OMX_SendCommand(dei, OMX_CommandPortEnable, PORT_DEI,
			NULL));
		OERR(OMX_SendCommand(dei, OMX_CommandPortEnable, PORT_DEI+1,
			NULL));
/* This is another port-changed event I should wait for: */
		sleep(1);
		dumpport(dei, 190);
		dumpport(dei, 191);
		OERR(OMX_SendCommand(dei, OMX_CommandStateSet,
			OMX_StateExecuting, NULL));
	}

	if (ctx->resize)
		OERR(OMX_SendCommand(rsz, OMX_CommandStateSet,
			OMX_StateExecuting, NULL));
	OERR(OMX_SendCommand(enc, OMX_CommandStateSet,
		OMX_StateExecuting, NULL));
	sleep(1);
	OERR(OMX_FillThisBuffer(enc, encbufs));

/* Dump current port states: */
	dumpport(dec, PORT_DEC);
	dumpport(dec, PORT_DEC+1);
	if (ctx->resize) {
		dumpport(rsz, PORT_RSZ);
		dumpport(rsz, PORT_RSZ+1);
	}
	dumpport(enc, PORT_ENC);
	dumpport(enc, PORT_ENC+1);

/* Make an output context, possibly: */
	if ((ctx->flags & FLAGS_RAW) == 0) {
		ctx->oc = makeoutputcontext(ctx->ic, ctx->oname, ctx->vidindex,ctx->audindex,
			portdef);
		if (!ctx->oc) {
			fprintf(stderr, "Whoops.\n");
			exit(1);
		}
	}

	atexit(dumpportstate);
	pthread_attr_init(&fpsa);
	pthread_attr_setdetachstate(&fpsa, PTHREAD_CREATE_DETACHED);
	pthread_create(&fpst, &fpsa, fps, NULL);
}



static void usage(const char *name)
{
	fprintf(stderr, "Usage: %s [-b bitrate] [-d] [-m] [-r size] [-x] <infile> "
		"<outfile>\n\n"
		"Where:\n"
	"\t-b bitrate\tTarget bitrate in bits/second (default: 2Mb/s)\n"
	"\t-d\t\tDeinterlace\n"
	"\t-m\t\tMonitor.  Display the decoder's output\n"
	"\t-r size\t\tResize output.  'size' is either a percentage, or XXxYY\n"
	"\t-x\t\tRemove all non-AV streams from the output\n"
	"\n"
	"Output container is guessed based on filename.  Use '.nal' for raw"
	" output.\n"
	"\n"
	"Input file must contain one of MPEG 2, H.264, or MJPEG video\n"
	"\n", name);
	exit(1);
}



static int parsebitrate(const char *s)
{
	float rate;
	char specifier;
	int r;

	r = sscanf(s, "%f%c", &rate, &specifier);
	switch (r) {
	case 1:
		return (int)rate;
	case 2:
		switch (specifier) {
		case 'K':
		case 'k':
			return (int)(rate * 1024.0);
		case 'M':
		case 'm':
			return (int)(rate * 1024.0 * 1024.0);
		default:
			fprintf(stderr, "Unrecognised bitrate specifier!\n");
			exit(1);
			break;
		}
		break;
	default:
		fprintf(stderr, "Failed to parse bitrate!\n");
		exit(1);
		break;
	}
	return 0;
}



static void freepacket(AVPacket *p)
{
	if (p->data)
		free(p->data);
	p->data = NULL;
	p->destruct = av_destruct_packet;
	av_free_packet(p);
}



static int getnextvideopacket(AVPacket *pkt)
{
	int rc;
	do {
		rc = av_read_frame(ctx.ic, pkt);
		if (rc == 0) {
			if (pkt->stream_index != ctx.vidindex) {

				if (ctx.decstate == ENCRUNNING) {
					writenonvideopacket(pkt);
				} else if ((ctx.flags & FLAGS_RAW) == 0) {
					/*
					   Save packet for when we open the output file 
					 */
					struct packetentry *entry;
					entry =
					    malloc(sizeof(struct packetentry));
					entry->packet = *pkt;
					TAILQ_INSERT_TAIL(&packetq, entry,
							  link);
					/*
					   We've copied out the contents of the packet so re-initialise 
					 */
					av_init_packet(pkt);
				} else {
					av_free_packet(pkt);
				}
			}
		}
	} while ((rc == 0) && (pkt->stream_index != ctx.vidindex));
	return rc;
}




int main(int argc, char *argv[])
{
	AVFormatContext	*ic;
	AVFormatContext	*oc;
	char		*iname;
	char		*oname;
	int		err;
	int		vidindex;
	int		i, j;
	OMX_ERRORTYPE	oerr;
	OMX_HANDLETYPE	dec = NULL, enc = NULL, rsz = NULL, dei = NULL;
	OMX_VIDEO_PARAM_PORTFORMATTYPE	*pfmt;
	OMX_PORT_PARAM_TYPE		*porttype;
	OMX_PARAM_PORTDEFINITIONTYPE	*portdef;
	OMX_BUFFERHEADERTYPE		*decbufs;
	OMX_VIDEO_PORTDEFINITIONTYPE	*viddef;
	OMX_VIDEO_PARAM_PROFILELEVELTYPE *level;
	time_t		start, end;
	int		offset;
	AVPacket	*p, *rp;
	int		ish264;
	int		filtertest;
	int		opt;
	uint8_t		*tmpbuf;
	off_t		tmpbufoff;
	uint8_t		*sps, *pps;
	int		spssize, ppssize;
	int		fd;

	if (argc < 3)
		usage(argv[0]);

	ctx.bitrate = 2*1024*1024;
	TAILQ_INIT(&packetq);

	while ((opt = getopt(argc, argv, "b:dmxr:v")) != -1) {
		switch (opt) {
		case 'b':
			ctx.bitrate = parsebitrate(optarg);
			break;
		case 'd':
			ctx.flags |= FLAGS_DEINTERLACE;
			break;
		case 'm':
			ctx.flags |= FLAGS_MONITOR;
			break;
		case 'r':
			ctx.resize = optarg;
			break;
		case 'v':
			ctx.verbosity++;
			break;
		case 'x':
			ctx.flags |= FLAGS_NOSUBS;
			break;
		case '?':
			usage(argv[0]);
		}
	}

	iname = argv[optind++];
	oname = argv[optind++];
	ctx.oname = oname;
	j = strlen(oname);
	if (strncmp(&oname[j-4], ".nal", 4) == 0 ||
		strncmp(&oname[j-4], ".264", 4) == 0) {
		ctx.flags |= FLAGS_RAW;
		fd = open(oname, O_CREAT|O_TRUNC|O_WRONLY, 0666);
		if (fd == -1) {
			fprintf(stderr,
				"Failed to open the output: %s\n",
				strerror(errno));
			exit(1);
		}
	}

	MAKEME(porttype, OMX_PORT_PARAM_TYPE);
	MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
	MAKEME(pfmt, OMX_VIDEO_PARAM_PORTFORMATTYPE);

	av_register_all();

	ic = NULL;
	ish264 = 0;
	pthread_mutex_init(&ctx.lock, NULL);
	pthread_mutex_init(&ctx.filelock, NULL);
        ctx.num_audio_frames = 0;
        ctx.num_video_frames = 0;
	sps = pps = NULL;
	spssize = ppssize = 0;

#if 0
	{
		AVOutputFormat *fmt;
		fmt = av_oformat_next(fmt);
		while (fmt) {
			printf("Found '%s'\t\t'%s'\n", fmt->name, fmt->long_name);
			fmt = av_oformat_next(fmt);
		}
		exit(0);
	}
#endif

	/* Input init: */

	if ((err = avformat_open_input(&ic, iname, NULL, NULL) != 0)) {
		fprintf(stderr, "Failed to open '%s': %s\n", iname,
			strerror(err));
		exit(1);
	}
	ctx.ic = ic;
	if (avformat_find_stream_info(ic, NULL) < 0) {
		fprintf(stderr, "Failed to find streams in '%s'\n", iname);
		exit(1);
	}

	av_dump_format(ic, 0, iname, 0);

	vidindex = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, -1, -1,
		NULL, 0);
	if (vidindex < 0) {
		fprintf(stderr, "Failed to find a video stream in '%s'\n",
			iname);
		exit(1);
	}
	printf("Found a video at index %d\n", vidindex);

	printf("Frame size: %dx%d\n", ic->streams[vidindex]->codec->width, 
		ic->streams[vidindex]->codec->height);
	ish264 = (ic->streams[vidindex]->codec->codec_id == CODEC_ID_H264);
	

	/* Output init: */
#if 0
	ctx.fd = fd = open(oname, O_CREAT | O_LARGEFILE | O_WRONLY | O_TRUNC,
			0666);
	printf("File descriptor %d\n", fd);
#endif

	for (i = 0; i < ic->nb_streams; i++) {
		printf("Found stream %d, context %p\n",
			ic->streams[i]->index, ic->streams[i]->codec);
	}

	bcm_host_init();
	OERR(OMX_Init());
	OERR(OMX_GetHandle(&dec, DECNAME, &ctx, &decevents));
	OERR(OMX_GetHandle(&enc, ENCNAME, &ctx, &encevents));
	OERR(OMX_GetHandle(&rsz, RSZNAME, &ctx, &rszevents));
	OERR(OMX_GetHandle(&dei, DEINAME, &ctx, &genevents));
	ctx.dec = dec;
	ctx.enc = enc;
	ctx.rsz = rsz;
	ctx.dei = dei;

	printf("Obtained handles.  %p decode, %p encode\n",
		dec, enc);

	OERR(OMX_GetParameter(dec, OMX_IndexParamVideoInit, porttype));
	printf("Found %d ports, starting at %d (%x) on decoder\n",
		porttype->nPorts, porttype->nStartPortNumber,
		porttype->nStartPortNumber);

	OERR(OMX_GetParameter(enc, OMX_IndexParamVideoInit, porttype));
	printf("Found %d ports, starting at %d (%x) on encoder\n",
		porttype->nPorts, porttype->nStartPortNumber,
		porttype->nStartPortNumber);

	OERR(OMX_GetParameter(rsz, OMX_IndexParamImageInit, porttype));
	printf("Found %d ports, starting at %d(%x) on resizer\n",
		porttype->nPorts, porttype->nStartPortNumber,
		porttype->nStartPortNumber);

	OERR(OMX_SendCommand(dec, OMX_CommandPortDisable, PORT_DEC, NULL));
	OERR(OMX_SendCommand(dec, OMX_CommandPortDisable, PORT_DEC+1, NULL));
	OERR(OMX_SendCommand(enc, OMX_CommandPortDisable, PORT_ENC, NULL));
	OERR(OMX_SendCommand(enc, OMX_CommandPortDisable, PORT_ENC+1, NULL));
	if (ctx.resize) {
		OERR(OMX_SendCommand(rsz, OMX_CommandPortDisable, PORT_RSZ,
			NULL));
		OERR(OMX_SendCommand(rsz, OMX_CommandPortDisable, PORT_RSZ+1,
			NULL));
	}

	portdef->nPortIndex = PORT_DEC;
	OERR(OMX_GetParameter(dec, OMX_IndexParamPortDefinition, portdef));
	viddef = &portdef->format.video;
	viddef->nFrameWidth = ic->streams[vidindex]->codec->width;
	viddef->nFrameHeight = ic->streams[vidindex]->codec->height;
	printf("Mapping codec %d to %d\n",
		ic->streams[vidindex]->codec->codec_id,
		mapcodec(ic->streams[vidindex]->codec->codec_id));
	viddef->eCompressionFormat = 
		mapcodec(ic->streams[vidindex]->codec->codec_id);
	viddef->bFlagErrorConcealment = 0;
//	viddef->xFramerate = 25<<16;
	OERR(OMX_SetParameter(dec, OMX_IndexParamPortDefinition, portdef));

#if 0
/* It appears these have limited effect: */
	dataunit->nPortIndex = PORT_DEC;
	dataunit->eUnitType = OMX_DataUnitCodedPicture;
	dataunit->eEncapsulationType = OMX_DataEncapsulationGenericPayload;
	OERR(OMX_SetParameter(dec, OMX_IndexParamBrcmDataUnit, dataunit));

	if (ish264) {
		naltype->nPortIndex = PORT_DEC;
		naltype->eNaluFormat = OMX_NaluFormatStartCodes;
		OERR(OMX_SetParameter(dec, OMX_IndexParamNalStreamFormatSelect,
			naltype));
	}
#endif

	MAKEME(level, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
	level->nPortIndex = PORT_ENC+1;
/* Dump what the encoder is capable of: */
	for (oerr = OMX_ErrorNone, i = 0; oerr == OMX_ErrorNone; i++) {
		pfmt->nIndex = i;
		oerr = OMX_GetParameter(enc, OMX_IndexParamVideoPortFormat,
			pfmt);
		if (oerr == OMX_ErrorNoMore)
			break;
		printf("Codecs supported:\n"
			"\tIndex:\t\t%d\n"
			"\tCodec:\t\t%d (%x)\n"
			"\tColour:\t\t%d\n"
			"\tFramerate:\t%x (%f)\n",
			pfmt->nIndex,
			pfmt->eCompressionFormat, pfmt->eCompressionFormat,
			pfmt->eColorFormat,
			pfmt->xFramerate,
			((float)pfmt->xFramerate/(float)65536));
	}

	for (oerr = OMX_ErrorNone, i = 0; oerr == OMX_ErrorNone; i++) {
		level->nProfileIndex = i;
		oerr = OMX_GetParameter(enc,
			OMX_IndexParamVideoProfileLevelQuerySupported,
			level);
		if (oerr == OMX_ErrorNoMore)
			break;
		printf("Levels supported:\n"
			"\tIndex:\t\t%d\n"
			"\tProfile:\t%d\n"
			"\tLevel:\t\t%d\n",
			level->nProfileIndex,
			level->eProfile,
			level->eLevel);
	}

/* Dump current port states: */
	dumpport(dec, PORT_DEC);
	dumpport(dec, PORT_DEC+1);
	if (ctx.resize) {
		dumpport(rsz, PORT_RSZ);
		dumpport(rsz, PORT_RSZ+1);
	}
	dumpport(enc, PORT_ENC);
	dumpport(enc, PORT_ENC+1);

	OERR(OMX_SendCommand(dec, OMX_CommandStateSet, OMX_StateIdle, NULL));
	if (ctx.resize)
		OERR(OMX_SendCommand(rsz, OMX_CommandStateSet, OMX_StateIdle,
			NULL));

	decbufs = allocbufs(dec, PORT_DEC, 1);

/* Start the initial loop.  Process until we have a state change on port 131 */
	ctx.decstate = DECINIT;
	ctx.encstate = ENCPREINIT;
	OERR(OMX_SendCommand(dec, OMX_CommandStateSet, OMX_StateExecuting,
		NULL));

	rp = calloc(1, sizeof(AVPacket));
	filtertest = ish264;
	tmpbufoff = 0;
	tmpbuf = NULL;
        ctx.audindex = audio_init(ic);
	for (offset = i = j = 0; ctx.decstate != DECFAILED; i++, j++) {
		int rc;
		int k;
		int size, nsize;
		int outindex;
		OMX_BUFFERHEADERTYPE *spare;
		AVRational omxtimebase = { 1, 1000000 };
		OMX_TICKS tick;

		if (offset == 0 && ctx.decstate != DECFLUSH) {
			uint64_t omt;
			rc = getnextvideopacket(rp);
			if (rc != 0) {
				if (ic->pb->eof_reached)
					ctx.decstate = DECFLUSH;
				break;
			}
			outindex = rp->stream_index;

			omt = av_rescale_q(rp->pts,
				ic->streams[vidindex]->time_base,
				omxtimebase);
			tick.nLowPart = (uint32_t) (omt & 0xffffffff);
			tick.nHighPart = (uint32_t)
					((omt & 0xffffffff00000000) >> 32);

			size = rp->size;
			ctx.fps++;
			ctx.framecount++;

			if (ish264 && filtertest) {
				filtertest = 0;
				ctx.bsfc = dofiltertest(rp);
			}
			if (ctx.bsfc) {
				p = filter(&ctx, rp);
			} else {
				p = rp;
			}
		}

		switch (ctx.decstate) {
		case DECTUNNELSETUP:
			start = time(NULL);
			configure(&ctx);
			oc = ctx.oc;
			ctx.decstate = ENCSPSPPS;
			break;
		case DECFLUSH:
			size = 0;
			/* Add the flush code here */
			break;
		case DECINIT:
			if (i < 120) /* Bail; decoder doesn't like it */
				break;
			ctx.decstate = DECFAILED;
			/* Drop through */
		case DECFAILED:
			fprintf(stderr, "Failed to set the parameters after "
					"%d video frames.  Giving up.\n", i);
			dumpport(dec, PORT_DEC);
			dumpport(dec, PORT_DEC+1);
			dumpport(enc, PORT_ENC);
			dumpport(enc, PORT_ENC+1);
			exit(1);
			break;
		default:
			break;	/* Shuts the compiler up */
		}

		for (spare = NULL; !spare; usleep(10)) {
			pthread_mutex_lock(&ctx.lock);
			spare = ctx.bufhead;
			ctx.bufhead = NULL;
			ctx.flags &= ~FLAGS_DECEMPTIEDBUF;
			pthread_mutex_unlock(&ctx.lock);
			while (spare) {
				AVPacket pkt;
				int r;
				OMX_TICKS tick = spare->nTimeStamp;
				int writepkt = (pps && sps);

				if (ctx.flags & FLAGS_RAW) {
					write(fd,
						&spare->pBuffer[spare->nOffset],
						spare->nFilledLen);
					spare->nFilledLen = 0;
					spare->nOffset = 0;
					OERRq(OMX_FillThisBuffer(enc, spare));
					spare = spare->pAppPrivate;
					continue;
				}

				if ((spare->nFlags & OMX_BUFFERFLAG_ENDOFNAL)
					== 0) {
					if (!tmpbuf)
						tmpbuf = malloc(1024*1024*1);
					memcpy(&tmpbuf[tmpbufoff],
						&spare->pBuffer[spare->nOffset],
						spare->nFilledLen);
					tmpbufoff += spare->nFilledLen;
					spare->nFilledLen = 0;
					spare->nOffset = 0;
					OERRq(OMX_FillThisBuffer(enc, spare));
					spare = spare->pAppPrivate;
					continue;
				}

				av_init_packet(&pkt);
				outindex = ctx.outindex[vidindex];
				pkt.stream_index = outindex;
				if (tmpbufoff) {
					memcpy(&tmpbuf[tmpbufoff],
						&spare->pBuffer[spare->nOffset],
						spare->nFilledLen);
					tmpbufoff += spare->nFilledLen;
					pkt.data = tmpbuf;
					pkt.size = tmpbufoff;
					tmpbufoff = 0;
					tmpbuf = NULL;
				} else {
					pkt.data = malloc(spare->nFilledLen);
					memcpy(pkt.data, spare->pBuffer +
						spare->nOffset,
						spare->nFilledLen);
					pkt.size = spare->nFilledLen;
				}
				pkt.destruct = freepacket;
				if (spare->nFlags & OMX_BUFFERFLAG_SYNCFRAME)
					pkt.flags |= AV_PKT_FLAG_KEY;
				pkt.pts = av_rescale_q(((((uint64_t)
					tick.nHighPart)<<32)
					| tick.nLowPart), omxtimebase,
					oc->streams[outindex]->time_base);
				if (pkt.pts != 0)
					pkt.pts -= ctx.ptsoff;

				pkt.dts = AV_NOPTS_VALUE; // dts;

				if (pkt.data[0] == 0 && pkt.data[1] == 0 &&
					pkt.data[2] == 0 && pkt.data[3] == 1) {
					int nt = pkt.data[4] & 0x1f;
					if (nt == 7) {
						if (sps)
							free(sps);
						sps = malloc(pkt.size);
						memcpy(sps, pkt.data,
							pkt.size);
						spssize = pkt.size;
						printf("New SPS, length %d\n", spssize);
						writepkt = false;
					} else if (nt == 8) {
						if (pps)
							free(pps);
						pps = malloc(pkt.size);
						memcpy(pps, pkt.data,
							pkt.size);
						ppssize = pkt.size;
						printf("New PPS, length %d\n", ppssize);
						writepkt = false;
					}
					if (nt == 7 || nt == 8) {
						AVCodecContext *c;
						c = oc->streams[outindex]->codec;
						if (c->extradata) {
							av_free(c->extradata);
							c->extradata = NULL;
							c->extradata_size = 0;
						}
						if (pps && sps) {
							c->extradata_size =
								ppssize +
								spssize;
							c->extradata = malloc(
								ppssize+spssize);
							memcpy(c->extradata,
							       sps, spssize);
							memcpy(&c->
							       extradata
							       [spssize], pps,
							       ppssize);
							openoutput(&ctx);
							ctx.decstate = ENCRUNNING;
						}
					}
				}

				if (writepkt) {
					r = av_interleaved_write_frame(ctx.oc, &pkt);
					if (r != 0) {
						char err[256];
						av_strerror(r, err, sizeof(err));
						printf("Failed to write a video frame: %s (%lld, %llx; %d %d) %x.\n", err, pkt.pts, pkt.pts, tick.nLowPart, tick.nHighPart, spare->nFlags);
					} else {
						ctx.num_video_frames++;
						printf("Wrote a video frame # %d! %lld (%llx)\n", ctx.num_video_frames,pkt.pts, pkt.pts);
						av_free_packet(&pkt);
					}
				}
				spare->nFilledLen = 0;
				spare->nOffset = 0;
				OERRq(OMX_FillThisBuffer(enc, spare));
				spare = spare->pAppPrivate;
			}

			spare = decbufs;
			for (k = 0; spare && spare->nFilledLen != 0; k++)
				spare = spare->pAppPrivate;
		}

		if (size > spare->nAllocLen) {
			nsize = spare->nAllocLen;
		} else {
			nsize = size;
		}

		if (ctx.decstate != DECFLUSH) {
			memcpy(spare->pBuffer, &(p->data[offset]), nsize);
			spare->nFlags = i == 0 ? OMX_BUFFERFLAG_STARTTIME : 0;
			spare->nFlags |= size == nsize ?
				OMX_BUFFERFLAG_ENDOFFRAME : 0;
		} else {
			spare->nFlags = OMX_BUFFERFLAG_STARTTIME |
					OMX_BUFFERFLAG_EOS;
		}
		if (p->flags & AV_PKT_FLAG_KEY)
			spare->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;
		spare->nTimeStamp = tick;
		spare->nFilledLen = nsize;
		spare->nOffset = 0;
		OERRq(OMX_EmptyThisBuffer(dec, spare));
		size -= nsize;
		if (size) {
			offset += nsize;
		} else {
			offset = 0;
//			printf("PTS: %lld (%llx)\n", p->pts, p->pts);
			av_free_packet(p);
		}
	}

	end = time(NULL);

	printf("Processed %d frames in %d seconds; %df/s\n\n\n",
		ctx.framecount, end-start, (ctx.framecount/(end-start)));

	if (oc) {
		av_write_trailer(oc);

		for (i = 0; i < oc->nb_streams; i++)
			avcodec_close(oc->streams[i]->codec);

		avio_close(oc->pb);
	} else {
		close(fd);
	}

	return 0;
}
