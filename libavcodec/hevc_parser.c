/*
 * HEVC Annex B format parser
 *
 * Copyright (C) 2012 - 2013 Guillaume Martres
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

#include "libavutil/common.h"

#include "parser.h"
#include "hevc.h"
#include "golomb.h"

#define START_CODE 0x000001 ///< start_code_prefix_one_3bytes

typedef struct HEVCParseContext {
    HEVCContext  h;
    ParseContext pc;
    uint8_t *extradata;
    int extradata_size;
} HEVCParseContext;

/**
 * Find the end of the current frame in the bitstream.
 * @return the position of the first byte of the next frame, or END_NOT_FOUND
 */
static int hevc_find_frame_end(AVCodecParserContext *s, const uint8_t *buf,
                               int buf_size)
{
    int i, start;
    HEVCContext  *h  = &((HEVCParseContext *)s->priv_data)->h;
    ParseContext *pc = &((HEVCParseContext *)s->priv_data)->pc;

    for (i = 0; i < buf_size; i++) {
        int nut;

        if (h->is_nalff) {
            int j;
            int nal_length = 0;

            if (i + h->nal_length_size >= buf_size)
                return END_NOT_FOUND;

            start = i;
            for (j = 0; j < h->nal_length_size; j++)
                nal_length = (nal_length << 8) | buf[i++];

            if (i + nal_length > buf_size)
                return END_NOT_FOUND;

            nut = (buf[i] >> 1) & 0x3F;
            i += nal_length - 1;
        } else {
            pc->state64 = (pc->state64 << 8) | buf[i];

            if (((pc->state64 >> 3 * 8) & 0xFFFFFF) != START_CODE)
                continue;

            nut = (pc->state64 >> 2 * 8 + 1) & 0x3F;
            start = i - 5;
        }

        // Beginning of access unit
        if ((nut >= NAL_VPS && nut <= NAL_AUD) || nut == NAL_SEI_PREFIX ||
            (nut >= 41 && nut <= 44) || (nut >= 48 && nut <= 55)) {
            if (pc->frame_start_found) {
                pc->frame_start_found = 0;
                return start;
            }
        } else if (nut <= NAL_RASL_R ||
                   (nut >= NAL_BLA_W_LP && nut <= NAL_CRA_NUT)) {
            int first_slice_segment_in_pic_flag = buf[h->is_nalff ? start + h->nal_length_size + 2 : i] >> 7;
            if (first_slice_segment_in_pic_flag) {
                if (!pc->frame_start_found) {
                    pc->frame_start_found = 1;
                } else { // First slice of next frame found
                    pc->frame_start_found = 0;
                    return start;
                }
            }
        }
    }

    return END_NOT_FOUND;
}

/**
 * Parse NAL units of found picture and decode some basic information.
 *
 * @param s parser context.
 * @param avctx codec context.
 * @param buf buffer with field/frame data.
 * @param buf_size size of the buffer.
 */
static inline int parse_nal_units(AVCodecParserContext *s,
                                  AVCodecContext *avctx,
                                  const uint8_t *buf, int buf_size)
{
    HEVCContext   *h  = &((HEVCParseContext *)s->priv_data)->h;
    GetBitContext *gb = &h->HEVClc.gb;
    SliceHeader   *sh = &h->sh;
    const uint8_t *buf_end = buf + buf_size;
    int state = -1, i;
    HEVCNAL *nal;

    /* set some sane default values */
    s->pict_type         = AV_PICTURE_TYPE_I;
    s->key_frame         = 0;
    s->picture_structure = AV_PICTURE_STRUCTURE_UNKNOWN;

    h->avctx = avctx;

    if (!buf_size)
        return 0;

    if (h->nals_allocated < 1) {
        HEVCNAL *tmp = av_realloc_array(h->nals, 1, sizeof(*tmp));
        if (!tmp)
            return AVERROR(ENOMEM);
        h->nals = tmp;
        memset(h->nals, 0, sizeof(*tmp));
        h->nals_allocated = 1;
    }

    nal = &h->nals[0];

    for (;;) {
        int nal_length, src_length, consumed;

        if (h->is_nalff) {
            int i;
            if (buf + h->nal_length_size >= buf_end)
                break;
            nal_length = 0;
            for (i = 0; i < h->nal_length_size; i++)
                nal_length = (nal_length << 8) | *(buf++);
            if (nal_length <= 2 || buf + nal_length > buf_end)
                break;
            src_length = nal_length;
        } else {
            buf = avpriv_find_start_code(buf, buf_end, &state);
            if (--buf + 2 >= buf_end)
                break;
            src_length = buf_end - buf;
        }

        h->nal_unit_type = (*buf >> 1) & 0x3f;
        h->temporal_id   = (*(buf + 1) & 0x07) - 1;
        if (h->nal_unit_type <= NAL_CRA_NUT) {
            // Do not walk the whole buffer just to decode slice segment header
            if (src_length > 20)
                src_length = 20;
        }

        consumed = ff_hevc_extract_rbsp(buf, src_length, nal);
        if (consumed < 0)
            return consumed;

        init_get_bits(gb, nal->data + 2, 8 * (nal->size - 2));
        switch (h->nal_unit_type) {
        case NAL_VPS:
            ff_hevc_decode_nal_vps(h);
            break;
        case NAL_SPS:
            ff_hevc_decode_nal_sps(h);
            break;
        case NAL_PPS:
            ff_hevc_decode_nal_pps(h);
            break;
        case NAL_SEI_PREFIX:
        case NAL_SEI_SUFFIX:
            ff_hevc_decode_nal_sei(h);
            break;
        case NAL_TRAIL_N:
        case NAL_TRAIL_R:
        case NAL_TSA_N:
        case NAL_TSA_R:
        case NAL_STSA_N:
        case NAL_STSA_R:
        case NAL_RADL_N:
        case NAL_RADL_R:
        case NAL_RASL_N:
        case NAL_RASL_R:
        case NAL_BLA_W_LP:
        case NAL_BLA_W_RADL:
        case NAL_BLA_N_LP:
        case NAL_IDR_W_RADL:
        case NAL_IDR_N_LP:
        case NAL_CRA_NUT:
            sh->first_slice_in_pic_flag = get_bits1(gb);

            if (h->nal_unit_type >= 16 && h->nal_unit_type <= 23) {
                s->key_frame = 1;
                sh->no_output_of_prior_pics_flag = get_bits1(gb);
            }

            sh->pps_id = get_ue_golomb(gb);
            if (sh->pps_id >= MAX_PPS_COUNT || !h->pps_list[sh->pps_id]) {
                av_log(h->avctx, AV_LOG_ERROR, "PPS id out of range: %d\n", sh->pps_id);
                return AVERROR_INVALIDDATA;
            }
            h->pps = (HEVCPPS*)h->pps_list[sh->pps_id]->data;

            if (h->pps->sps_id >= MAX_SPS_COUNT || !h->sps_list[h->pps->sps_id]) {
                av_log(h->avctx, AV_LOG_ERROR, "SPS id out of range: %d\n", h->pps->sps_id);
                return AVERROR_INVALIDDATA;
            }
            if (h->sps != (HEVCSPS*)h->sps_list[h->pps->sps_id]->data) {
                h->sps = (HEVCSPS*)h->sps_list[h->pps->sps_id]->data;
                h->vps = h->vps_list[h->sps->vps_id];
            }

            if (!sh->first_slice_in_pic_flag) {
                int slice_address_length;

                if (h->pps->dependent_slice_segments_enabled_flag)
                    sh->dependent_slice_segment_flag = get_bits1(gb);
                else
                    sh->dependent_slice_segment_flag = 0;

                slice_address_length = av_ceil_log2_c(h->sps->ctb_width *
                                                      h->sps->ctb_height);
                sh->slice_segment_addr = get_bits(gb, slice_address_length);
                if (sh->slice_segment_addr >= h->sps->ctb_width * h->sps->ctb_height) {
                    av_log(h->avctx, AV_LOG_ERROR, "Invalid slice segment address: %u.\n",
                           sh->slice_segment_addr);
                    return AVERROR_INVALIDDATA;
                }
            } else
                sh->dependent_slice_segment_flag = 0;

            if (sh->dependent_slice_segment_flag)
                break;

            for (i = 0; i < h->pps->num_extra_slice_header_bits; i++)
                skip_bits(gb, 1); // slice_reserved_undetermined_flag[]

            sh->slice_type = get_ue_golomb(gb);
            if (!(sh->slice_type == I_SLICE || sh->slice_type == P_SLICE ||
                  sh->slice_type == B_SLICE)) {
                av_log(h->avctx, AV_LOG_ERROR, "Unknown slice type: %d.\n",
                       sh->slice_type);
                return AVERROR_INVALIDDATA;
            }
            s->pict_type = sh->slice_type == B_SLICE ? AV_PICTURE_TYPE_B :
                           sh->slice_type == P_SLICE ? AV_PICTURE_TYPE_P :
                                                       AV_PICTURE_TYPE_I;

            if (h->pps->output_flag_present_flag)
                sh->pic_output_flag = get_bits1(gb);

            if (h->sps->separate_colour_plane_flag)
                sh->colour_plane_id = get_bits(gb, 2);

            if (!IS_IDR(h)) {
                sh->pic_order_cnt_lsb = get_bits(gb, h->sps->log2_max_poc_lsb);
                s->output_picture_number = h->poc = ff_hevc_compute_poc(h, sh->pic_order_cnt_lsb);
            } else
                s->output_picture_number = h->poc = 0;

            if (h->temporal_id == 0 &&
                h->nal_unit_type != NAL_TRAIL_N &&
                h->nal_unit_type != NAL_TSA_N &&
                h->nal_unit_type != NAL_STSA_N &&
                h->nal_unit_type != NAL_RADL_N &&
                h->nal_unit_type != NAL_RASL_N &&
                h->nal_unit_type != NAL_RADL_R &&
                h->nal_unit_type != NAL_RASL_R)
                h->pocTid0 = h->poc;

            return 0; /* no need to evaluate the rest */
        }
        buf += h->is_nalff ? nal_length : consumed;
    }
    /* didn't find a picture! */
    av_log(h->avctx, AV_LOG_ERROR, "missing picture in access unit\n");
    return -1;
}

static int hevc_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                      const uint8_t **poutbuf, int *poutbuf_size,
                      const uint8_t *buf, int buf_size)
{
    int next;
    HEVCParseContext *hpc = (HEVCParseContext *)s->priv_data;
    ParseContext     *pc  = &hpc->pc;

    if (hpc->extradata_size != avctx->extradata_size ||
        (hpc->extradata && avctx->extradata &&
         memcmp(hpc->extradata, avctx->extradata, avctx->extradata_size))) {
        uint8_t *temp = av_realloc(hpc->extradata, avctx->extradata_size + FF_INPUT_BUFFER_PADDING_SIZE);
        if (temp) {
            hpc->extradata      = temp;
            hpc->extradata_size = avctx->extradata_size;
            memcpy(hpc->extradata, avctx->extradata, hpc->extradata_size);

            hpc->h.avctx = avctx;
            ff_hevc_decode_extradata(&hpc->h);
        }
    }

    if (s->flags & PARSER_FLAG_COMPLETE_FRAMES) {
        next = buf_size;
    } else {
        next = hevc_find_frame_end(s, buf, buf_size);
        if (ff_combine_frame(pc, next, &buf, &buf_size) < 0) {
            *poutbuf      = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    parse_nal_units(s, avctx, buf, buf_size);

    *poutbuf      = buf;
    *poutbuf_size = buf_size;
    return next;
}

// Split after the parameter sets at the beginning of the stream if they exist.
static int hevc_split(AVCodecContext *avctx, const uint8_t *buf, int buf_size)
{
    int i;
    uint32_t state = -1;
    int has_ps = 0;

    for (i = 0; i < buf_size; i++) {
        state = (state << 8) | buf[i];
        if (((state >> 8) & 0xFFFFFF) == START_CODE) {
            int nut = (state >> 1) & 0x3F;
            if (nut >= NAL_VPS && nut <= NAL_PPS)
                has_ps = 1;
            else if (has_ps)
                return i - 3;
            else // no parameter set at the beginning of the stream
                return 0;
        }
    }
    return 0;
}

static void close(AVCodecParserContext *s)
{
    int i;
    HEVCParseContext *hpc = (HEVCParseContext *)s->priv_data;
    HEVCContext      *h   = &hpc->h;
    ParseContext     *pc  = &hpc->pc;

    av_freep(&pc->buffer);

    av_freep(&hpc->extradata);

    for (i = 0; i < FF_ARRAY_ELEMS(h->vps_list); i++)
        av_freep(&h->vps_list[i]);
    for (i = 0; i < FF_ARRAY_ELEMS(h->sps_list); i++)
        av_buffer_unref(&h->sps_list[i]);
    for (i = 0; i < FF_ARRAY_ELEMS(h->pps_list); i++)
        av_buffer_unref(&h->pps_list[i]);

    for (i = 0; i < h->nals_allocated; i++)
        av_freep(&h->nals[i].rbsp_buffer);
    av_freep(&h->nals);
    h->nals_allocated = 0;
}

AVCodecParser ff_hevc_parser = {
    .codec_ids      = { AV_CODEC_ID_HEVC },
    .priv_data_size = sizeof(HEVCParseContext),
    .parser_parse   = hevc_parse,
    .parser_close   = close,
    .split          = hevc_split,
};
