#include "h264_dec.h"

#include <stdexcept>
#include <cstring>

H264Decoder::H264Decoder(int width, int height)
{
    codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec_)
        throw std::runtime_error("H264 decoder not found");

    codec_ctx_ = avcodec_alloc_context3(codec_);
    codec_ctx_->width = width;
    codec_ctx_->height = height;
    codec_ctx_->pix_fmt = AV_PIX_FMT_NV16;
    codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;

    // VA-API hardware acceleration (Intel GPU), but here transfering data from gpu to cpu takes more time (10ms).
    // int ret = av_hwdevice_ctx_create(&hw_device_ctx_, AV_HWDEVICE_TYPE_VAAPI, nullptr, nullptr, 0);
    // if (ret >= 0) {
    //     // Successfully created VA-API device context
    //     codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
    //     hw_frame_ = av_frame_alloc();
    // }  // < 0 use software decoding

    if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0)
        throw std::runtime_error("Failed to open H264 decoder");

    packet_ = av_packet_alloc();
    frame_ = av_frame_alloc();
}

H264Decoder::~H264Decoder()
{
    av_packet_free(&packet_);
    av_frame_free(&frame_);
    if (frame_bgr_)
        av_frame_free(&frame_bgr_);
    if (hw_frame_)
        av_frame_free(&hw_frame_);
    if (sws_ctx_)
        sws_freeContext(sws_ctx_);
    avcodec_free_context(&codec_ctx_);
    if (hw_device_ctx_)
        av_buffer_unref(&hw_device_ctx_);
}

AVFrame *H264Decoder::decode_cpu2cpu(AVPacket *packet)
{
    int ret = avcodec_send_packet(codec_ctx_, packet);
    if (ret < 0) {
        // Return nullptr on error instead of throwing
        // This allows skipping incomplete/corrupted frames
        return nullptr;
    }

    // If hardware decoding is enabled, receive into hw_frame first
    AVFrame *recv_frame = hw_frame_ ? hw_frame_ : frame_;

    ret = avcodec_receive_frame(codec_ctx_, recv_frame);
    if (ret < 0) {
        // EAGAIN means we need more data, EOF means end of stream
        // Both are normal conditions, just return nullptr
        return nullptr;
    }

    // If hardware frame received, transfer to CPU memory
    if (hw_frame_ && recv_frame->format == AV_PIX_FMT_VAAPI) {
        ret = av_hwframe_transfer_data(frame_, recv_frame, 0);
        if (ret < 0) {
            // Hardware transfer failed, return nullptr
            return nullptr;
        }
        // Copy frame properties
        av_frame_copy_props(frame_, recv_frame);
        return frame_;
    }

    return recv_frame;
}

AVFrame *H264Decoder::yuv2rgb(AVFrame *yuv_frame, AVPixelFormat rgb_order)
{
    if (!yuv_frame || (rgb_order != AV_PIX_FMT_RGB24 && rgb_order != AV_PIX_FMT_BGR24))
        return nullptr;

    // Initialize SwsContext for color conversion
    if (!sws_ctx_) {
        sws_ctx_ = sws_getContext(
            yuv_frame->width, yuv_frame->height, (AVPixelFormat)yuv_frame->format,
            yuv_frame->width, yuv_frame->height, rgb_order,
            SWS_BILINEAR, nullptr, nullptr, nullptr);

        if (!sws_ctx_)
            throw std::runtime_error("Failed to create SwsContext");
    }

    // Allocate BGR frame if not already allocated
    if (!frame_bgr_) {
        frame_bgr_ = av_frame_alloc();
        frame_bgr_->format = rgb_order;
        frame_bgr_->width = yuv_frame->width;
        frame_bgr_->height = yuv_frame->height;

        if (av_frame_get_buffer(frame_bgr_, 0) < 0)
            throw std::runtime_error("Failed to allocate BGR frame buffer");
    }

    // Convert YUV to BGR
    sws_scale(sws_ctx_,
                yuv_frame->data, yuv_frame->linesize, 0, yuv_frame->height,
                frame_bgr_->data, frame_bgr_->linesize);

    return frame_bgr_;
}

std::vector<uint8_t> H264Decoder::cpu2bytes(AVFrame *frame)
{
    int yuv_size = av_image_get_buffer_size(
        (AVPixelFormat)frame->format, frame->width, frame->height, 1);

    std::vector<uint8_t> yuv_data(yuv_size);

    av_image_copy_to_buffer(yuv_data.data(), yuv_size,
        frame->data, frame->linesize,
        (AVPixelFormat)frame->format, frame->width, frame->height, 1);

    return yuv_data;
}
