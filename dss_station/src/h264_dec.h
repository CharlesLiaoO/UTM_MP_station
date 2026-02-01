#ifndef H264_DECODER_H
#define H264_DECODER_H

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/hwcontext.h>
#include <libswscale/swscale.h>
}

#include <vector>

// H264 Decoder: decode h264 to yuv
// Decoder function name rule:
// decode_aaa2bbb: decode aaa h264(frame/packet) to bbb yuv(frame)
//   aaa/bbb: cpu    (CPU memory)
//   aaa/bbb: bytes  (CPU memory)
// usually use: bytes2bytes and bytes2cpu
class H264Decoder {
public:
    H264Decoder(int width, int height);

    ~H264Decoder();

    // Decode bytes h264 to bytes yuv
    std::vector<uint8_t> decode_bytes2bytes(const std::vector<uint8_t>& h264_data) {
        auto frame = decode_bytes2cpu(h264_data);
        return cpu2bytes(frame);
    }

    // Decode bytes h264 to cpu yuv frame
    AVFrame* decode_bytes2cpu(const std::vector<uint8_t>& h264_data) {
        packet_->data = const_cast<uint8_t*>(h264_data.data());
        packet_->size = h264_data.size();
        return decode_cpu2cpu(packet_);
    }

    // Decode cpu h264 packet to cpu yuv frame
    AVFrame* decode_cpu2cpu(AVPacket* packet);

    // Convert YUV frame to RGB frame, with order rgb(AV_PIX_FMT_RGB24) or bgr(AV_PIX_FMT_BGR24)
    AVFrame* yuv2rgb(AVFrame* yuv_frame, AVPixelFormat rgb_order);

    // Transfer cpu frame to bytes
    std::vector<uint8_t> cpu2bytes(AVFrame* frame);

private:
    const AVCodec* codec_ = nullptr;
    AVCodecContext* codec_ctx_ = nullptr;
    AVPacket* packet_ = nullptr;
    AVFrame* frame_ = nullptr;
    AVFrame* frame_bgr_ = nullptr;
    SwsContext* sws_ctx_ = nullptr;
    AVBufferRef* hw_device_ctx_ = nullptr;  // VA-API hardware device context
    AVFrame* hw_frame_ = nullptr;  // Hardware frame for receiving decoded data
};

#endif // H264_DECODER_H