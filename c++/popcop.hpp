/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2017-2018 Zubax Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#ifndef POPCOP_HPP_INCLUDED
#define POPCOP_HPP_INCLUDED

#if __cplusplus < 201703
# error "This library requires C++17 or newer"
#endif

#include <type_traits>
#include <algorithm>
#include <optional>
#include <variant>
#include <cstring>
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <chrono>
#include <array>


namespace popcop
{
/**
 * PoPCoP transport implementation - data emission and parsing.
 */
namespace transport
{
/**
 * The two special characters were chosen to be among the least frequently used characters in
 * typical embedded applications. Additionally, it is important that these characters are outside
 * of the 7-bit ASCII set, which allows applications to mix ASCII data with encoded binary data
 * in the same communication channel.
 */
static constexpr std::uint8_t FrameDelimiter  = 0x8E;
static constexpr std::uint8_t EscapeCharacter = 0x9E;

/**
 * The library guarantees that the buffer pointer returned by the parser is aligned at std::max_align_t
 * or at 64 bytes, whichever is larger. This guarantee allows the application to directly alias the
 * buffer pointer to a POD structure.
 */
static constexpr std::size_t ParserBufferAlignment = std::max<std::size_t>(64U, alignof(std::max_align_t)); // NOLINT

/**
 * Implementation of the CRC-32C (Castagnoli) algorithm.
 */
class CRCComputer
{
    std::uint32_t value_ = 0xFFFFFFFFU;

public:
    void add(std::uint8_t byte)
    {
        static constexpr std::uint32_t table[] =
        {
            0x00000000U, 0xF26B8303U, 0xE13B70F7U, 0x1350F3F4U, 0xC79A971FU, 0x35F1141CU, 0x26A1E7E8U, 0xD4CA64EBU,
            0x8AD958CFU, 0x78B2DBCCU, 0x6BE22838U, 0x9989AB3BU, 0x4D43CFD0U, 0xBF284CD3U, 0xAC78BF27U, 0x5E133C24U,
            0x105EC76FU, 0xE235446CU, 0xF165B798U, 0x030E349BU, 0xD7C45070U, 0x25AFD373U, 0x36FF2087U, 0xC494A384U,
            0x9A879FA0U, 0x68EC1CA3U, 0x7BBCEF57U, 0x89D76C54U, 0x5D1D08BFU, 0xAF768BBCU, 0xBC267848U, 0x4E4DFB4BU,
            0x20BD8EDEU, 0xD2D60DDDU, 0xC186FE29U, 0x33ED7D2AU, 0xE72719C1U, 0x154C9AC2U, 0x061C6936U, 0xF477EA35U,
            0xAA64D611U, 0x580F5512U, 0x4B5FA6E6U, 0xB93425E5U, 0x6DFE410EU, 0x9F95C20DU, 0x8CC531F9U, 0x7EAEB2FAU,
            0x30E349B1U, 0xC288CAB2U, 0xD1D83946U, 0x23B3BA45U, 0xF779DEAEU, 0x05125DADU, 0x1642AE59U, 0xE4292D5AU,
            0xBA3A117EU, 0x4851927DU, 0x5B016189U, 0xA96AE28AU, 0x7DA08661U, 0x8FCB0562U, 0x9C9BF696U, 0x6EF07595U,
            0x417B1DBCU, 0xB3109EBFU, 0xA0406D4BU, 0x522BEE48U, 0x86E18AA3U, 0x748A09A0U, 0x67DAFA54U, 0x95B17957U,
            0xCBA24573U, 0x39C9C670U, 0x2A993584U, 0xD8F2B687U, 0x0C38D26CU, 0xFE53516FU, 0xED03A29BU, 0x1F682198U,
            0x5125DAD3U, 0xA34E59D0U, 0xB01EAA24U, 0x42752927U, 0x96BF4DCCU, 0x64D4CECFU, 0x77843D3BU, 0x85EFBE38U,
            0xDBFC821CU, 0x2997011FU, 0x3AC7F2EBU, 0xC8AC71E8U, 0x1C661503U, 0xEE0D9600U, 0xFD5D65F4U, 0x0F36E6F7U,
            0x61C69362U, 0x93AD1061U, 0x80FDE395U, 0x72966096U, 0xA65C047DU, 0x5437877EU, 0x4767748AU, 0xB50CF789U,
            0xEB1FCBADU, 0x197448AEU, 0x0A24BB5AU, 0xF84F3859U, 0x2C855CB2U, 0xDEEEDFB1U, 0xCDBE2C45U, 0x3FD5AF46U,
            0x7198540DU, 0x83F3D70EU, 0x90A324FAU, 0x62C8A7F9U, 0xB602C312U, 0x44694011U, 0x5739B3E5U, 0xA55230E6U,
            0xFB410CC2U, 0x092A8FC1U, 0x1A7A7C35U, 0xE811FF36U, 0x3CDB9BDDU, 0xCEB018DEU, 0xDDE0EB2AU, 0x2F8B6829U,
            0x82F63B78U, 0x709DB87BU, 0x63CD4B8FU, 0x91A6C88CU, 0x456CAC67U, 0xB7072F64U, 0xA457DC90U, 0x563C5F93U,
            0x082F63B7U, 0xFA44E0B4U, 0xE9141340U, 0x1B7F9043U, 0xCFB5F4A8U, 0x3DDE77ABU, 0x2E8E845FU, 0xDCE5075CU,
            0x92A8FC17U, 0x60C37F14U, 0x73938CE0U, 0x81F80FE3U, 0x55326B08U, 0xA759E80BU, 0xB4091BFFU, 0x466298FCU,
            0x1871A4D8U, 0xEA1A27DBU, 0xF94AD42FU, 0x0B21572CU, 0xDFEB33C7U, 0x2D80B0C4U, 0x3ED04330U, 0xCCBBC033U,
            0xA24BB5A6U, 0x502036A5U, 0x4370C551U, 0xB11B4652U, 0x65D122B9U, 0x97BAA1BAU, 0x84EA524EU, 0x7681D14DU,
            0x2892ED69U, 0xDAF96E6AU, 0xC9A99D9EU, 0x3BC21E9DU, 0xEF087A76U, 0x1D63F975U, 0x0E330A81U, 0xFC588982U,
            0xB21572C9U, 0x407EF1CAU, 0x532E023EU, 0xA145813DU, 0x758FE5D6U, 0x87E466D5U, 0x94B49521U, 0x66DF1622U,
            0x38CC2A06U, 0xCAA7A905U, 0xD9F75AF1U, 0x2B9CD9F2U, 0xFF56BD19U, 0x0D3D3E1AU, 0x1E6DCDEEU, 0xEC064EEDU,
            0xC38D26C4U, 0x31E6A5C7U, 0x22B65633U, 0xD0DDD530U, 0x0417B1DBU, 0xF67C32D8U, 0xE52CC12CU, 0x1747422FU,
            0x49547E0BU, 0xBB3FFD08U, 0xA86F0EFCU, 0x5A048DFFU, 0x8ECEE914U, 0x7CA56A17U, 0x6FF599E3U, 0x9D9E1AE0U,
            0xD3D3E1ABU, 0x21B862A8U, 0x32E8915CU, 0xC083125FU, 0x144976B4U, 0xE622F5B7U, 0xF5720643U, 0x07198540U,
            0x590AB964U, 0xAB613A67U, 0xB831C993U, 0x4A5A4A90U, 0x9E902E7BU, 0x6CFBAD78U, 0x7FAB5E8CU, 0x8DC0DD8FU,
            0xE330A81AU, 0x115B2B19U, 0x020BD8EDU, 0xF0605BEEU, 0x24AA3F05U, 0xD6C1BC06U, 0xC5914FF2U, 0x37FACCF1U,
            0x69E9F0D5U, 0x9B8273D6U, 0x88D28022U, 0x7AB90321U, 0xAE7367CAU, 0x5C18E4C9U, 0x4F48173DU, 0xBD23943EU,
            0xF36E6F75U, 0x0105EC76U, 0x12551F82U, 0xE03E9C81U, 0x34F4F86AU, 0xC69F7B69U, 0xD5CF889DU, 0x27A40B9EU,
            0x79B737BAU, 0x8BDCB4B9U, 0x988C474DU, 0x6AE7C44EU, 0xBE2DA0A5U, 0x4C4623A6U, 0x5F16D052U, 0xAD7D5351U,
        };
        static_assert(sizeof(table) == 1024, "Invalid CRC table");

        value_ = table[byte ^ (value_ & 0xFF)] ^ (value_ >> 8);
    }

    [[nodiscard]] std::uint32_t get() const { return value_ ^ 0xFFFFFFFFU; }

    /**
     * Checks whether the current CRC value is a correct residual.
     * This method can be used to perform 'one-pass checking' of datagrams, where the CRC is computed over
     * the payload and then over the CRC fields as well.
     */
    [[nodiscard]] bool isResidueCorrect() const
    {
        return value_ == 0xB798B438U;
    }
};

/**
 * Result type of the Parser state update function.
 * The parser guarantees that the buffer pointer is aligned at least at std::max_align_t (or even larger).
 * This guarantee allows the application to directly alias the buffer pointer to a POD structure.
 * This is how you use it:
 *
 *   while (true)
 *   {
 *       const auto out = parser.processNextByte(readByte());
 *       if (auto x = out.getReceivedFrame())
 *       {
 *           // Process the received frame here
 *       }
 *       else if (auto x = out.getExtraneousData())
 *       {
 *           // Process the unparseable data, if you need it (e.g. if the binary interface is overlaid over
 *           // a plain text logging, this is where you can extract the text from).
 *           // If you don't need the unparsed data, just remove this branch.
 *       }
 *       else
 *       {
 *           ;   // Nothing to do
 *       }
 *   }
 */
class [[nodiscard]] ParserOutput
{
public:
    class AlignedBufferView
    {
        std::size_t size_;
        const std::uint8_t* ptr_;

    public:
        AlignedBufferView() :
            size_(0),
            ptr_(nullptr)
        { }

        AlignedBufferView(const std::uint8_t* data_ptr, std::size_t data_size) :
            size_(data_size),
            ptr_(data_ptr)
        {
            assert(data_ptr != nullptr);
            // We GUARANTEE proper alignment for the application, checking it here at runtime.
            assert((reinterpret_cast<std::uintptr_t>(data_ptr) % ParserBufferAlignment) == 0);
        }

        // The API is modeled after STL containers
        const std::uint8_t* begin() const { return ptr_; }
        const std::uint8_t* end() const { return ptr_ + size_; }
        std::size_t size() const { return size_; }
        [[nodiscard]]   // prevents confusion with clear()
        bool empty() const { return size_ == 0; }

        /**
         * The buffer pointer is guaranteed to be aligned at least at std::max_align_t (or even larger).
         * This guarantee allows the application to directly alias the buffer pointer to a POD structure.
         */
        const std::uint8_t* data() const
        {
            assert((reinterpret_cast<std::uintptr_t>(ptr_) % ParserBufferAlignment) == 0);
            return ptr_;
        }

        std::uint8_t at(const std::size_t index) const
        {
            assert(index < size_);
            return *(ptr_ + index);
        }

        /**
         * This helper function performs safe aliasing of the specified type to the underlying buffer.
         * Alignment requirements are checked at compile time.
         * Be careful using this function, remember that type punning is never safe.
         */
        template <typename T>
        const T& alias() const
        {
            // Static alignment check at no runtime cost
            static_assert(alignof(T) <= ParserBufferAlignment,
                          "Aliasing the buffer to a type with stricter alignment requirements is unsafe.");

            // Super paranoid check, debug builds only
            assert((reinterpret_cast<std::uintptr_t>(ptr_) % alignof(T)) == 0);

            return *static_cast<const T*>(static_cast<const void*>(ptr_));
        }
    };

    struct Frame
    {
        std::uint8_t type_code = 0;
        AlignedBufferView payload;
    };

private:
    enum class Contents : std::uint_fast8_t
    {
        None,
        Frame,
        ExtraneousData,
    };

    Frame frame_;
    Contents contents_;

public:
    ParserOutput() :
        contents_(Contents::None)
    { }

    ParserOutput(std::uint8_t type_code, const std::uint8_t* data_ptr, std::size_t data_size) :
        contents_(Contents::Frame)
    {
        frame_.type_code = type_code;
        frame_.payload = AlignedBufferView(data_ptr, data_size);
    }

    ParserOutput(const std::uint8_t* data_ptr, std::size_t data_size) :
        contents_(Contents::ExtraneousData)
    {
        frame_.payload = AlignedBufferView(data_ptr, data_size);
    }

    const Frame* getReceivedFrame() const
    {
        return (contents_ == Contents::Frame) ? &frame_ : nullptr;
    }

    const AlignedBufferView* getExtraneousData() const
    {
        return (contents_ == Contents::ExtraneousData) ? &frame_.payload : nullptr;
    }
};

/**
 * Simple and robust parser.
 * TODO: add support for timestamping in the future.
 * @tparam MaxPayloadSize   The maximum length of payload this parser will be able to receive.
 *                          This value should not be less than 1024 bytes.
 */
template <std::size_t MaxPayloadSize = 2048>
class Parser
{
    static_assert(MaxPayloadSize >= 1024, "Maximum payload size should be larger");

    static constexpr std::uint8_t PayloadOverheadNotIncludingDelimiters = 5;

    /// We add +1 to the size because of a special case explained in the update method.
    /// The buffer pointer passed to the application is GUARANTEED to be aligned.
    alignas(ParserBufferAlignment)
    std::array<std::uint8_t, MaxPayloadSize + PayloadOverheadNotIncludingDelimiters + 1> buffer_;

    std::size_t buffer_pos_ = 0;
    CRCComputer crc_;
    bool unescape_next_ = false;

    bool checkIfReceivedFrameValid()
    {
        return (buffer_pos_ >= PayloadOverheadNotIncludingDelimiters) && (crc_.isResidueCorrect());
    }

    class RAIIFrameFinalizer
    {
        Parser& self_;

    public:
        explicit RAIIFrameFinalizer(Parser* s) : self_(*s) { }

        ~RAIIFrameFinalizer()
        {
            self_.unescape_next_ = false;
            self_.buffer_pos_ = 0;
            self_.crc_ = CRCComputer();
        }
    };

public:
    /**
     * Invoke this method for every received byte from the channel.
     * The inner state machine will attempt to decode data from the channel as it comes in.
     *
     * @param x     The byte of data received from the channel.
     *
     * @return      A parser output object, which has one of three possible states:
     *                  - A new frame has been received. The object contains a reference to the frame.
     *                  - An unparseable data is available. The application can either ignore it or
     *                    use it in some way (e.g. print it as-is for the user to inspect, log it,
     *                    or update the statistics of damaged packets).
     *                  - Nothing is available - parsing is in progress.
     *              Beware that the returned object contains references to the data that WILL BE
     *              INVALIDATED upon the next execution of this method!
     */
    ParserOutput processNextByte(std::uint8_t x)
    {
        if (x == FrameDelimiter)
        {
            // This protocol offers a transparent data channel, which means that it is guaranteed that
            // a frame delimiter will NEVER occur in the payload. Therefore, having received a frame
            // delimiter, we unconditionally finalize the current frame and see if it is valid.
            // If it is, the frame is returned. If some data was received but it does not constitute a valid
            // frame, it is received as-is, marked as 'unparsed data' (perhaps the application can still make
            // some use of it). If no data was received at all, we return an empty output.
            RAIIFrameFinalizer finalizer(this);

            if (checkIfReceivedFrameValid())
            {
                return ParserOutput(buffer_[buffer_pos_ - PayloadOverheadNotIncludingDelimiters],
                                    buffer_.data(),
                                    buffer_pos_ - PayloadOverheadNotIncludingDelimiters);
            }
            else if (buffer_pos_ > 0)
            {
                return ParserOutput(buffer_.data(), buffer_pos_);
            }
            else
            {
                return {};
            }
        }

        if (x == EscapeCharacter)
        {
            unescape_next_ = true;
            return {};
        }

        if (unescape_next_)
        {
            unescape_next_ = false;
            x = std::uint8_t(~x);
        }

        // Just add one byte and update the CRC. What we're receiving may not be a valid frame; the
        // only way to detect whether it's a valid frame or not is to wait until the next frame delimiter
        // and see if the CRC equals the right magic value.
        assert(buffer_pos_ < buffer_.size());
        buffer_[buffer_pos_] = x;
        ++buffer_pos_;
        crc_.add(x);

        if (buffer_pos_ >= buffer_.size())
        {
            // We ran out of space in the buffer! There is no hope to have a valid frame received then,
            // so we just unload this data to the application as unparseable bytes and reset the buffer.
            // Note an interesting case: if the next byte were a frame delimiter, we would still be able
            // to receive the frame properly, because when a frame delimiter is received we don't have
            // to put anything into the buffer. So from this standpoint it would make sense to postpone
            // the buffer overflow detection until the next byte. However, if we did that, we would always
            // end up losing one byte if the next character is not a frame delimiter, because we would have
            // no place to store that one byte in - the buffer would be full! Therefore, we detect overflow
            // one cycle early in order to ensure that no byte is lost. This is why we allocate one byte
            // more in the buffer than the maximum payload length requires.
            RAIIFrameFinalizer finalizer(this);
            return ParserOutput(buffer_.data(), buffer_pos_);
        }
        else
        {
            return {};
        }
    }

    /**
     * Resets the inner state of the parser.
     * Use this method when your communication channel is reset.
     */
    void reset()
    {
        RAIIFrameFinalizer finalizer(this);
    }
};

/**
 * Simple byte-by-byte data encoder/emitter.
 */
class Emitter
{
    enum class State : std::uint8_t
    {
        FrontDelimiter,
        Payload,
        FrameType,
        CRCFirst,
        CRCSecond,
        CRCThird,
        CRCFourth,
        BackDelimiter,
        Finished,
    };

    class EscapeInjector
    {
        bool pending_ = false;
        std::uint8_t escaped_ = 0;

    public:
        std::uint8_t process(const std::uint8_t x)
        {
            assert(!pending_);
            if ((x == FrameDelimiter) || (x == EscapeCharacter))
            {
                pending_ = true;
                escaped_ = std::uint8_t(~x);
                return EscapeCharacter;
            }
            else
            {
                return x;
            }
        }

        bool isPending() const { return pending_; }

        std::uint8_t getPendingByte()
        {
            assert(pending_);
            pending_ = false;
            return escaped_;
        }
    };

    std::uint8_t type_code_;              // Made non-const in order to make the object copyable
    std::size_t remaining_bytes_;
    const std::uint8_t* next_byte_ptr_;
    CRCComputer crc_;
    State state_ = State::FrontDelimiter;
    EscapeInjector escaper_;

public:
    /**
     * Constructs the object from a raw data pointer with explicit length.
     *
     * @param frame_type_code       Frame type code to use with this frame.
     *
     * @param payload_ptr           Pointer to the frame payload.
     *
     * @param payload_size          Length of the frame payload.
     */
    Emitter(std::uint8_t frame_type_code,
            const void* payload_ptr,
            std::size_t payload_size) :
        type_code_(frame_type_code),
        remaining_bytes_(payload_size),
        next_byte_ptr_(static_cast<const std::uint8_t*>(payload_ptr))
    { }

    /**
     * A helper constructor that accepts a reference to a class/struct object, and uses its
     * size by default. The size can be overridden by the caller if necessary.
     *
     * @tparam T                    The type of the object; it is deduced automatically.
     *                              The following concept requirements apply (if they are
     *                              not met, the candidate will be rejected by SFINAE):
     *                                  - T must be a class or a struct
     *                                  - sizeof(T) > 0
     *                                  - T must be trivially copyable
     *                                  - T must not have a virtual destructor
     *
     * @param frame_type_code       Frame type code to use with this frame.
     *
     * @param payload_object_ref    Const reference to the object that will be encoded.
     *
     * @param payload_size          Optional size of the object. If not provided, sizeof(T) will be used.
     *                              If provided, it must not be larger than sizeof(T).
     */
    template <typename T,
              typename = std::enable_if_t<std::is_class<T>::value &&
                                         (sizeof(T) > 0) &&
                                         !std::has_virtual_destructor<T>::value &&
                                          std::is_trivially_copyable<T>::value>>
    Emitter(std::uint8_t frame_type_code,
            const T& payload_object_ref,
            std::size_t payload_size = sizeof(T)) :
        Emitter(frame_type_code,
                static_cast<const void*>(&payload_object_ref),
                payload_size)
    {
        assert(payload_size <= sizeof(T));
    }

    /**
     * Returns true if there are no more bytes to emit.
     */
    [[nodiscard]]
    bool isFinished() const
    {
        return state_ == State::Finished;
    }

    /**
     * Returns the next byte to emit.
     * This function should not be invoked if @ref isFinished() returns true.
     */
    [[nodiscard]]
    std::uint8_t getNextByte()
    {
        if (escaper_.isPending())
        {
            return escaper_.getPendingByte();
        }

        switch (state_)
        {
        case State::FrontDelimiter:
        {
            state_ = (remaining_bytes_ > 0) ? State::Payload : State::FrameType;
            return FrameDelimiter;
        }

        case State::Payload:
        {
            const std::uint8_t x = *next_byte_ptr_;
            ++next_byte_ptr_;
            --remaining_bytes_;

            if (remaining_bytes_ == 0)
            {
                state_ = State::FrameType;
            }

            crc_.add(x);
            return escaper_.process(x);
        }

        case State::FrameType:
        {
            state_ = State::CRCFirst;
            crc_.add(type_code_);
            return escaper_.process(type_code_);
        }

        case State::CRCFirst:
        {
            state_ = State::CRCSecond;
            return escaper_.process(std::uint8_t(crc_.get() >> 0));
        }

        case State::CRCSecond:
        {
            state_ = State::CRCThird;
            return escaper_.process(std::uint8_t(crc_.get() >> 8));
        }

        case State::CRCThird:
        {
            state_ = State::CRCFourth;
            return escaper_.process(std::uint8_t(crc_.get() >> 16));
        }

        case State::CRCFourth:
        {
            state_ = State::BackDelimiter;
            return escaper_.process(std::uint8_t(crc_.get() >> 24));
        }

        case State::BackDelimiter:
        case State::Finished:
        {
            state_ = State::Finished;
            break;
        }
        }

        // Stuff the output after the packet is finished with frame delimiters.
        return FrameDelimiter;
    }
};

} // namespace transport

/**
 * Utilities and helpers.
 */
namespace util
{
/**
 * A helper for constructing integer types of given bit width.
 */
template <std::size_t BitWidth>
struct UnsignedImpl;
template <> struct UnsignedImpl<8>  { using Type = std::uint8_t; };
template <> struct UnsignedImpl<16> { using Type = std::uint16_t; };
template <> struct UnsignedImpl<32> { using Type = std::uint32_t; };
template <> struct UnsignedImpl<64> { using Type = std::uint64_t; };

template <std::size_t BitWidth>
using Unsigned = typename UnsignedImpl<BitWidth>::Type;

/**
 * A string with fixed storage, API like std::string.
 */
template <std::size_t Capacity_>
class FixedCapacityString
{
public:
    static constexpr std::size_t Capacity = Capacity_;

    static_assert(Capacity > 0, "Capacity must be positive");

private:
    template <std::size_t C>
    friend class FixedCapacityString;

    std::size_t len_ = 0;
    char buf_[Capacity + 1];

public:
    FixedCapacityString() // NOLINT
    {
        buf_[len_] = '\0';
    }

    /// Implicit on purpose
    FixedCapacityString(const char* const initializer) // NOLINT
    {
        (*this) += initializer;
    }

    template <typename InputIterator>
    FixedCapacityString(InputIterator begin, const InputIterator end) // NOLINT
    {
        while ((begin != end) && (len_ < Capacity))
        {
            buf_[len_] = static_cast<char>(*begin);
            ++len_;
            ++begin;
        }
        buf_[len_] = '\0';
    }

    template <std::size_t C>
    FixedCapacityString(const FixedCapacityString<C>& initializer) // NOLINT
    {
        (*this) += initializer;
    }

    /*
     * std::string API
     */
    using value_type = char;
    using size_type = std::size_t;
    using iterator = const char*;           ///< Const is intentional
    using const_iterator = const char*;

    [[nodiscard]] constexpr std::size_t capacity() const { return Capacity; }
    [[nodiscard]] constexpr std::size_t max_size() const { return Capacity; }

    [[nodiscard]] std::size_t size()   const { return len_; }
    [[nodiscard]] std::size_t length() const { return len_; }

    [[nodiscard]] // nodiscard prevents confusion with clear()
    bool empty() const { return len_ == 0; }

    [[nodiscard]] const char* c_str() const { return &buf_[0]; }

    void clear()
    {
        len_ = 0;
        buf_[len_] = '\0';
    }

    void push_back(char c)
    {
        if (len_ < Capacity)
        {
            buf_[len_] = c;
            ++len_;
        }
        buf_[len_] = '\0';
    }

    [[nodiscard]] char&       front()       { return operator[](0); }
    [[nodiscard]] const char& front() const { return operator[](0); }

    [[nodiscard]]
    char& back()
    {
        if (len_ > 0)
        {
            return buf_[len_ - 1U];
        }
        else
        {
            assert(false);
            return buf_[0];
        }
    }
    [[nodiscard]] const char& back() const { return const_cast<FixedCapacityString*>(this)->back(); }

    /*
     * Iterators - only const iterators are provided for safety reasons.
     * It is easy to accidentally bring the object into an invalid state by zeroing a char in the middle.
     */
    [[nodiscard]] const char* begin() const { return &buf_[0]; }
    [[nodiscard]] const char* end()   const { return &buf_[len_]; }

    /*
     * Operators
     */
    template <typename T>
    FixedCapacityString& operator=(const T& s)
    {
        clear();
        (*this) += s;
        return *this;
    }

    template <typename T, typename = decltype(std::declval<T>().c_str())>
    FixedCapacityString& operator+=(const T& s)
    {
        (*this) += s.c_str();
        return *this;
    }

    FixedCapacityString& operator+=(const char* p)
    {
        while ((*p != '\0') && (len_ < Capacity))
        {
            buf_[len_] = *p++;
            ++len_;
        }
        buf_[len_] = '\0';
        return *this;
    }

    FixedCapacityString& operator+=(char c)
    {
        push_back(c);
        return *this;
    }

    [[nodiscard]]
    char& operator[](std::size_t index)
    {
        if (index < len_)
        {
            return buf_[index];
        }
        else
        {
            assert(false);
            return back();
        }
    }
    [[nodiscard]]
    const char& operator[](std::size_t index) const
    {
        return const_cast<FixedCapacityString*>(this)->operator[](index);
    }

    template <typename T, typename = decltype(std::declval<T>().begin())>
    [[nodiscard]]
    bool operator==(const T& s) const
    {
        return std::equal(begin(), end(), std::begin(s), std::end(s));
    }

    [[nodiscard]]
    bool operator==(const char* s) const
    {
        return 0 == std::strncmp(this->c_str(), s, sizeof(buf_));
    }

    template <typename T>
    [[nodiscard]]
    bool operator!=(const T& s) const
    {
        return !operator==(s);
    }

    /*
     * Helpers
     */
    [[nodiscard]]   // nodiscard is used to emphasize that the method is not mutating
    FixedCapacityString<Capacity> toLowerCase() const
    {
        FixedCapacityString<Capacity> out;
        std::transform(begin(), end(), std::back_inserter(out), [](char c) { return char(std::tolower(c)); });
        return out;
    }

    [[nodiscard]]   // nodiscard is used to emphasize that the method is not mutating
    FixedCapacityString<Capacity> toUpperCase() const
    {
        FixedCapacityString<Capacity> out;
        std::transform(begin(), end(), std::back_inserter(out), [](char c) { return char(std::toupper(c)); });
        return out;
    }
};

template <std::size_t LeftCapacity, std::size_t RightCapacity>
[[nodiscard]]
inline auto operator+(const FixedCapacityString<LeftCapacity>& left, const FixedCapacityString<RightCapacity>& right)
{
    FixedCapacityString<LeftCapacity + RightCapacity> out(left);
    out += right;
    return out;
}

template <std::size_t Capacity>
[[nodiscard]]
inline auto operator+(const FixedCapacityString<Capacity>& left, const char* right)
{
    FixedCapacityString<Capacity> out(left);
    out += right;
    return out;
}

template <std::size_t Capacity>
[[nodiscard]]
inline auto operator+(const char* left, const FixedCapacityString<Capacity>& right)
{
    FixedCapacityString<Capacity> out(left);
    out += right;
    return out;
}

template <std::size_t Capacity>
[[nodiscard]]
inline bool operator==(const char* left, const FixedCapacityString<Capacity>& right)
{
    return right == left;
}

template <std::size_t Capacity>
[[nodiscard]]
inline bool operator!=(const char* left, const FixedCapacityString<Capacity>& right)
{
    return right != left;
}

/**
 * A vector with fixed storage, API like std::vector<>.
 * This implementation supports only trivial types, since that is sufficient for the needs of this library.
 */
template <typename T, std::size_t Capacity_>
class FixedCapacityVector
{
public:
    static constexpr std::size_t Capacity = Capacity_;

    static_assert(Capacity > 0, "Capacity must be positive");
    static_assert(std::is_trivial<T>::value, "This implementation supports only trivial types.");

private:
    std::size_t len_ = 0;
    T buf_[Capacity]{};

public:
    FixedCapacityVector() = default;

    template <typename InputIterator>
    FixedCapacityVector(InputIterator begin, const InputIterator end) // NOLINT
    {
        while ((begin != end) && (len_ < Capacity))
        {
            buf_[len_] = T(*begin);
            ++len_;
            ++begin;
        }
    }

    /*
     * std::vector API
     */
    using value_type = T;
    using size_type = std::size_t;
    using iterator = T*;
    using const_iterator = const T*;

    [[nodiscard]] constexpr std::size_t capacity() const { return Capacity; }
    [[nodiscard]] constexpr std::size_t max_size() const { return Capacity; }

    [[nodiscard]] std::size_t size() const { return len_; }

    [[nodiscard]] // nodiscard prevents confusion with clear()
    bool empty() const { return len_ == 0; }

    void clear()
    {
        len_ = 0;
    }

    void push_back(const T& c)
    {
        if (len_ < Capacity)
        {
            buf_[len_] = c;
            ++len_;
        }
        else
        {
            assert(false);
        }
    }

    [[nodiscard]] T&       front()       { return operator[](0); }
    [[nodiscard]] const T& front() const { return operator[](0); }

    [[nodiscard]]
    T& back()
    {
        if (len_ > 0)
        {
            return buf_[len_ - 1U];
        }
        else
        {
            assert(false);
            return buf_[0];
        }
    }
    [[nodiscard]] const T& back() const { return const_cast<FixedCapacityVector*>(this)->back(); }

    [[nodiscard]] T* begin() { return &buf_[0]; }
    [[nodiscard]] T* end()   { return &buf_[len_]; }

    [[nodiscard]] const T* begin() const { return &buf_[0]; }
    [[nodiscard]] const T* end()   const { return &buf_[len_]; }

    /*
     * Operators
     */
    [[nodiscard]]
    T& operator[](std::size_t index)
    {
        if (index < len_)
        {
            return buf_[index];
        }
        else
        {
            assert(false);
            return back();
        }
    }
    [[nodiscard]]
    const T& operator[](std::size_t index) const
    {
        return const_cast<FixedCapacityVector*>(this)->operator[](index);
    }

    template <typename S, typename = decltype(std::declval<S>().begin())>
    [[nodiscard]]
    bool operator==(const S& s) const
    {
        return std::equal(begin(), end(), std::begin(s), std::end(s));
    }

    template <typename S>
    [[nodiscard]]
    bool operator!=(const S& s) const
    {
        return !operator==(s);
    }
};

}   // namespace util

/**
 * Data presentation layer.
 */
namespace presentation
{
/**
 * Application-specific frame type codes range from 0 to 127 (0x7F), inclusive.
 * Standard frame type codes range from 128 (0x80) to 255 (0xFF), inclusive. Currently, only 255 (0xFF) is used.
 */
static constexpr std::uint8_t StandardFrameTypeCode               = 0xFF;
static constexpr std::uint8_t MaxApplicationSpecificFrameTypeCode = 0x7F;

/**
 * Helper type used to specify whether an entity is mutable.
 */
enum class Mutability
{
    Immutable,
    Mutable
};

/**
 * This base can be inherited by frame structures or it can be used directly.
 *
 * It provides convenient functions that enable easy serialization and
 * deserialization of fields that are of primitive types, ensuring correct byte ordering.
 * Refer to the methods set(), setASCIIString(), get(), getASCIIString() for more info.
 *
 * It is also possible to avoid serialization completely by merely treating data structures as
 * raw array of bytes (via std::memcpy()); however, this approach will break on architectures that
 * are not little-endian; additionally, it is unsafe because it relies on type punning.
 */
template <Mutability UnderlyingBufferMutability>
class ScalarCodec
{
    using BytePtr = std::conditional_t<UnderlyingBufferMutability == Mutability::Mutable,
                                       std::uint8_t*,
                                       const std::uint8_t*>;
    // The fields are non-const because we want it to be copyable.
    std::size_t buffer_size_;
    BytePtr buffer_ptr_;

    /// Internal use only
    ScalarCodec(BytePtr buf, std::size_t sz) :
        buffer_size_(sz),
        buffer_ptr_(buf)
    {
        assert(buffer_ptr_ != nullptr);
    }

public:
    /**
     * This constructor binds the serializer to an arbitrary underlying buffer that
     * has methods .size() and .data(), such as the standard std::array<>, std::vector<>,
     * or the AlignedBufferView class defined in this module.
     */
    template <typename T,
              typename = decltype(std::declval<T>().size()),
              typename = decltype(std::declval<T>().data())>
    explicit ScalarCodec(T& buffer) :
        buffer_size_(buffer.size()),
        buffer_ptr_(buffer.data())
    {
        assert(buffer_ptr_ != nullptr);
    }

    template <typename T>
    std::enable_if_t<std::is_unsigned<T>::value> set(const std::size_t offset, const T& value)
    {
        static_assert(UnderlyingBufferMutability == Mutability::Mutable, "The underlying buffer is immutable");
        assert((offset + sizeof(T)) <= buffer_size_);
        for (std::size_t i = 0; i < sizeof(T); i++)         // Dear compiler, consider unrolling this please.
        {
            buffer_ptr_[offset + i] = std::uint8_t(value >> (i * 8U));
        }
    }

    template <typename T>
    std::enable_if_t<std::is_signed<T>::value> set(const std::size_t offset, const T& value)
    {
        // This specialization handles all signed types including float and double.
        // Long double might fail at compile time if it is greater than uint64.
        using Unsigned = util::Unsigned<sizeof(T) * 8>;
        Unsigned u{};
        (void) std::memcpy(&u, &value, sizeof(T));  // Copy because we don't want to violate strict aliasing rules
        set<Unsigned>(offset, u);
    }

    template <typename InputIterator>
    void set(std::size_t offset, InputIterator begin, const InputIterator end)
    {
        static_assert(UnderlyingBufferMutability == Mutability::Mutable, "The underlying buffer is immutable");
        while (begin != end)
        {
            assert(offset < buffer_size_);
            buffer_ptr_[offset] = std::uint8_t(*begin);
            ++offset;
            ++begin;
        }
    }

    template <std::size_t Capacity>
    void setASCIIString(const std::size_t offset, const util::FixedCapacityString<Capacity>& s)
    {
        set(offset, s.begin(), s.end());
    }

    template <typename T>
    [[nodiscard]]
    std::enable_if_t<std::is_unsigned<T>::value, T> get(const std::size_t offset) const
    {
        assert((offset + sizeof(T)) <= buffer_size_);
        T out{};
        for (std::size_t i = 0; i < sizeof(T); i++)         // Dear compiler, consider unrolling this please.
        {
            out = T(out | (T(buffer_ptr_[offset + i]) << (i * 8U)));
        }
        return out;
    }

    template <typename T>
    [[nodiscard]]
    std::enable_if_t<std::is_signed<T>::value, T> get(const std::size_t offset) const
    {
        // This specialization handles all signed types including float and double.
        // Long double might fail at compile time if it is greater than uint64.
        using Unsigned = util::Unsigned<sizeof(T) * 8>;
        const Unsigned u = get<Unsigned>(offset);
        T out{};
        (void) std::memcpy(&out, &u, sizeof(T));  // Copy because we don't want to violate strict aliasing rules
        return out;
    }

    template <typename OutputIterator>
    void get(std::size_t offset, OutputIterator begin, const OutputIterator end) const
    {
        while (begin != end)
        {
            assert(offset < buffer_size_);
            *begin = buffer_ptr_[offset];
            ++offset;
            ++begin;
        }
    }

    template <std::size_t Capacity>
    [[nodiscard]]
    util::FixedCapacityString<Capacity> getASCIIString(const std::size_t offset) const
    {
        util::FixedCapacityString<Capacity> out;

        for (std::size_t i = 0; i < Capacity; i++)
        {
            const auto index = offset + i;
            if (index >= buffer_size_)
            {
                break;
            }

            const auto c = buffer_ptr_[index];
            if ((c > 0) && (c < 127))
            {
                out += char(c);
            }
            else
            {
                break;
            }
        }

        return out;
    }

    template <std::size_t Capacity>
    [[nodiscard]]
    std::size_t getASCIIStringLength(const std::size_t offset) const
    {
        std::size_t out = 0;
        for (out = 0; out < Capacity; out++)
        {
            const auto index = offset + out;
            if (index >= buffer_size_)
            {
                break;
            }

            const auto c = buffer_ptr_[index];
            if ((c > 0) && (c < 127))
            {
                out++;
            }
            else
            {
                break;
            }
        }

        return out;
    }

    /**
     * Returns the size of the underlying buffer, which may be larger than the serialized message itself.
     */
    [[nodiscard]] std::size_t getUnderlyingBufferSize() const { return buffer_size_; }

    /**
     * Returns an immutable pointer to the underlying buffer.
     */
    [[nodiscard]] const std::uint8_t* getUnderlyingBufferPointer() const { return buffer_ptr_; }

    /**
     * Returns a new codec object that points to the same underlying buffer, but with a specified offset.
     */
    [[nodiscard]]
    ScalarCodec getSubCodec(std::size_t offset) const
    {
        assert(offset <= buffer_size_);
        return ScalarCodec(buffer_ptr_ + offset, buffer_size_ - offset);
    }

    /**
     * Returns an instance of Emitter that is bound to the underlying buffer.
     * Size must be provided explicitly, because the final message may be smaller than the buffer.
     */
    [[nodiscard]]
    transport::Emitter makeEmitter(const std::uint8_t frame_type_code, const std::size_t size) const
    {
        return transport::Emitter(frame_type_code, buffer_ptr_, size);
    }
};

/// Shortcuts; use these rather than the template directly.
using ScalarEncoder = ScalarCodec<Mutability::Mutable>;
using ScalarDecoder = ScalarCodec<Mutability::Immutable>;

}   // namespace presentation

/**
 * Standard messages.
 */
namespace standard
{
/**
 * Every message is prepended by this many bytes.
 */
static constexpr std::size_t MessageHeaderSize = 8;

/**
 * Default request timeout applicable to standard messages.
 */
static constexpr std::chrono::seconds DefaultStandardRequestTimeout = std::chrono::seconds(1);

/**
 * This class represents a standard frame header.
 * It can be used to parse and generate standard frames.
 * Layout (note that all fields are aligned, no padding necessary):
 *
 *    Offset    Type    Name
 *  ---------------------------------------------------
 *      0       u16     message_id
 *      2       u8[6]   <reserved>
 *  ---------------------------------------------------
 *      8                                                   All standard messages are aligned at 8 bytes.
 */
template <typename ScalarCodec = presentation::ScalarEncoder>
class MessageHeaderView final
{
    static constexpr std::size_t MessageIDOffset = 0;

    ScalarCodec codec_;

    ScalarCodec getPayloadCodec() const
    {
        return codec_.getSubCodec(MessageHeaderSize);
    }

public:
    explicit MessageHeaderView(const ScalarCodec& sc) : codec_(sc) { }

    /**
     * Returns true if this is valid header. Reject the message if the header is invalid.
     */
    [[nodiscard]]
    bool isValid() const
    {
        return codec_.getUnderlyingBufferSize() >= MessageHeaderSize;
    }

    /**
     * Returns the message ID set in this header.
     */
    [[nodiscard]]
    std::uint16_t getMessageID() const
    {
        if (isValid())
        {
            return codec_.template get<std::uint16_t>(MessageIDOffset);
        }
        else
        {
            return 0;
        }
    }

    /**
     * Messages with empty payload are typically used to request data from remote nodes.
     */
    [[nodiscard]]
    bool isMessagePayloadEmpty() const
    {
        if (isValid())
        {
            return getPayloadCodec().getUnderlyingBufferSize() == 0;
        }
        else
        {
            return true;
        }
    }

    /**
     * Returns true if the message ID of the supplied message type matches the message ID encoded in the header.
     */
    template <template <typename> class MessageViewType>
    [[nodiscard]]
    bool doesMessageIDMatch() const
    {
        if (isValid())
        {
            return MessageViewType<ScalarCodec>::MessageID == getMessageID();
        }
        else
        {
            return false;
        }
    }

    /**
     * Returns an instance of message codec of the specified type bound to the underlying buffer of this header.
     * Returns an empty option if the ID of the current message does not match the ID of the provided message
     * codec type.
     */
    template <template <typename> class MessageViewType>
    [[nodiscard]]
    std::optional<MessageViewType<ScalarCodec>> getMessageView() const
    {
        using Type = MessageViewType<ScalarCodec>;
        if (isValid() && (getMessageID() == Type::MessageID))
        {
            return Type(getPayloadCodec());
        }
        else
        {
            return {};
        }
    }

    /**
     * Use this method to create new messages.
     * It accepts the message template and the scalar codec that will be used to allocate the message in.
     */
    template <template <typename> class MessageViewType>
    [[nodiscard]]
    MessageViewType<ScalarCodec> emplaceMessage()
    {
        using Type = MessageViewType<ScalarCodec>;
        assert(isValid());
        codec_.template set<std::uint16_t>(MessageIDOffset, Type::MessageID);
        return Type(codec_.getSubCodec(MessageHeaderSize));
    }
};

/**
 * This simple helper class simplifies construction of new messages.
 */
template <template <typename> class MessageViewTemplate, std::size_t MaxSerializedSize>
class MessageConstructionHelper : public MessageViewTemplate<presentation::ScalarEncoder>
{
    // The buffer MUST be zero-initialized!
    std::array<std::uint8_t, MessageHeaderSize + MaxSerializedSize> buffer_{};

public:
    MessageConstructionHelper() :
        MessageViewTemplate<presentation::ScalarEncoder>(MessageHeaderView(presentation::ScalarEncoder(buffer_)).
            template emplaceMessage<MessageViewTemplate>())
    { }

    [[nodiscard]] const auto& getSerializedMessageWithHeader() const { return buffer_; }
};

/**
 * Node info message representation.
 * This class performs encoding/decoding in/out the scalar codec on the fly,
 * so data access via its methods can be very slow.
 *
 *      Offset  Type        Name
 *  ---------------------------------------------------
 *      0       u64         software_image_crc
 *      8       u32         software_vcs_commit_id
 *      12      u32         software_build_timestamp_utc        UTC Unix time in seconds
 *      16      u8          software_version_major
 *      17      u8          software_version_minor
 *      18      u8          hardware_version_major
 *      19      u8          hardware_version_minor
 *      20      u8          flags                               1 - SW CRC set, 2 - SW release, 4 - SW dirty build
 *      21      u8          mode                                0 - normal, 1 - bootloader
 *              u8[2]       <reserved>
 *      24      u8[16]      globally_unique_id
 *      40      u8[80]      node_name
 *      120     u8[80]      node_description
 *      200     u8[80]      build_environment_description
 *      280     u8[80]      runtime_environment_description
 *      360     u8[<=255]   certificate_of_authenticity         Until the end of the message
 *  ---------------------------------------------------
 *      <=615
 */
template <typename ScalarCodec = presentation::ScalarEncoder>
class NodeInfoView
{
    static constexpr std::uint8_t FlagSoftwareImageCRCAvailable = 1;
    static constexpr std::uint8_t FlagSoftwareReleaseBuild      = 2;
    static constexpr std::uint8_t FlagSoftwareDirtyBuild        = 4;

    struct Offsets
    {
        static constexpr std::size_t SoftwareImageCRC                =   0;
        static constexpr std::size_t SoftwareVCSCommitID             =   8;
        static constexpr std::size_t SoftwareBuildTimestampUTC       =  12;
        static constexpr std::size_t SoftwareVersionMajorMinor       =  16;
        static constexpr std::size_t HardwareVersionMajorMinor       =  18;
        static constexpr std::size_t Flags                           =  20;
        static constexpr std::size_t Mode                            =  21;
        static constexpr std::size_t GloballyUniqueID                =  24;
        static constexpr std::size_t NodeName                        =  40;
        static constexpr std::size_t NodeDescription                 = 120;
        static constexpr std::size_t BuildEnvironmentDescription     = 200;
        static constexpr std::size_t RuntimeEnvironmentDescription   = 280;
        static constexpr std::size_t CertificateOfAuthenticity       = 360;
    };

    ScalarCodec codec_;

    void setSingleFlag(const std::uint8_t x)
    {
        codec_.template set<std::uint8_t>(Offsets::Flags,
                                          std::uint8_t(codec_.template get<std::uint8_t>(Offsets::Flags) | x));
    }

    void clearSingleFlag(const std::uint8_t x)
    {
        codec_.template set<std::uint8_t>(Offsets::Flags,
                                          std::uint8_t(codec_.template get<std::uint8_t>(Offsets::Flags) & ~x));
    }

    bool checkSingleFlag(const std::uint8_t x) const
    {
        return (codec_.template get<std::uint8_t>(Offsets::Flags) & x) == x;
    }

public:
    static constexpr std::uint8_t MessageID = 0;   ///< ID of this message structure

    explicit NodeInfoView(const ScalarCodec& sc) : codec_(sc) { }

    std::optional<std::uint64_t> getSoftwareImageCRCIfAvailable() const
    {
        if (checkSingleFlag(FlagSoftwareImageCRCAvailable))
        {
            return codec_.template get<std::uint64_t>(Offsets::SoftwareImageCRC);
        }
        else
        {
            return {};
        }
    }

    void setSoftwareImageCRC(const std::uint64_t x)
    {
        codec_.set(Offsets::SoftwareImageCRC, x);
        setSingleFlag(FlagSoftwareImageCRCAvailable);
    }

    void clearSoftwareImageCRC()
    {
        codec_.template set<std::uint64_t>(Offsets::SoftwareImageCRC, 0);   // This is not mandatory
        clearSingleFlag(FlagSoftwareImageCRCAvailable);
    }

    std::uint32_t getSoftwareVCSCommitID() const
    {
        return codec_.template get<std::uint32_t>(Offsets::SoftwareVCSCommitID);
    }

    void setSoftwareVCSCommitID(const std::uint32_t x)
    {
        codec_.set(Offsets::SoftwareVCSCommitID, x);
    }

    std::uint32_t getSoftwareBuildTimestampUTC() const
    {
        return codec_.template get<std::uint32_t>(Offsets::SoftwareBuildTimestampUTC);
    }

    void setSoftwareBuildTimestampUTC(const std::uint32_t x)
    {
        codec_.set(Offsets::SoftwareBuildTimestampUTC, x);
    }

    struct Version
    {
        std::uint8_t major = 0;
        std::uint8_t minor = 0;
    };

    Version getSoftwareVersion() const
    {
        return {
            codec_.template get<std::uint8_t>(Offsets::SoftwareVersionMajorMinor + 0),
            codec_.template get<std::uint8_t>(Offsets::SoftwareVersionMajorMinor + 1)
        };
    }

    void setSoftwareVersion(const Version x)
    {
        codec_.set(Offsets::SoftwareVersionMajorMinor + 0, x.major);
        codec_.set(Offsets::SoftwareVersionMajorMinor + 1, x.minor);
    }

    Version getHardwareVersion() const
    {
        return {
            codec_.template get<std::uint8_t>(Offsets::HardwareVersionMajorMinor + 0),
            codec_.template get<std::uint8_t>(Offsets::HardwareVersionMajorMinor + 1)
        };
    }

    void setHardwareVersion(const Version x)
    {
        codec_.set(Offsets::HardwareVersionMajorMinor + 0, x.major);
        codec_.set(Offsets::HardwareVersionMajorMinor + 1, x.minor);
    }

    bool getSoftwareReleaseBuildFlag() const
    {
        return checkSingleFlag(FlagSoftwareReleaseBuild);
    }

    void setSoftwareReleaseBuildFlag(const bool x)
    {
        if (x)
        {
            setSingleFlag(FlagSoftwareReleaseBuild);
        }
        else
        {
            clearSingleFlag(FlagSoftwareReleaseBuild);
        }
    }

    bool getSoftwareDirtyBuildFlag() const
    {
        return checkSingleFlag(FlagSoftwareDirtyBuild);
    }

    void setSoftwareDirtyBuildFlag(const bool x)
    {
        if (x)
        {
            setSingleFlag(FlagSoftwareDirtyBuild);
        }
        else
        {
            clearSingleFlag(FlagSoftwareDirtyBuild);
        }
    }

    enum class Mode : std::uint8_t
    {
        Normal,
        Bootloader,
    };

    Mode getMode() const
    {
        const std::uint8_t m = codec_.template get<std::uint8_t>(Offsets::Mode);
        switch (m)
        {
        case std::uint8_t(Mode::Normal):
        case std::uint8_t(Mode::Bootloader):
        {
            return Mode(m);
        }
        default:
        {
            return Mode::Normal;    // Fallback case
        }
        }
    }

    void setMode(const Mode m)
    {
        switch (m)
        {
        case Mode::Bootloader:
        case Mode::Normal:
        {
            codec_.set(Offsets::Mode, std::uint8_t(m));
            return;
        }
        }

        // Default case - do nothing (do not set invalid values)
        assert(false);
    }

    std::array<std::uint8_t, 16> getGloballyUniqueID() const
    {
        std::array<std::uint8_t, 16> out{};
        codec_.get(Offsets::GloballyUniqueID, out.begin(), out.end());
        return out;
    }

    void setGloballyUniqueID(const std::array<std::uint8_t, 16>& uid)
    {
        codec_.set(Offsets::GloballyUniqueID, uid.begin(), uid.end());
    }

    util::FixedCapacityString<80> getNodeName() const
    {
        return codec_.template getASCIIString<80>(Offsets::NodeName);
    }

    void setNodeName(const util::FixedCapacityString<80>& s)
    {
        codec_.setASCIIString(Offsets::NodeName, s);
    }

    util::FixedCapacityString<80> getNodeDescription() const
    {
        return codec_.template getASCIIString<80>(Offsets::NodeDescription);
    }

    void setNodeDescription(const util::FixedCapacityString<80>& s)
    {
        codec_.setASCIIString(Offsets::NodeDescription, s);
    }

    util::FixedCapacityString<80> getBuildEnvironmentDescription() const
    {
        return codec_.template getASCIIString<80>(Offsets::BuildEnvironmentDescription);
    }

    void setBuildEnvironmentDescription(const util::FixedCapacityString<80>& s)
    {
        codec_.setASCIIString(Offsets::BuildEnvironmentDescription, s);
    }

    util::FixedCapacityString<80> getRuntimeEnvironmentDescription() const
    {
        return codec_.template getASCIIString<80>(Offsets::RuntimeEnvironmentDescription);
    }

    void setRuntimeEnvironmentDescription(const util::FixedCapacityString<80>& s)
    {
        codec_.setASCIIString(Offsets::RuntimeEnvironmentDescription, s);
    }

    template <typename Storage, typename = decltype(std::declval<Storage>().size())>
    void setCertificateOfAuthenticity(const Storage& s)
    {
        std::size_t len = s.size();
        assert(len <= 255);
        len = std::min<std::size_t>(255, len);

        codec_.set(Offsets::CertificateOfAuthenticity, std::begin(s), std::begin(s) + len);
    }

    /**
     * The output iterator must point to a storage that can accommodate at least 255 bytes of data.
     */
    template <typename RandomAccessOutputIterator>
    std::uint8_t getCertificateOfAuthenticity(const RandomAccessOutputIterator out) const
    {
        const auto len = std::min<std::size_t>(255,
                                               codec_.getUnderlyingBufferSize() - Offsets::CertificateOfAuthenticity);
        codec_.get(Offsets::CertificateOfAuthenticity, out, out + len);
        return std::uint8_t(len);
    }

    static constexpr std::size_t getSerializedSize(const std::uint8_t certificate_of_authenticity_length)
    {
        return Offsets::CertificateOfAuthenticity + certificate_of_authenticity_length;
    }
};

/**
 * Register name type - an ASCII string with a bounded size.
 */
using RegisterName = util::FixedCapacityString<93>;

/**
 * Typesafe alias for an unstructured array of bytes.
 */
struct RegisterValueUnstructured : public util::FixedCapacityVector<std::uint8_t, 256> { };

/**
 * Typesafe alias for an array of boolean values.
 */
struct RegisterValueBoolean : public util::FixedCapacityVector<bool, 256> { };

/**
 * This big variant defines all possible values that can be contained in a register.
 */
using RegisterValue = std::variant<
    std::monostate,                      ///< Empty value
    util::FixedCapacityString<256>,
    RegisterValueUnstructured,
    RegisterValueBoolean,
    // Signed integer values
    util::FixedCapacityVector<std::int64_t, 32>,
    util::FixedCapacityVector<std::int32_t, 64>,
    util::FixedCapacityVector<std::int16_t, 128>,
    util::FixedCapacityVector<std::int8_t,  256>,
    // Unsigned integer values
    util::FixedCapacityVector<std::uint64_t, 32>,
    util::FixedCapacityVector<std::uint32_t, 64>,
    util::FixedCapacityVector<std::uint16_t, 128>,
    util::FixedCapacityVector<std::uint8_t,  256>,
    // Floating point values
    util::FixedCapacityVector<double, 32>,
    util::FixedCapacityVector<float, 64>
>;

/**
 * This is not a message class. Rather, it is a base class for the following message classes defined below.
 *
 * It can be used by the application as well, but it can't be sent or received over the wire.
 * Use the derived classes instead.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u8              type_id         @ref RegisterTypeID
 *      1       u8[<=93]        name            ASCII name, zero-terminated.
 *      1...94  u8[<=256]       encoded_payload Array of values whose types are defined by type_id.
 *  -----------------------------------------------------------------------------------------------
 *    <=350
 */
template <typename ScalarCodec = presentation::ScalarEncoder>
class RegisterData
{
    ScalarCodec codec_;

    struct Offsets
    {
        static constexpr std::size_t TypeID = 0;
        static constexpr std::size_t Name   = 1;
    };

    static constexpr std::size_t MaxPayloadSize = 256;

    enum class RegisterTypeID : std::uint8_t
    {
        Empty,              ///< No value is provided
        String,             ///< UTF-8 encoded string of text
        Unstructured,       ///< Raw unstructured bytes
        Boolean,            ///< One byte per value; 0 - false, 1...255 - true
        // Signed integers
        I64,
        I32,
        I16,
        I8,
        // Unsigned integers
        U64,
        U32,
        U16,
        U8,
        // IEEE754 floating point
        F64,
        F32,
    };
    static constexpr std::uint8_t NumberOfRegisterTypes = 14;

    RegisterTypeID getTypeID() const
    {
        if (codec_.getUnderlyingBufferSize() > Offsets::TypeID)
        {
            const std::uint8_t value = codec_.template get<std::uint8_t>(Offsets::TypeID);
            if (value < NumberOfRegisterTypes)
            {
                return RegisterTypeID(value);
            }
        }
        return RegisterTypeID::Empty;
    }

    void setTypeID(const RegisterTypeID type_id)
    {
        codec_.set(Offsets::TypeID, std::uint8_t(type_id));
    }

    std::size_t getValueOffset() const
    {
        return Offsets::Name + codec_.template getASCIIStringLength<RegisterName::Capacity>(Offsets::Name);
    }

    template <typename T>
    auto copyOut() const
    {
        constexpr std::size_t ItemSize = sizeof(T);
        constexpr std::size_t NumberOfItems = MaxPayloadSize / ItemSize;
        static_assert(NumberOfItems * ItemSize == MaxPayloadSize);

        const auto value_codec = codec_.getSubCodec(getValueOffset());

        util::FixedCapacityVector<T, NumberOfItems> out;
        std::size_t offset = 0;
        while (offset < value_codec.getUnderlyingBufferSize())
        {
            out.push_back(value_codec.template get<T>(offset));
            offset += ItemSize;
        }

        return out;
    }

public:
    static constexpr std::size_t MaxSerializedSize = 1 + RegisterName::Capacity + MaxPayloadSize;

    explicit RegisterData(const ScalarCodec& sc) : codec_(sc) { }

    [[nodiscard]]
    RegisterName getName() const
    {
        return codec_.template getASCIIString<RegisterName::Capacity>(Offsets::Name);
    }

    void setName(const RegisterName& name)
    {
        // We must store the value temporarily because its offset is not fixed
        const RegisterValue value = getValue();
        codec_.setASCIIString(Offsets::Name, name);
        setValue(value);
    }

    [[nodiscard]]
    RegisterValue getValue() const
    {
        switch (getTypeID())
        {
        case RegisterTypeID::Empty:  { return std::monostate(); }
        case RegisterTypeID::String: { return codec_.template getASCIIString<MaxPayloadSize>(getValueOffset()); }

        case RegisterTypeID::Unstructured:
        {
            const auto value_codec = codec_.getSubCodec(getValueOffset());
            RegisterValueUnstructured out;
            std::copy_n(value_codec.getUnderlyingBufferPointer(),
                        value_codec.getUnderlyingBufferSize(),
                        std::back_inserter(out));
            return out;
        }

        case RegisterTypeID::Boolean:
        {
            const auto value_codec = codec_.getSubCodec(getValueOffset());
            RegisterValueBoolean out;
            for (std::size_t i = 0; i < value_codec.getUnderlyingBufferSize(); i++)
            {
                out.push_back(value_codec.template get<std::uint8_t>(i) != 0);
            }
            return out;
        }

        case RegisterTypeID::I64: { return copyOut<std::int64_t>(); }
        case RegisterTypeID::I32: { return copyOut<std::int32_t>(); }
        case RegisterTypeID::I16: { return copyOut<std::int16_t>(); }
        case RegisterTypeID::I8:  { return copyOut<std::int8_t>(); }

        case RegisterTypeID::U64: { return copyOut<std::uint64_t>(); }
        case RegisterTypeID::U32: { return copyOut<std::uint32_t>(); }
        case RegisterTypeID::U16: { return copyOut<std::uint16_t>(); }
        case RegisterTypeID::U8:  { return copyOut<std::uint8_t>(); }

        case RegisterTypeID::F64:
        {
            assert(sizeof(double) == 8);
            return copyOut<double>();
        }

        case RegisterTypeID::F32:
        {
            assert(sizeof(float) == 4);
            return copyOut<float>();
        }
        }

        assert(false);
        return {};
    }

    void setValue(const RegisterValue& value)
    {
        (void) value;
    }

    template <typename T>
    std::enable_if_t<std::is_integral_v<T> && !std::is_same_v<std::remove_cv_t<T>, bool>>
    setScalarValue(const T& value)
    {
        constexpr std::size_t ItemSize = sizeof(T);
        constexpr std::size_t NumberOfItems = MaxPayloadSize / ItemSize;
        static_assert(NumberOfItems * ItemSize == MaxPayloadSize);

        using ContainedType = util::FixedCapacityVector<T, NumberOfItems>;

        RegisterValue variant;
        variant.template emplace<ContainedType>();
        std::template get<ContainedType>(variant).push_back(value);
        setValue(variant);
    }

    template <typename T>
    void setScalarValue(const bool& value)
    {
        RegisterValue variant;
        variant.template emplace<RegisterValueBoolean>();
        std::template get<RegisterValueBoolean>(variant).push_back(value);
        setValue(variant);
    }
};

} // namespace standard

} // namespace popcop

#endif
