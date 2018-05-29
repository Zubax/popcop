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
#include <functional>
#include <algorithm>
#include <optional>
#include <iterator>
#include <variant>
#include <cstring>
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <chrono>
#include <array>

/*
 * A third-party dependency - Senoval - which is a simple header-only library of
 * C++17 classes for real-time embedded systems. The user is expected to make the
 * headers available for inclusion for Popcop.
 */
#include <senoval/string.hpp>
#include <senoval/vector.hpp>


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
 * This version uses an underlying buffer, and returns encoded data byte-by-byte.
 * There is also a more complex but more efficient version, see @ref StreamEmitter.
 */
class BufferedEmitter
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
    BufferedEmitter(std::uint8_t frame_type_code,
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
    BufferedEmitter(std::uint8_t frame_type_code,
                    const T& payload_object_ref,
                    std::size_t payload_size = sizeof(T)) :
        BufferedEmitter(frame_type_code,
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

/**
 * This emitter is a bit trickier than @ref BufferedEmitter, use with care!
 * It works through a SEQUENTIAL ACCESS OUTPUT ITERATOR. When a byte of data is written into the iterator,
 * the emitter automatically encodes it and sends it over the wire via the provided data sink (output callback).
 * This way, the latency is virtually zero, memory usage is minimal, and no data buffers are necessary.
 * The class automatically finalizes the emitted message when the instance is destroyed. This is why the class
 * can't be copyable - that would break the RAII paradigm. An attempt to copy an instance of it will trigger a
 * compile-time error, so it is safe in this regard.
 */
class StreamEmitter
{
    const std::uint8_t frame_type_code_;
    const std::function<void (std::uint8_t)> sink_;
    mutable CRCComputer crc_;

    void sinkWithEscaping(const std::uint8_t byte) const
    {
        if ((byte == FrameDelimiter) || (byte == EscapeCharacter))
        {
            sink_(EscapeCharacter);
            sink_(std::uint8_t(~byte));
        }
        else
        {
            sink_(byte);
        }
    }

public:
    /**
     * The proxy output iterator that is actually used to emit data.
     * Note that it must be copyable, unlike the class that constructs it!
     */
    class OutputIterator
    {
        friend class StreamEmitter;

        const StreamEmitter* owner_;

        explicit OutputIterator(const StreamEmitter* master) : owner_(master) { }

    public:
        /**
         * Parts of the Iterator concept.
         * http://en.cppreference.com/w/cpp/header/iterator
         * Note that std::iterator<> has been deprecated in C++17.
         */
        using iterator_category = std::output_iterator_tag;
        using value_type        = std::uint8_t;
        using difference_type   = void;
        using pointer           = void;
        using reference         = void;

        /**
         * Assignment to the output iterator invokes the write-out operation and the state machine update.
         */
        OutputIterator& operator=(std::uint8_t byte)
        {
            owner_->sinkWithEscaping(byte);
            owner_->crc_.add(byte);
            return *this;
        }

        /**
         * These are not defined for this type of iterator, but we have to provide the operators nevertheless.
         * Semantically this is similar to std::back_insert_iterator<>.
         */
        OutputIterator& operator*() { return *this; }
        OutputIterator& operator++() { return *this; }
        OutputIterator operator++(int) const { return *this; }
    };

    /**
     * Note that this is a RAII-type, so construction and destruction have significant side effects.
     * The preferred pattern is as follows:
     *
     *      encode(StreamEmitter(frame_type_code, my_callback).begin());
     *
     * In this way, usage is simple and the transfer is finalized as soon as possible.
     * More on lifetimes: https://stackoverflow.com/questions/584824/guaranteed-lifetime-of-temporary-in-c
     */
    StreamEmitter(std::uint8_t frame_type_code,
                  const std::function<void (std::uint8_t)>& sink) :
        frame_type_code_(frame_type_code),
        sink_(sink)
    {
        sink_(FrameDelimiter);
    }

    /**
     * Finalizes the transfer.
     * Make sure that the destructor is invoked before the next transfer is initiated!
     */
    ~StreamEmitter()
    {
        sinkWithEscaping(frame_type_code_);
        crc_.add(frame_type_code_);

        sinkWithEscaping(std::uint8_t(crc_.get() >> 0));
        sinkWithEscaping(std::uint8_t(crc_.get() >> 8));
        sinkWithEscaping(std::uint8_t(crc_.get() >> 16));
        sinkWithEscaping(std::uint8_t(crc_.get() >> 24));

        sink_(FrameDelimiter);
    }

    /**
     * Returns an iterator object that can be used to emit data via this emitter.
     */
    OutputIterator begin() const
    {
        return OutputIterator(this);
    }

    /**
     * This is a RAII class with strong side effects, therefore it is non-copyable.
     */
    StreamEmitter(const StreamEmitter&) = delete;
    StreamEmitter& operator=(const StreamEmitter&) = delete;
};

} // namespace transport

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
 * A simple helper class that writes out encoded data into a designated iterator.
 */
template <typename OutputByteIterator>
class StreamEncoder
{
    std::size_t length_ = 0;
    OutputByteIterator output_;

public:
    explicit StreamEncoder(OutputByteIterator begin) : output_(begin) { }

    template <std::size_t NumBytes, typename T>
    void addUnsignedInteger(const T& value)
    {
        static_assert(std::is_unsigned_v<T>,
                      "The type must be unsigned. If you're using an integer literal, add the U suffix to it.");
        for (std::size_t i = 0; i < NumBytes; i++)         // Dear compiler, consider unrolling this please.
        {
            *output_++ = std::uint8_t(value >> (i * 8U));
            length_++;
        }
    }

    template <std::size_t NumBytes, typename T>
    void addSignedInteger(const T& value)
    {
        static_assert(std::is_signed_v<T> && std::is_integral_v<T>);
        std::make_unsigned_t<T> u = 0;
        static_assert(sizeof(u) == sizeof(value));
        std::memcpy(&u, &value, sizeof(u));
        addUnsignedInteger<NumBytes>(u);
    }

    template <std::size_t NumBytes, typename T>
    void addIEEE754(const T& value)
    {
        if constexpr (NumBytes == 4)
        {
            static_assert(sizeof(float) == 4);
            const float f = float(value);
            std::uint32_t u = 0;
            static_assert(sizeof(u) == sizeof(f));
            std::memcpy(&u, &f, sizeof(u));
            addU32(u);
        }
        else
        {
            static_assert(NumBytes == 8, "Only 32-bit and 64-bit floating point formats are defined");
            static_assert(sizeof(double) == 8);
            const double f = double(value);
            std::uint64_t u = 0;
            static_assert(sizeof(u) == sizeof(f));
            std::memcpy(&u, &f, sizeof(u));
            addU64(u);
        }
    }

    template <typename T> void addU8(const T& x)  { addUnsignedInteger<1>(x); }
    template <typename T> void addU16(const T& x) { addUnsignedInteger<2>(x); }
    template <typename T> void addU32(const T& x) { addUnsignedInteger<4>(x); }
    template <typename T> void addU64(const T& x) { addUnsignedInteger<8>(x); }

    template <typename T> void addI8(const T& x)  { addSignedInteger<1>(x); }
    template <typename T> void addI16(const T& x) { addSignedInteger<2>(x); }
    template <typename T> void addI32(const T& x) { addSignedInteger<4>(x); }
    template <typename T> void addI64(const T& x) { addSignedInteger<8>(x); }

    template <typename T> void addF32(const T& x) { addIEEE754<4>(x); }
    template <typename T> void addF64(const T& x) { addIEEE754<8>(x); }

    void fillUpToOffset(const std::size_t offset, const std::uint8_t fill_value = 0)
    {
        assert(length_ <= offset);
        while (length_ < offset)
        {
            *output_++ = fill_value;
            length_++;
        }
    }

    template <typename Container, typename = decltype(std::begin(std::declval<Container>()))>
    void addBytes(const Container& cont)
    {
        for (auto b : cont)
        {
            *output_++ = std::uint8_t(b);
            length_++;
        }
    }

    std::size_t getOffset() const { return length_; }

    StreamEncoder<OutputByteIterator> makeNew() const
    {
        return StreamEncoder<OutputByteIterator>(output_);
    }
};

/**
 * The inverse of @ref StreamEncoder.
 */
template <typename InputByteIterator>
class StreamDecoder
{
    // We use a dummy argument because only partial specializations are allowed within a class scope
    template <std::size_t NumBytes, typename = void> struct UnsignedTypeSelector;
    template <typename Dummy> struct UnsignedTypeSelector<1, Dummy> { using T = std::uint8_t;  };
    template <typename Dummy> struct UnsignedTypeSelector<2, Dummy> { using T = std::uint16_t; };
    template <typename Dummy> struct UnsignedTypeSelector<4, Dummy> { using T = std::uint32_t; };
    template <typename Dummy> struct UnsignedTypeSelector<8, Dummy> { using T = std::uint64_t; };

    std::size_t length_ = 0;
    InputByteIterator input_;
    const InputByteIterator end_;

public:
    StreamDecoder(InputByteIterator begin, InputByteIterator end) :
        input_(begin),
        end_(end)
    { }

    template <std::size_t NumBytes>
    auto fetchUnsignedInteger()
    {
        using T = typename UnsignedTypeSelector<NumBytes>::T;
        T out = 0;
        for (std::size_t i = 0; (i < NumBytes) && (input_ != end_); i++)
        {
            out = T(out | (T(*input_++) << (i * 8U)));
            length_++;
        }
        return out;
    }

    template <std::size_t NumBytes>
    auto fetchSignedInteger()
    {
        const auto u = fetchUnsignedInteger<NumBytes>();
        std::make_signed_t<typename UnsignedTypeSelector<NumBytes>::T> s = 0;
        static_assert(sizeof(s) == sizeof(u));
        std::memcpy(&s, &u, sizeof(s));         // Remember about strict aliasing!
        return s;
    }

    template <std::size_t NumBytes>
    auto fetchIEEE754()
    {
        if constexpr (NumBytes == 4)
        {
            static_assert(sizeof(float) == 4);
            const auto u = fetchU32();
            float out{};
            static_assert(sizeof(u) == sizeof(out));
            std::memcpy(&out, &u, sizeof(out));
            return out;
        }
        else
        {
            static_assert(NumBytes == 8, "Only 32-bit and 64-bit floating point formats are defined");
            static_assert(sizeof(double) == 8);
            const auto u = fetchU64();
            double out{};
            static_assert(sizeof(u) == sizeof(out));
            std::memcpy(&out, &u, sizeof(out));
            return out;
        }
    }

    auto fetchU8()  { return fetchUnsignedInteger<1>(); }
    auto fetchU16() { return fetchUnsignedInteger<2>(); }
    auto fetchU32() { return fetchUnsignedInteger<4>(); }
    auto fetchU64() { return fetchUnsignedInteger<8>(); }

    auto fetchI8()  { return fetchSignedInteger<1>(); }
    auto fetchI16() { return fetchSignedInteger<2>(); }
    auto fetchI32() { return fetchSignedInteger<4>(); }
    auto fetchI64() { return fetchSignedInteger<8>(); }

    auto fetchF32() { return fetchIEEE754<4>(); }
    auto fetchF64() { return fetchIEEE754<8>(); }

    template <typename OutputByteIterator, typename = std::enable_if_t<!std::is_integral_v<OutputByteIterator>>>
    void fetchBytes(OutputByteIterator out_begin, const OutputByteIterator out_end)
    {
        while ((out_begin != out_end) && (input_ != end_))
        {
            *out_begin++ = *input_++;
            length_++;
        }
    }

    template <typename OutputByteIterator>
    void fetchBytes(OutputByteIterator out_begin, std::size_t amount)
    {
        while ((amount --> 0) && (input_ != end_))
        {
            *out_begin++ = *input_++;
            length_++;
        }
    }

    template <std::size_t Capacity>
    void fetchASCIIString(senoval::String<Capacity>& out_string)
    {
        out_string.clear();
        for (std::size_t i = 0 ; (i < Capacity) && (input_ != end_); i++)
        {
            const std::uint8_t byte = fetchU8();
            if (byte == 0)
            {
                break;
            }
            else
            {
                out_string.push_back(char(byte));
            }
        }
    }

    void skipUpToOffset(const std::size_t offset)
    {
        assert(length_ <= offset);
        while ((length_ < offset) && (input_ != end_))
        {
            input_++;
            length_++;
        }
    }

    std::size_t getOffset() const { return length_; }

    std::size_t getRemainingLength() const
    {
        return std::size_t(std::max<std::int64_t>(0, std::distance(input_, end_)));
    }

    StreamDecoder<InputByteIterator> makeNew() const
    {
        return StreamDecoder<InputByteIterator>(input_, end_);
    }
};

}   // namespace presentation

/**
 * Standard messages.
 */
namespace standard
{
/**
 * Default request timeout applicable to standard messages.
 */
static constexpr std::chrono::seconds DefaultStandardRequestTimeout = std::chrono::seconds(1);

/**
 * Time point representation used in standard Popcop messages. Application-specific messages can obviously use custom
 * time representation formats.
 * The timestamp is a 64-bit unsigned integer number of nanoseconds since some arbitrary point in time.
 * Assuming the worst case that the reference point is the beginning of the UNIX epoch, we can expect the
 * timestamp to roll over to zero on:
 *  1970 + (2**64 - 1) / 1e9 / 3600 / 24 / 365 = 2554.9
 */
using Timestamp = std::chrono::duration<std::uint64_t, std::nano>;

/**
 * ID of all standard messages are listed here.
 */
enum class MessageID : std::uint16_t
{
    EndpointInfo                    = 0,
    RegisterDataRequest             = 1,
    RegisterDataResponse            = 2,
    RegisterDiscoveryRequest        = 3,
    RegisterDiscoveryResponse       = 4,
    RegisterTraceSetupRequest       = 5,
    RegisterTraceSetupResponse      = 6,
    RegisterTraceEvent              = 7,
    DeviceManagementCommandRequest  = 8,
    DeviceManagementCommandResponse = 9,
    BootloaderStatusRequest         = 10,
    BootloaderStatusResponse        = 11,
    BootloaderImageDataRequest      = 12,
    BootloaderImageDataResponse     = 13,
};

/**
 * This class represents a standard frame header.
 * It can be used to parse and generate standard frames.
 *
 *    Offset    Type    Name
 *  ---------------------------------------------------
 *      0       u16     message_id
 *  ---------------------------------------------------
 *      2
 */
struct MessageHeader
{
    static constexpr std::size_t Size = 2;

    MessageID message_id{};

    MessageHeader() = default;

    explicit MessageHeader(MessageID mid) : message_id(mid) { }

    /**
     * Encodes the message into the provided stream encoder.
     * The encoder's underlying iterator can encode and emit the message on the fly - that would be highly efficient.
     */
    template <typename OutputIterator>
    void encode(presentation::StreamEncoder<OutputIterator>& encoder) const
    {
        encoder.addU16(std::uint16_t(message_id));
    }

    template <typename InputIterator>
    static std::optional<MessageHeader> tryDecode(presentation::StreamDecoder<InputIterator>& decoder)
    {
        if (decoder.getRemainingLength() >= Size)
        {
            MessageHeader hdr(MessageID(decoder.fetchU16()));
            return hdr;
        }
        else
        {
            return {};
        }
    }
};

/**
 * Static buffer type that can be used by fixed-size message definitions.
 */
template <std::size_t MessageSizeNotIncludingHeader>
using StaticMessageBuffer = std::array<std::uint8_t, MessageSizeNotIncludingHeader + MessageHeader::Size>;

/**
 * Resizeable buffer type that can be used by variable-length message definitions.
 */
template <std::size_t MaxMessageSizeNotIncludingHeader>
using DynamicMessageBuffer = senoval::Vector<std::uint8_t, MaxMessageSizeNotIncludingHeader + MessageHeader::Size>;

/**
 * Endpoint info message representation.
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
 *      40      u8[80]      endpoint_name
 *      120     u8[80]      endpoint_description
 *      200     u8[80]      build_environment_description
 *      280     u8[80]      runtime_environment_description
 *      360     u8[<=255]   certificate_of_authenticity         Until the end of the message
 *  ---------------------------------------------------
 *      <=615
 */
struct EndpointInfoMessage
{
    static constexpr std::size_t MinEncodedSize = 360;
    static constexpr std::size_t MaxEncodedSize = 615;

    struct SoftwareVersion
    {
        std::optional<std::uint64_t> image_crc;
        std::uint32_t vcs_commit_id = 0;
        std::uint32_t build_timestamp_utc = 0;
        std::uint8_t major = 0;
        std::uint8_t minor = 0;
        bool release_build = false;
        bool dirty_build = false;
    };

    struct HardwareVersion
    {
        std::uint8_t major = 0;
        std::uint8_t minor = 0;
    };

    enum class Mode : std::uint8_t
    {
        Normal,
        Bootloader,
    };

    using String = senoval::String<80>;

    /**
     * Message fields.
     * Note that an empty message is treated as request for endpoint info. There is a dedicated method for that.
     */
    SoftwareVersion software_version;
    HardwareVersion hardware_version;
    Mode mode{};
    std::array<std::uint8_t, 16> globally_unique_id{};
    String endpoint_name;
    String endpoint_description;
    String build_environment_description;
    String runtime_environment_description;
    senoval::Vector<std::uint8_t, 255> certificate_of_authenticity;

    /**
     * True if the message is empty. An empty message is considered a request for endpoint info.
     */
    bool isRequest() const { return endpoint_name.empty() && endpoint_description.empty(); }

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder header_encoder(begin);
        MessageHeader(MessageID::EndpointInfo).encode(header_encoder);
        auto encoder = header_encoder.makeNew();     // Creating new encoder in order to exclude the header offset

        encoder.addU64(software_version.image_crc ? *software_version.image_crc : 0);
        encoder.addU32(software_version.vcs_commit_id);
        encoder.addU32(software_version.build_timestamp_utc);
        encoder.addU8(software_version.major);
        encoder.addU8(software_version.minor);

        encoder.addU8(hardware_version.major);
        encoder.addU8(hardware_version.minor);

        {
            std::uint8_t flags = 0;

            if (software_version.image_crc)
            {
                flags |= 1;
            }

            if (software_version.release_build)
            {
                flags |= 2;
            }

            if (software_version.dirty_build)
            {
                flags |= 4;
            }

            encoder.addU8(flags);
        }

        encoder.addU8(std::uint8_t(mode));
        encoder.fillUpToOffset(24);
        encoder.addBytes(globally_unique_id);
        encoder.addBytes(endpoint_name);
        encoder.fillUpToOffset(120);
        encoder.addBytes(endpoint_description);
        encoder.fillUpToOffset(200);
        encoder.addBytes(build_environment_description);
        encoder.fillUpToOffset(280);
        encoder.addBytes(runtime_environment_description);
        encoder.fillUpToOffset(360);

        assert(encoder.getOffset() == MinEncodedSize);
        encoder.addBytes(certificate_of_authenticity);
        assert(encoder.getOffset() >= MinEncodedSize);
        assert(encoder.getOffset() <= MaxEncodedSize);
        assert(encoder.getOffset() ==  MinEncodedSize + certificate_of_authenticity.size());

        return header_encoder.getOffset() + encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    DynamicMessageBuffer<MaxEncodedSize> encode() const
    {
        DynamicMessageBuffer<MaxEncodedSize> out;
        const std::size_t size = encode(std::back_inserter(out));
        (void) size;
        assert(size == out.size());
        return out;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     * If no message could be parsed, an empty optional<> will be returned.
     */
    template <typename InputIterator>
    static std::optional<EndpointInfoMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder header_decoder(begin, end);
        const auto header = MessageHeader::tryDecode(header_decoder);
        if (!header || (header->message_id != MessageID::EndpointInfo))
        {
            return {};
        }

        auto decoder = header_decoder.makeNew();     // Creating a new one to exclude header offset
        EndpointInfoMessage msg;
        if (decoder.getRemainingLength() < MinEncodedSize)
        {
            return msg;     // An empty message is considered a request for endpoint info.
        }

        if (decoder.getRemainingLength() > MaxEncodedSize)
        {
            return {};
        }

        msg.software_version.image_crc              = decoder.fetchU64();
        msg.software_version.vcs_commit_id          = decoder.fetchU32();
        msg.software_version.build_timestamp_utc    = decoder.fetchU32();
        msg.software_version.major                  = decoder.fetchU8();
        msg.software_version.minor                  = decoder.fetchU8();

        msg.hardware_version.major = decoder.fetchU8();
        msg.hardware_version.minor = decoder.fetchU8();

        {
            const std::uint8_t flags = decoder.fetchU8();

            if ((flags & 1) == 0)
            {
                msg.software_version.image_crc.reset();
            }

            msg.software_version.release_build = (flags & 2) != 0;
            msg.software_version.dirty_build   = (flags & 4) != 0;
        }

        switch (decoder.fetchU8())
        {
        case std::uint8_t(Mode::Normal):
        {
            msg.mode = Mode::Normal;
            break;
        }
        case std::uint8_t(Mode::Bootloader):
        {
            msg.mode = Mode::Bootloader;
            break;
        }
        default:
        {
            return {};
        }
        }

        decoder.skipUpToOffset(24);
        decoder.fetchBytes(msg.globally_unique_id.begin(),
                           msg.globally_unique_id.end());

        assert(decoder.getOffset() == 40);
        decoder.fetchASCIIString(msg.endpoint_name);
        decoder.skipUpToOffset(120);
        decoder.fetchASCIIString(msg.endpoint_description);
        decoder.skipUpToOffset(200);
        decoder.fetchASCIIString(msg.build_environment_description);
        decoder.skipUpToOffset(280);
        decoder.fetchASCIIString(msg.runtime_environment_description);
        decoder.skipUpToOffset(360);
        decoder.fetchBytes(std::back_inserter(msg.certificate_of_authenticity),
                           decoder.getRemainingLength());
        return msg;
    }
};

/**
 * This is not a message class.
 * This is a serializable register name that can be used in register manipulation messages.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u8              length          Length of the next field.
 *      1       u8[<=93]        name            ASCII name, not terminated - see the previous field.
 *  -----------------------------------------------------------------------------------------------
 *    <=94
 */
struct RegisterName : public senoval::String<93>
{
    using Base = senoval::String<93>;

    static constexpr std::size_t MinEncodedSize = 1;
    static constexpr std::size_t MaxEncodedSize = 94;

    using Base::String;    ///< All constructors are inherited
    using Base::operator=;

    /**
     * Optional high-level naming convention.
     * If the default value is defined for a register, it can be represented in a different register
     * that has the same name suffixed with '='. Minimum and maximum values can be represented likewise,
     * if necessary. For example:
     *  Register name: "foo", default value 42, minimum 12, maximum 72.
     *  Default value is stored in a read-only register "foo=" with the constant value 42.
     *  Minimum and maximum values are stored in registers named "foo<" and "foo>", respectively.
     */
    struct Suffix
    {
        static constexpr char DefaultValue = '=';
        static constexpr char MinimumValue = '<';
        static constexpr char MaximumValue = '>';
        // Other optional suffixes may be added in the future.
    };

    /**
     * Serializes the object into the provided encoder.
     */
    template <typename OutputIterator>
    void encode(presentation::StreamEncoder<OutputIterator>& encoder) const
    {
        encoder.addU8(std::uint8_t(this->length()));
        encoder.addBytes(*this);
    }

    /**
     * Attempts to reconstruct the state of the current object from the provided data stream.
     * Maximum size is not checked because this is not a greedy decoder, i.e. there may be more data after the string.
     */
    template <typename InputIterator>
    bool tryDecode(presentation::StreamDecoder<InputIterator>& decoder)
    {
        if (decoder.getRemainingLength() < MinEncodedSize)
        {
            return false;
        }

        const std::uint8_t name_len = decoder.fetchU8();
        if ((name_len > Capacity) ||
            (name_len > decoder.getRemainingLength()))
        {
            return false;
        }

        clear();
        for (std::uint8_t i = 0; i < name_len; i++)
        {
            push_back(char(decoder.fetchU8()));
        }

        return true;
    }
};

/// Implementation details; do not use that in user code
namespace detail_
{
/**
 * List of possible register value types. There are 14 of them, each has a distinct type ID starting from 0.
 * This type cannot be instantiated directly! It is only a type container.
 */
struct RegisterValueTypes
{
    /// No value; used to represent missing registers and requests for data
    using Empty = std::monostate;                                             ///< Type ID 0

    /// ASCII string, or UTF-8 encoded bytes
    using String = senoval::String<256>;                                      ///< Type ID 1

    /// A wrapper is needed rather than alias to create a distinct type for std::variant
    struct Unstructured : public senoval::Vector<std::uint8_t, 256>           ///< Type ID 2
    {
        using Base = senoval::Vector<std::uint8_t, 256>;
        Unstructured() = default;
        Unstructured(std::size_t len, const std::uint8_t* ptr) : Base(ptr, ptr + len) {}
    };

    /// A wrapper is needed rather than alias to create a distinct type for std::variant
    struct Boolean : public senoval::Vector<bool, 256>                        ///< Type ID 3
    {
        using Base = senoval::Vector<bool, 256>;
        using Base::Vector;
    };

    /// Signed integers
    using I64 = senoval::Vector<std::int64_t, 32>;                            ///< Type ID 4
    using I32 = senoval::Vector<std::int32_t, 64>;                            ///< Type ID 5
    using I16 = senoval::Vector<std::int16_t, 128>;                           ///< Type ID 6
    using I8  = senoval::Vector<std::int8_t,  256>;                           ///< Type ID 7

    /// Unsigned integers
    using U64 = senoval::Vector<std::uint64_t, 32>;                           ///< Type ID 8
    using U32 = senoval::Vector<std::uint32_t, 64>;                           ///< Type ID 9
    using U16 = senoval::Vector<std::uint16_t, 128>;                          ///< Type ID 10
    using U8  = senoval::Vector<std::uint8_t,  256>;                          ///< Type ID 11

    /// IEEE754 floating point
    using F64 = senoval::Vector<double, 32>;                                  ///< Type ID 12
    using F32 = senoval::Vector<float,  64>;                                  ///< Type ID 13

    /**
     * All value types represented as a single algebraic type (tagged union).
     * Note that the order of declaration matters, because it defines the type ID mappings.
     */
    using Variant = std::variant<
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
        F32
    >;

private:
    template <typename Scalar>
    struct ValueTypeForScalarImpl;

public:
    /**
     * This convenience type reduces the amount of boilerplate needed to map types from simple
     * scalar types to value vector types.
     */
    template <typename Scalar>
    using ValueTypeForScalar = typename ValueTypeForScalarImpl<Scalar>::T;

protected:  // This type cannot be instantiated directly
    RegisterValueTypes() = default;
    ~RegisterValueTypes() = default;

    /// The number of bytes the value is allowed to take in the encoded form.
    static constexpr std::size_t MaxEncodedValueSize = 256;
};

template <> struct RegisterValueTypes::ValueTypeForScalarImpl<bool> { using T = Boolean; };

template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::int64_t> { using T = I64; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::int32_t> { using T = I32; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::int16_t> { using T = I16; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::int8_t>  { using T = I8;  };

template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::uint64_t> { using T = U64; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::uint32_t> { using T = U32; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::uint16_t> { using T = U16; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<std::uint8_t>  { using T = U8;  };

template <> struct RegisterValueTypes::ValueTypeForScalarImpl<double> { using T = F64; };
template <> struct RegisterValueTypes::ValueTypeForScalarImpl<float>  { using T = F32;  };

} // namespace detail_

/**
 * This is not a message class.
 * This is a serializable representation of a register value that can be used in register manipulation messages.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u8              type_id         Type of the value contained in this register.
 *      1       u8[<=256]       encoded_payload Array of values whose types are defined by type_id.
 *  -----------------------------------------------------------------------------------------------
 *    <=257
 */
struct RegisterValue : public detail_::RegisterValueTypes::Variant,
                       public detail_::RegisterValueTypes
{
    /**
     * Alias of the underlying std::variant<>
     */
    using Variant = detail_::RegisterValueTypes::Variant;

    /**
     * Size limits of the encoded representation.
     */
    static constexpr std::size_t MinEncodedSize = 1;
    static constexpr std::size_t MaxEncodedSize = 257;

    /**
     * Number of types that are defined for this std::variant<>.
     */
    static constexpr std::uint8_t NumberOfVariants = std::uint8_t(std::variant_size_v<Variant>);

    /**
     * Maps variant index to type at compile time.
     */
    template <std::size_t VariantIndex>
    using VariantTypeAtIndex = std::variant_alternative_t<VariantIndex, Variant>;

    /**
     * All constructors and assignment operators are inherited from std::variant<>.
     */
    using Variant::variant;
    using Variant::operator=;

    /**
     * Shortcut for std::holds_alternative<T>(*this).
     * Usage:
     *  if (msg.is<RegisterData::U64>())
     *  {
     *      // Message contains an array of uint64
     *  }
     */
    template <typename T> [[nodiscard]] bool is() const { return std::holds_alternative<T>(*this); }

    /**
     * Shortcut for std::get_if<T>(this).
     * Usage:
     *  if (auto value = msg.as<RegisterData::U64>())
     *  {
     *      // Message contains an array of uint64; the pointer to that array is now stored in 'value'
     *  }
     */
    template <typename T> [[nodiscard]]       T* as()       { return std::get_if<T>(this); }
    template <typename T> [[nodiscard]] const T* as() const { return std::get_if<T>(this); }

    /**
     * A simple wrapper over std::visit<>(), refer to that for more info.
     */
    template <typename Visitor>
    auto visit(Visitor&& vis) const
    {
        return std::visit(std::forward<Visitor>(vis), *static_cast<const Variant*>(this));
    }
    template <typename Visitor>
    auto visit(Visitor&& vis)
    {
        return std::visit(std::forward<Visitor>(vis), *static_cast<Variant*>(this));
    }

    /**
     * Serializes the object into the provided encoder.
     */
    template <typename OutputIterator>
    void encode(presentation::StreamEncoder<OutputIterator>& encoder) const
    {
        encoder.addU8(std::uint8_t(index()));
        visit(VariantEncoder(encoder));
    }

    /**
     * Attempts to reconstruct the state of the current object from the provided data stream.
     * Note that the function is greedy: it takes all remaining data in the stream.
     * It is therefore necessary to always allocate the value at the end of the message,
     * or prepend each value with a 16-bit length field and pass an accordingly truncated
     * stream into this function.
     */
    template <typename InputIterator>
    bool tryDecode(presentation::StreamDecoder<InputIterator>& decoder)
    {
        if (decoder.getRemainingLength() < MinEncodedSize)
        {
            emplace<Empty>();       // No payload is treated as empty value as a last resort (not required)
            return true;
        }

        if (decoder.getRemainingLength() > MaxEncodedSize)
        {
            return false;
        }

        const std::uint8_t type_id = decoder.fetchU8();
        if (type_id >= NumberOfVariants)
        {
            return false;
        }

        decodeByTypeID(decoder, type_id);
        return true;
    }

private:
    template <typename OutputIterator>
    class VariantEncoder
    {
        presentation::StreamEncoder<OutputIterator>& encoder_;

    public:
        explicit VariantEncoder(presentation::StreamEncoder<OutputIterator>& stream_encoder) :
            encoder_(stream_encoder)
        { }

        /// Empty value handler
        void operator()(const std::monostate&) const { }

        /// String handler
        void operator()(const String& str) const
        {
            encoder_.addBytes(str);
        }

        /// Integer vector handler
        template <typename T, std::size_t Capacity>
        std::enable_if_t<std::is_integral_v<T>>
        operator()(const senoval::Vector<T, Capacity>& vec) const
        {
            // The following construct cleverly avoids dependency on the native type sizes.
            static constexpr std::size_t EncodedItemSize = MaxEncodedValueSize / Capacity;
            for (const T& x : vec)
            {
                if constexpr (std::is_signed_v<T>)
                {
                    encoder_.template addSignedInteger<EncodedItemSize>(x);
                }
                else
                {
                    encoder_.template addUnsignedInteger<EncodedItemSize>(x);
                }
            }
        }

        /// Float vector handler
        template <typename T, std::size_t Capacity>
        std::enable_if_t<std::is_floating_point_v<T>>
        operator()(const senoval::Vector<T, Capacity>& vec) const
        {
            for (const T& x : vec)
            {
                encoder_.template addIEEE754<MaxEncodedValueSize / Capacity>(x);
            }
        }
    };

    template <typename InputByteIterator, std::uint8_t TypeIDCandidate = 0>
    void decodeByTypeID(presentation::StreamDecoder<InputByteIterator>& decoder, const std::uint8_t type_id)
    {
        if constexpr (TypeIDCandidate < NumberOfVariants)
        {
            if (type_id == TypeIDCandidate)
            {
                decodeSpecificValue(decoder, emplace<VariantTypeAtIndex<TypeIDCandidate>>());
            }
            else
            {
                decodeByTypeID<InputByteIterator, TypeIDCandidate + 1>(decoder, type_id);
            }
        }
        // Otherwise the type ID is wrong. The caller is supposed to catch that; we're going to leave the value Empty.
    }

    template <typename InputByteIterator, typename T, std::size_t Capacity>
    static void resizeVectorBeforeDecoding(presentation::StreamDecoder<InputByteIterator>& decoder,
                                           senoval::Vector<T, Capacity>& vector)
    {
        // The following construct cleverly avoids dependency on the native type sizes.
        static constexpr std::size_t EncodedItemSize = MaxEncodedValueSize / Capacity;
        const std::size_t NumItems = std::min(Capacity, decoder.getRemainingLength() / EncodedItemSize);
        vector.resize(NumItems);
    }

    /// Empty value handler
    template <typename InputByteIterator>
    static void decodeSpecificValue(presentation::StreamDecoder<InputByteIterator>&, std::monostate&) { }

    /// String handler
    template <typename InputByteIterator>
    static void decodeSpecificValue(presentation::StreamDecoder<InputByteIterator>& decoder,
                                    String& out_string)
    {
        decoder.fetchASCIIString(out_string);
    }

    /// Integer vector handler
    template <typename InputByteIterator, typename T, std::size_t Capacity>
    static std::enable_if_t<std::is_integral_v<T>>
    decodeSpecificValue(presentation::StreamDecoder<InputByteIterator>& decoder,
                        senoval::Vector<T, Capacity>& out_vector)
    {
        // The following construct cleverly avoids dependency on the native type sizes.
        static constexpr std::size_t EncodedItemSize = MaxEncodedValueSize / Capacity;
        resizeVectorBeforeDecoding(decoder, out_vector);
        for (auto& x : out_vector)
        {
            if constexpr (std::is_signed_v<T>)
            {
                x = decoder.template fetchSignedInteger<EncodedItemSize>();
            }
            else
            {
                // Cast is needed here to properly handle booleans
                x = T(decoder.template fetchUnsignedInteger<EncodedItemSize>());
            }
        }
    }

    /// Float vector handler
    template <typename InputByteIterator, typename T, std::size_t Capacity>
    static std::enable_if_t<std::is_floating_point_v<T>>
    decodeSpecificValue(presentation::StreamDecoder<InputByteIterator>& decoder,
                        senoval::Vector<T, Capacity>& out_vector)
    {
        resizeVectorBeforeDecoding(decoder, out_vector);
        for (auto& x : out_vector)
        {
            x = decoder.template fetchIEEE754<MaxEncodedValueSize / Capacity>();
        }
    }
};

/**
 * This is a simple composition of RegisterName and RegisterValue.
 * Register read request if the value is empty.
 * Register write request if the value is not empty.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u8              name_length     Length of the next field.
 *      1       u8[<=93]        name            ASCII name, not terminated - see the previous field.
 *      1..94   u8              type_id         Type of the value contained in this message.
 *      2..95   u8[<=256]       encoded_payload Array of values whose types are defined by type_id.
 *  -----------------------------------------------------------------------------------------------
 *    <=351
 */
struct RegisterDataRequestMessage
{
    static constexpr std::size_t MinEncodedSize = 2;
    static constexpr std::size_t MaxEncodedSize = 351;

    static constexpr MessageID ID = MessageID::RegisterDataRequest;

    /**
     * All fields of this message type.
     */
    RegisterName name;
    RegisterValue value;

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        name.encode(encoder);
        value.encode(encoder);
        assert(encoder.getOffset() >= (MinEncodedSize + MessageHeader::Size));
        assert(encoder.getOffset() <= (MaxEncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    DynamicMessageBuffer<MaxEncodedSize> encode() const
    {
        DynamicMessageBuffer<MaxEncodedSize> buf;
        const std::size_t size = encode(std::back_inserter(buf));
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<RegisterDataRequestMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        RegisterDataRequestMessage msg;
        if (!msg.name.tryDecode(decoder))
        {
            return {};
        }

        if (!msg.value.tryDecode(decoder))
        {
            return {};
        }

        return msg;
    }
};

/**
 * Representation of register flags.
 */
struct RegisterFlags
{
    using Type = std::uint8_t;

    static constexpr Type Mutable       = 1;
    static constexpr Type Persistent    = 2;

    Type value = 0;

    [[nodiscard]] bool isMutable()    const { return check<Mutable>(); }
    [[nodiscard]] bool isPersistent() const { return check<Persistent>(); }

    void setMutable(bool x)    { set<Mutable>(x); }
    void setPersistent(bool x) { set<Persistent>(x); }

private:
    template <Type F> [[nodiscard]] bool check() const { return (value & F) != 0; }

    template <Type F>
    void set(const bool x)
    {
        if (x) { value = Type(value |  F); }
        else   { value = Type(value & ~F); }
    }
};

/**
 * This is a simple composition of RegisterName and RegisterValue with some additional values.
 * Register does not exist if the value is empty.
 * Register exists and the response contains its data if the value is not empty.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u64             timestamp       Timestamp of the provided value.
 *      8       u8              flags           Register flags: 1 - mutable, 2 - persistent.
 *      9       u8              name_length     Length of the next field.
 *      10      u8[<=93]        name            ASCII name, not terminated - see the previous field.
 *      10..103 u8              type_id         Type of the value contained in this message.
 *      11..104 u8[<=256]       encoded_payload Array of values whose types are defined by type_id.
 *  -----------------------------------------------------------------------------------------------
 *    <=360
 */
struct RegisterDataResponseMessage
{
    static constexpr std::size_t MinEncodedSize = 11;
    static constexpr std::size_t MaxEncodedSize = 360;

    static constexpr MessageID ID = MessageID::RegisterDataResponse;

    /**
     * All fields of this message type.
     */
    Timestamp timestamp{};
    RegisterFlags flags;
    RegisterName name;
    RegisterValue value;

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU64(timestamp.count());
        encoder.addU8(flags.value);
        name.encode(encoder);
        value.encode(encoder);
        assert(encoder.getOffset() >= (MinEncodedSize + MessageHeader::Size));
        assert(encoder.getOffset() <= (MaxEncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    DynamicMessageBuffer<MaxEncodedSize> encode() const
    {
        DynamicMessageBuffer<MaxEncodedSize> buf;
        const std::size_t size = encode(std::back_inserter(buf));
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<RegisterDataResponseMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() < MinEncodedSize)
        {
            return {};
        }

        RegisterDataResponseMessage msg;
        msg.timestamp = Timestamp(decoder.fetchU64());
        msg.flags.value = decoder.fetchU8();

        if (!msg.name.tryDecode(decoder))
        {
            return {};
        }

        if (!msg.value.tryDecode(decoder))
        {
            return {};
        }

        return msg;
    }
};

/**
 * Request of register name by index. Used for discovery purposes only.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u16             register_index  Index of the register name of which is requested.
 *  -----------------------------------------------------------------------------------------------
 *      2
 */
struct RegisterDiscoveryRequestMessage
{
    static constexpr std::size_t EncodedSize = 2;

    static constexpr MessageID ID = MessageID::RegisterDiscoveryRequest;

    /**
     * All fields of this message type.
     */
    std::uint16_t index = 0;

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU16(index);
        assert(encoder.getOffset() == (EncodedSize + MessageHeader::Size));
        return EncodedSize + MessageHeader::Size;
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    StaticMessageBuffer<EncodedSize> encode() const
    {
        StaticMessageBuffer<EncodedSize> buf;
        (void) encode(buf.begin());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<RegisterDiscoveryRequestMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() != EncodedSize)
        {
            return {};
        }

        RegisterDiscoveryRequestMessage msg;
        msg.index = decoder.fetchU16();
        return msg;
    }
};

/**
 * Discovery response - contains the name of the register at the index. Used for discovery purposes only.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u16             register_index  Index of the provided register name.
 *      2       u8              name_length     Length of the next field.
 *      3       u8[<=93]        name            ASCII name, not terminated - see the previous field.
 *      3..96                                   reserved for future use
 *  -----------------------------------------------------------------------------------------------
 *      (max size unconstrained)
 */
struct RegisterDiscoveryResponseMessage
{
    static constexpr std::size_t MinEncodedSize = 3;
    static constexpr std::size_t MaxEncodedSize = 96;   ///< May be changed in the future, do not check when decoding

    static constexpr MessageID ID = MessageID::RegisterDiscoveryResponse;

    /**
     * All fields of this message type.
     */
    std::uint16_t index = 0;
    RegisterName name;

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU16(index);
        name.encode(encoder);
        assert(encoder.getOffset() >= (MinEncodedSize + MessageHeader::Size));
        assert(encoder.getOffset() <= (MaxEncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    DynamicMessageBuffer<MaxEncodedSize> encode() const
    {
        DynamicMessageBuffer<MaxEncodedSize> buf;
        const std::size_t size = encode(std::back_inserter(buf));
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<RegisterDiscoveryResponseMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() < MinEncodedSize)  // Max is not checked - extra data should be ignored
        {
            return {};
        }

        RegisterDiscoveryResponseMessage msg;
        msg.index = decoder.fetchU16();
        if (!msg.name.tryDecode(decoder))
        {
            return {};
        }

        return msg;
    }
};

/**
 * Standard generic device command set.
 * Commands should be idempotent whenever possible.
 * More can be added in the future.
 */
enum class DeviceManagementCommand : std::uint16_t
{
    Restart                     = 0,
    PowerOff                    = 1,
    LaunchBootloader            = 2,
    FactoryReset                = 3,
    PrintDiagnosticsBrief       = 4,
    PrintDiagnosticsVerbose     = 5,
};

/**
 * Generic device command message.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u16             command         Generic command code
 *  -----------------------------------------------------------------------------------------------
 *      2
 */
struct DeviceManagementCommandRequestMessage
{
    static constexpr std::size_t EncodedSize = 2;

    static constexpr MessageID ID = MessageID::DeviceManagementCommandRequest;

    /**
     * All fields of this message type.
     */
    DeviceManagementCommand command{};

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU16(std::uint16_t(command));
        assert(encoder.getOffset() == (EncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    StaticMessageBuffer<EncodedSize> encode() const
    {
        StaticMessageBuffer<EncodedSize> buf;
        const std::size_t size = encode(buf.begin());
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<DeviceManagementCommandRequestMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() != EncodedSize)
        {
            return {};
        }

        DeviceManagementCommandRequestMessage msg;
        msg.command = DeviceManagementCommand(decoder.fetchU16());

        return msg;
    }
};

/**
 * Generic device command response message.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u16             command         Generic command code copied from the request.
 *      2       u8              status          Command execution status.
 *  -----------------------------------------------------------------------------------------------
 *      3
 */
struct DeviceManagementCommandResponseMessage
{
    static constexpr std::size_t EncodedSize = 3;

    static constexpr MessageID ID = MessageID::DeviceManagementCommandResponse;

    /**
     * Command execution result.
     */
    enum class Status : std::uint8_t
    {
        Ok                  = 0,
        BadCommand          = 1,
        MaybeLater          = 2,
    };

    /**
     * All fields of this message type.
     */
    DeviceManagementCommand command{};
    Status status{};

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU16(std::uint16_t(command));
        encoder.addU8(std::uint8_t(status));
        assert(encoder.getOffset() == (EncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    StaticMessageBuffer<EncodedSize> encode() const
    {
        StaticMessageBuffer<EncodedSize> buf;
        const std::size_t size = encode(buf.begin());
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<DeviceManagementCommandResponseMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() != EncodedSize)
        {
            return {};
        }

        DeviceManagementCommandResponseMessage msg;
        msg.command = DeviceManagementCommand(decoder.fetchU16());
        msg.status  = Status(decoder.fetchU8());

        return msg;
    }
};

/**
 * All possible states of the generic bootloader API. See the following state machine diagram.
 *
 *
 *     No valid application found ###################### Valid application found
 *               /----------------# Bootloader started #----------\ /-------------------------------------------\
 *               |                ######################          | |                                           |
 *               v                                                v v  Boot delay expired                       |
 *         +-------------+                               +-----------+  (typically zero)  +-------------+       |
 *     /-->| NoAppToBoot |        /----------------------| BootDelay |------------------->| ReadyToBoot |       |
 *     |   +-------------+       /                       +-----------+                    +-------------+       |
 *     |          |             /                          |Boot cancelled                   |ReadyToBoot is    |
 *     |Upgrade   |<-----------/                           |e.g. received a state transition |an auxiliary      /
 *     |failed,   |Upgrade requested,                      |request to BootCancelled.        |state, it is     /
 *     |no valid  |e.g. received a state transition        v                                 |left automati-  /
 *     |image is  |request to AppUpgradeInProgress. +---------------+                        |cally ASAP.    /
 *     |now ava-  |<--------------------------------| BootCancelled |                        v              /
 *     |ilable    |                                 +---------------+                ###############       /
 *     |          v                                        ^                         # Booting the #      /
 *     | +----------------------+ Upgrade failed, but the  |                         # application #     /
 *     \-| AppUpgradeInProgress |--------------------------/                         ###############    /
 *       +----------------------+ existing valid image was not                                         /
 *                |               altered and remains valid.                                          /
 *                |                                                                                  /
 *                | Upgrade successful, received image is valid.                                    /
 *                \--------------------------------------------------------------------------------/
 */
enum class BootloaderState : std::uint8_t
{
    NoAppToBoot             = 0,
    BootDelay               = 1,
    BootCancelled           = 2,
    AppUpgradeInProgress    = 3,
    ReadyToBoot             = 4,
};

/**
 * The bootloader protocol supports several types of images that can be written.
 * Not all of them must be supported by the target.
 */
enum class BootloaderImageType : std::uint8_t
{
    Application                 = 0,
    CertificateOfAuthenticity   = 1,
};

/**
 * Bootloader status request; contains the desired status, the response will contain the actual new status.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u8              desired_state   Which state the bootloader should transition to.
 *  -----------------------------------------------------------------------------------------------
 *      1
 */
struct BootloaderStatusRequestMessage
{
    static constexpr std::size_t EncodedSize = 1;

    static constexpr MessageID ID = MessageID::BootloaderStatusRequest;

    /**
     * Only the following states can be commanded:
     *      NoAppToBoot             - erases the application (optional, not mandatory to support)
     *      AppUpgradeInProgress    - initiates the upgrade process (mandatory)
     *      ReadyToBoot             - commands to launch the application (if present) (mandatory)
     *      BootCancelled           - cancels the auto-boot timer (optional, not mandatory to support)
     * All other states cannot be used here. If they are, the bootloader is free to choose the most logical action.
     */
    BootloaderState desired_state{};

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU8(std::uint8_t(desired_state));
        assert(encoder.getOffset() == (EncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    StaticMessageBuffer<EncodedSize> encode() const
    {
        StaticMessageBuffer<EncodedSize> buf;
        const std::size_t size = encode(buf.begin());
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<BootloaderStatusRequestMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() != EncodedSize)
        {
            return {};
        }

        BootloaderStatusRequestMessage msg;
        msg.desired_state  = BootloaderState(decoder.fetchU8());

        return msg;
    }
};

/**
 * Bootloader status response; contains the current status and stuff.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u64             uptime_ns       Bootloader's uptime in nanoseconds.
 *      8       u64             flags           Reserved.
 *      16      u8              state           The current state of the bootloader's standard state machine.
 *  -----------------------------------------------------------------------------------------------
 *      17
 */
struct BootloaderStatusResponseMessage
{
    static constexpr std::size_t EncodedSize = 17;

    static constexpr MessageID ID = MessageID::BootloaderStatusResponse;

    /**
     * All fields of this message type.
     */
    Timestamp timestamp{};
    std::uint64_t flags = 0;
    BootloaderState state{};

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(ID).encode(encoder);
        encoder.addU64(timestamp.count());
        encoder.addU64(flags);
        encoder.addU8(std::uint8_t(state));
        assert(encoder.getOffset() == (EncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    StaticMessageBuffer<EncodedSize> encode() const
    {
        StaticMessageBuffer<EncodedSize> buf;
        const std::size_t size = encode(buf.begin());
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<BootloaderStatusResponseMessage> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() != EncodedSize)
        {
            return {};
        }

        BootloaderStatusResponseMessage msg;
        msg.timestamp = Timestamp(decoder.fetchU64());
        msg.flags = decoder.fetchU64();
        msg.state  = BootloaderState(decoder.fetchU8());

        return msg;
    }
};

/// Implementation details; do not use that in user code
namespace detail_
{
/**
 * Base type for request/response messages.
 *
 *      Offset  Type            Name            Description
 *  -----------------------------------------------------------------------------------------------
 *      0       u64             image_offset    Offset from the beginning of the image. Must grow sequentially.
 *      1       u8              image_type      Kind of image contained in the message:
 *                                                  0 - application
 *                                                  1 - certificate of authenticity
 *      9       u8[<=256]       image_data      Image data at the specified offset. All messages except the last
 *                                              one are required to contain exactly 256 bytes of image data.
 *                                              The last message is required to contain less than 256 bytes of
 *                                              image data, possibly zero if the image size is 256-byte aligned.
 *                                              Terminated at the end of the message (implicit length).
 *  -----------------------------------------------------------------------------------------------
 *      265
 */
template <typename Derived>
struct BootloaderImageDataMessageBase
{
    static constexpr std::size_t MinEncodedSize = 9;
    static constexpr std::size_t MaxEncodedSize = 265;

    /**
     * All fields of this message type.
     */
     std::uint64_t image_offset = 0;
     BootloaderImageType image_type{};
     senoval::Vector<std::uint8_t, 256> image_data;

    /**
     * Encodes the message into the provided sequential iterator.
     * The iterator can encode and emit the message on the fly - that would be highly efficient;
     * see @ref transport::StreamEmitter.
     * Returns the number of bytes in the encoded stream.
     */
    template <typename OutputIterator>
    std::size_t encode(OutputIterator begin) const
    {
        presentation::StreamEncoder encoder(begin);
        MessageHeader(Derived::ID).encode(encoder);
        encoder.addU64(image_offset);
        encoder.addU8(std::uint8_t(image_type));
        encoder.addBytes(image_data);
        assert(encoder.getOffset() <= (MaxEncodedSize + MessageHeader::Size));
        assert(encoder.getOffset() >= (MinEncodedSize + MessageHeader::Size));
        return encoder.getOffset();
    }

    /**
     * A simpler wrapper on top of the other version of @ref encode<>() that accepts an output iterator.
     * This version encodes the message into a fixed capacity array and returns it by value.
     * Needless to say, it is less efficient than the iterator-based version, but it's easier to use.
     */
    DynamicMessageBuffer<MaxEncodedSize> encode() const
    {
        DynamicMessageBuffer<MaxEncodedSize> buf;
        const std::size_t size = encode(std::back_inserter(buf));
        (void) size;
        assert(size == buf.size());
        return buf;
    }

    /**
     * Attempts to decode a message from the provided standard frame.
     * The message ID value in the header will be checked.
     */
    template <typename InputIterator>
    static std::optional<Derived> tryDecode(InputIterator begin, InputIterator end)
    {
        presentation::StreamDecoder decoder(begin, end);
        const auto header = MessageHeader::tryDecode(decoder);
        if (!header || (header->message_id != Derived::ID))
        {
            return {};
        }

        if (decoder.getRemainingLength() < MinEncodedSize)
        {
            return {};
        }

        if (decoder.getRemainingLength() > MaxEncodedSize)
        {
            return {};
        }

        Derived msg;
        msg.image_offset = decoder.fetchU64();
        msg.image_type = BootloaderImageType(decoder.fetchU8());
        decoder.fetchBytes(std::back_inserter(msg.image_data),
                           decoder.getRemainingLength());
        return msg;
    }
};

} // namespace detail_

/**
 * This message is used to write new application images via the bootloader.
 * It can also be utilized to read data back, but this is not considered useful at the moment.
 */
struct BootloaderImageDataRequestMessage :
    public detail_::BootloaderImageDataMessageBase<BootloaderImageDataRequestMessage>
{
    static constexpr MessageID ID = MessageID::BootloaderImageDataRequest;
};

/**
 * The counterpart for the request message.
 * The response always contains the actual data from the memory.
 * The host compares the written data with the response and determines whether the write was successful.
 * All fields except image_data must have the same values as in the request.
 */
struct BootloaderImageDataResponseMessage :
    public detail_::BootloaderImageDataMessageBase<BootloaderImageDataResponseMessage>
{
    static constexpr MessageID ID = MessageID::BootloaderImageDataResponse;
};

} // namespace standard

} // namespace popcop

#endif
