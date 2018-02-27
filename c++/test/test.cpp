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

// We want to ensure that assertion checks are enabled when tests are run, for extra safety
#ifdef NDEBUG
# undef NDEBUG
#endif

// The library should be included first in order to ensure that all necessary headers are included in the library itself
#include <popcop.hpp>

// Test-only dependencies
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <cmath>

// Note that we should NOT define CATCH_CONFIG_MAIN in the same translation unit which also contains tests;
// that slows things down.
// https://github.com/catchorg/Catch2/blob/master/docs/slow-compiles.md
#include "catch.hpp"


using namespace popcop;


template <typename InputIterator>
inline void printHexDump(InputIterator begin, const InputIterator end)
{
    struct RAIIFlagsSaver
    {
        const std::ios::fmtflags flags_ = std::cout.flags();
        ~RAIIFlagsSaver() { std::cout.flags(flags_); }
    } _flags_saver;

    static constexpr unsigned BytesPerRow = 16;
    unsigned offset = 0;

    std::cout << std::hex << std::setfill('0');

    do
    {
        std::cout << std::setw(8) << offset << "  ";
        offset += BytesPerRow;

        {
            auto it = begin;
            for (unsigned i = 0; i < BytesPerRow; ++i)
            {
                if (i == 8)
                {
                    std::cout << ' ';
                }

                if (it != end)
                {
                    std::cout << std::setw(2) << unsigned(*it) << ' ';
                    ++it;
                }
                else
                {
                    std::cout << "   ";
                }
            }
        }

        std::cout << "  ";
        for (unsigned i = 0; i < BytesPerRow; ++i)
        {
            if (begin != end)
            {
                std::cout << ((unsigned(*begin) >= 32U && unsigned(*begin) <= 126U) ? char(*begin) : '.');
                ++begin;
            }
            else
            {
                std::cout << ' ';
            }
        }

        std::cout << std::endl;
    }
    while (begin != end);
}

template <typename Container>
inline void printHexDump(const Container& cont)
{
    printHexDump(std::begin(cont), std::end(cont));
}


inline void printParserOutput(const transport::ParserOutput& o)
{
    if (auto f = o.getReceivedFrame())
    {
        std::cout << "Frame type code: " << int(f->type_code) << std::endl;
        printHexDump(f->payload);
    }
    else if (auto u = o.getExtraneousData())
    {
        printHexDump(*u);
    }
    else
    {
        std::cout << "EMPTY OUTPUT" << std::endl;
    }
}


template <std::size_t Size>
inline bool doesParserOutputMatch(const transport::ParserOutput& o,
                                  const std::uint8_t frame_type_code,
                                  const std::array<std::uint8_t, Size>& payload)
{
    if (auto f = o.getReceivedFrame())
    {
        REQUIRE((reinterpret_cast<std::uintptr_t>(f->payload.data()) % transport::ParserBufferAlignment) == 0);

        const bool match =
            (f->type_code == frame_type_code) &&
            (f->payload.size() == Size) &&
            std::equal(payload.begin(), payload.end(), f->payload.begin());

        if (match)
        {
            return true;
        }
    }

    std::cout << "PARSER OUTPUT MISMATCH:" << std::endl;
    printParserOutput(o);
    return false;
}


template <typename... Args>
constexpr inline std::array<std::uint8_t, sizeof...(Args)> makeArray(const Args... a)
{
    return std::array<std::uint8_t, sizeof...(Args)>{{std::uint8_t(a)...}};
}


template <typename First, typename Second>
constexpr inline bool areSubSequencesEqual(const First& f, const Second& s)
{
    for (std::size_t i = 0; i < std::min(f.size(), s.size()); i++)
    {
        if (f[i] != s[i])
        {
            return false;
        }
    }

    return true;
}


inline bool isParserOutputEmpty(const transport::ParserOutput& o)
{
    const bool res = (o.getReceivedFrame() == nullptr) && (o.getExtraneousData() == nullptr);
    if (!res)
    {
        std::cout << "NONEMPTY OUTPUT:" << std::endl;
        printParserOutput(o);
    }
    return res;
}


TEST_CASE("ParserSimple")
{
    using transport::FrameDelimiter;
    using transport::EscapeCharacter;

    transport::Parser<> parser;

    REQUIRE(isParserOutputEmpty(parser.processNextByte(FrameDelimiter)));

    SECTION("empty")
    {
        REQUIRE(isParserOutputEmpty(parser.processNextByte(123)));                     // Frame type code
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0x67)));                    // CRC low
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0xAC)));                    // CRC
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0x6C)));                    // CRC
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0xBA)));                    // CRC high
        auto out = parser.processNextByte(FrameDelimiter);
        REQUIRE(doesParserOutputMatch(out, 123, makeArray()));
        REQUIRE(!doesParserOutputMatch(out, 123, makeArray(0)));                       // Test of the test
        REQUIRE(!doesParserOutputMatch(out, 123, makeArray(1, 2)));                    // Ditto
    }

    SECTION("non-empty")
    {
        REQUIRE(isParserOutputEmpty(parser.processNextByte(42)));                      // Payload
        REQUIRE(isParserOutputEmpty(parser.processNextByte(12)));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(34)));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(56)));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(78)));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(90)));                      // Frame type code
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0xCE)));                    // CRC low
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0x4E)));                    // CRC
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0x88)));                    // CRC
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0xBC)));                    // CRC high
        auto out = parser.processNextByte(FrameDelimiter);
        REQUIRE(doesParserOutputMatch(out, 90, makeArray(42, 12, 34, 56, 78)));
        REQUIRE(!doesParserOutputMatch(out, 123, makeArray()));                        // Test of the test
        REQUIRE(!doesParserOutputMatch(out, 123, makeArray(1, 2)));                    // Ditto
    }

    SECTION("escaped")
    {
        REQUIRE(isParserOutputEmpty(parser.processNextByte(EscapeCharacter)));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(FrameDelimiter ^ 0xFF)));    // Payload
        REQUIRE(isParserOutputEmpty(parser.processNextByte(EscapeCharacter)));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(EscapeCharacter ^ 0xFF)));   // Frame type code
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0x91)));                     // CRC low
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0x5C)));                     // CRC
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0xA9)));                     // CRC
        REQUIRE(isParserOutputEmpty(parser.processNextByte(0xC0)));                     // CRC high
        auto out = parser.processNextByte(FrameDelimiter);
        REQUIRE(doesParserOutputMatch(out, EscapeCharacter, makeArray(FrameDelimiter)));
    }

    SECTION("unparseable")
    {
        using Ch = std::uint8_t;
        REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('H'))));                  // Payload
        REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('e'))));                  // Frame type code
        REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('l'))));                  // CRC (supposed to be)
        REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('l'))));                  // (it is invalid)
        REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('o'))));
        REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('!'))));
        auto out = parser.processNextByte(FrameDelimiter);
        REQUIRE(out.getReceivedFrame() == nullptr);
        REQUIRE(out.getExtraneousData() != nullptr);
        REQUIRE(std::equal(out.getExtraneousData()->begin(), out.getExtraneousData()->end(), std::begin("Hello!")));
    }
}


TEST_CASE("ParserNoDoubleDelimiters")
{
    using transport::FrameDelimiter;
    using transport::EscapeCharacter;

    transport::Parser<> parser;

    REQUIRE(isParserOutputEmpty(parser.processNextByte(FrameDelimiter)));

    REQUIRE(isParserOutputEmpty(parser.processNextByte(123)));                     // Frame type code
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x67)));                    // CRC low
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xAC)));                    // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x6C)));                    // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xBA)));                    // CRC high
    auto out = parser.processNextByte(FrameDelimiter);
    REQUIRE(doesParserOutputMatch(out, 123, makeArray()));
    REQUIRE(!doesParserOutputMatch(out, 123, makeArray(0)));                       // Test of the test
    REQUIRE(!doesParserOutputMatch(out, 123, makeArray(1, 2)));                    // Ditto

    REQUIRE(isParserOutputEmpty(parser.processNextByte(42)));                      // Payload
    REQUIRE(isParserOutputEmpty(parser.processNextByte(12)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(34)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(56)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(78)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(90)));                      // Frame type code
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xCE)));                    // CRC low
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x4E)));                    // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x88)));                    // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xBC)));                    // CRC high
    out = parser.processNextByte(FrameDelimiter);
    REQUIRE(doesParserOutputMatch(out, 90, makeArray(42, 12, 34, 56, 78)));
    REQUIRE(!doesParserOutputMatch(out, 123, makeArray()));                        // Test of the test
    REQUIRE(!doesParserOutputMatch(out, 123, makeArray(1, 2)));                    // Ditto

    REQUIRE(isParserOutputEmpty(parser.processNextByte(EscapeCharacter)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(FrameDelimiter ^ 0xFF)));    // Payload
    REQUIRE(isParserOutputEmpty(parser.processNextByte(EscapeCharacter)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(EscapeCharacter ^ 0xFF)));   // Frame type code
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x91)));                     // CRC low
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x5C)));                     // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xA9)));                     // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xC0)));                     // CRC high
    out = parser.processNextByte(FrameDelimiter);
    REQUIRE(doesParserOutputMatch(out, EscapeCharacter, makeArray(FrameDelimiter)));

    using Ch = std::uint8_t;
    REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('H'))));                  // Payload
    REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('e'))));                  // Frame type code
    REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('l'))));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('l'))));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('o'))));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(Ch('!'))));
    out = parser.processNextByte(FrameDelimiter);
    REQUIRE(out.getReceivedFrame() == nullptr);
    REQUIRE(out.getExtraneousData() != nullptr);
    REQUIRE(std::equal(out.getExtraneousData()->begin(), out.getExtraneousData()->end(), std::begin("Hello!")));
}


TEST_CASE("ParserReset")
{
    using transport::FrameDelimiter;
    using transport::EscapeCharacter;

    transport::Parser<> parser;

    REQUIRE(isParserOutputEmpty(parser.processNextByte(FrameDelimiter)));

    REQUIRE(isParserOutputEmpty(parser.processNextByte(123)));                     // Frame type code
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x67)));                    // CRC low
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xAC)));                    // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0x6C)));                    // CRC
    REQUIRE(isParserOutputEmpty(parser.processNextByte(0xBA)));                    // CRC high
    parser.reset();
    REQUIRE(isParserOutputEmpty(parser.processNextByte(FrameDelimiter)));
}


template <std::size_t Size>
inline bool doesEmitterOutputMatch(transport::BufferedEmitter encoder,
                                   const std::array<std::uint8_t, Size>& output)
{
    std::size_t index = 0;
    do
    {
        const auto b = encoder.getNextByte();
        if (output.at(index) != b)
        {
            std::cout << "ENCODER OUTPUT MISMATCH: " << int(output.at(index)) << " != " << int(b) << std::endl;
            return false;
        }
        ++index;
    }
    while (!encoder.isFinished());

    return true;
}


TEST_CASE("BufferedEmitterSimple")
{
    using transport::FrameDelimiter;
    using transport::EscapeCharacter;

    REQUIRE(doesEmitterOutputMatch(transport::BufferedEmitter(123, "", 0),
                                   makeArray(FrameDelimiter, 123, 0x67, 0xAC, 0x6C, 0xBA, FrameDelimiter)));

    REQUIRE(doesEmitterOutputMatch(transport::BufferedEmitter(90, makeArray(42, 12, 34, 56, 78)),
                                   makeArray(FrameDelimiter, 42, 12, 34, 56, 78, 90, 0xCE, 0x4E, 0x88, 0xBC,
                                             FrameDelimiter)));

    REQUIRE(doesEmitterOutputMatch(transport::BufferedEmitter(EscapeCharacter, makeArray(FrameDelimiter)),
                                   makeArray(FrameDelimiter,
                                             EscapeCharacter, ~FrameDelimiter,
                                             EscapeCharacter, ~EscapeCharacter,
                                             0x91, 0x5C, 0xA9, 0xC0, FrameDelimiter)));
}


template <std::size_t ParserBufferSize, typename FramePayloadContainer, typename ExtraneousDataContainer>
inline bool validateEncodeDecodeLoop(transport::Parser<ParserBufferSize>& parser,
                                     const std::uint8_t frame_type_code,
                                     const FramePayloadContainer& frame_payload,
                                     const ExtraneousDataContainer& extraneous_data)
{
    transport::BufferedEmitter encoder(frame_type_code, frame_payload.data(), frame_payload.size());
    transport::ParserOutput out;

    while (!encoder.isFinished())
    {
        out = parser.processNextByte(encoder.getNextByte());

        if (out.getReceivedFrame() != nullptr)
        {
            REQUIRE(encoder.isFinished());
            break;
        }

        if (auto e = out.getExtraneousData())
        {
            REQUIRE(!encoder.isFinished());
            REQUIRE(std::equal(extraneous_data.begin(), extraneous_data.end(), e->begin()));
        }
    }

    if (auto f = out.getReceivedFrame())
    {
        REQUIRE((reinterpret_cast<std::uintptr_t>(f->payload.data()) % transport::ParserBufferAlignment) == 0);
        return (f->type_code == frame_type_code) &&
               std::equal(frame_payload.begin(), frame_payload.end(), f->payload.begin());
    }
    else
    {
        std::cout << "ENCODE-DECODE LOOP ERROR: EXPECTED FRAME:" << std::endl;
        printParserOutput(out);
        return false;
    }
}


template <std::size_t ParserBufferSize, typename FramePayloadContainer, typename ExtraneousDataContainer>
inline void validateEncodeDecodeLoopWithStreamEmitter(transport::Parser<ParserBufferSize>& parser,
                                                      const std::uint8_t frame_type_code,
                                                      const FramePayloadContainer& frame_payload,
                                                      const ExtraneousDataContainer& extraneous_data)
{
    const auto sink = [&](const std::uint8_t byte)
    {
        const auto out = parser.processNextByte(byte);

        if (auto f = out.getReceivedFrame())
        {
            REQUIRE((reinterpret_cast<std::uintptr_t>(f->payload.data()) % transport::ParserBufferAlignment) == 0);
            REQUIRE((f->type_code == frame_type_code));
            REQUIRE(std::equal(frame_payload.begin(), frame_payload.end(), f->payload.begin()));
        }

        if (auto e = out.getExtraneousData())
        {
            REQUIRE(std::equal(extraneous_data.begin(), extraneous_data.end(), e->begin()));
        }
    };

    // Watch magic happen! We just copy encoded data out into the output channel!
    std::copy(frame_payload.begin(),
              frame_payload.end(),
              transport::StreamEmitter(frame_type_code, sink).begin());
}


inline std::uint8_t getRandomByte()
{
    return std::uint8_t(std::rand());  // NOLINT
}


inline bool getRandomBit()
{
    return getRandomByte() % 2 == 0;
}


inline std::vector<std::uint8_t> getRandomNumberOfRandomBytes(const bool allow_frame_delimiters = true)
{
    std::vector<std::uint8_t> o;
    o.resize(std::size_t(getRandomByte()) * std::size_t(getRandomByte()));

    for (auto& x: o)
    {
        x = getRandomByte();
        if (!allow_frame_delimiters)
        {
            if (x == transport::FrameDelimiter)
            {
                ++x;
            }
        }
    }

    return o;
}


template <std::size_t NumBytes, bool IsSigned, bool IsFloating>
inline auto getRandomNumber()
{
    static_assert((NumBytes == 1) || (NumBytes == 2) || (NumBytes == 4) || (NumBytes == 8));
    static_assert(IsFloating ? ((NumBytes == 4) || (NumBytes == 8)) && IsSigned : true);

    using Unsigned = std::conditional_t<
        NumBytes == 1, std::uint8_t,
        std::conditional_t<
            NumBytes == 2, std::uint16_t,
            std::conditional_t<
                NumBytes == 4, std::uint32_t,
                std::conditional_t<
                    NumBytes == 8, std::uint64_t,
                    void>>>>;
    using Float = std::conditional_t<NumBytes == 4, float, double>;
    using ValueType = std::conditional_t<IsFloating, Float,
        std::conditional_t<IsSigned, std::make_signed_t<Unsigned>, Unsigned>>;

    std::array<std::uint8_t, NumBytes> bytes{};
    for (auto& b : bytes)
    {
        b = getRandomByte();
    }

    static_assert(sizeof(ValueType) == NumBytes);
    ValueType value{};
    std::memcpy(&value, bytes.data(), sizeof(ValueType));
    return value;
}


TEST_CASE("EmitterParserLoop-slow")
{
    std::srand(unsigned(std::time(nullptr)));

    transport::Parser<65535> parser;

    REQUIRE(validateEncodeDecodeLoop(parser, 123, makeArray(1, 2, 3), makeArray()));
    REQUIRE(validateEncodeDecodeLoop(parser, transport::FrameDelimiter, makeArray(), makeArray()));

    REQUIRE(isParserOutputEmpty(parser.processNextByte(123)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(213)));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(32)));
    REQUIRE(validateEncodeDecodeLoop(parser,
                                     transport::EscapeCharacter,
                                     makeArray(transport::EscapeCharacter),
                                     makeArray(123, 213, 32)));

    std::cout << "Random bytes:\n" << std::endl;
    printHexDump(getRandomNumberOfRandomBytes());

    constexpr long long NumberOfIterations = 20000;

    for (long long ago = 0; ago < NumberOfIterations; ago++)
    {
        if (ago % 1000 == 0)
        {
            std::cout << "\r" << ago << "/" << NumberOfIterations << "  \r" << std::flush;
        }

        const auto extraneous = getRandomNumberOfRandomBytes(false);
        const auto payload = getRandomNumberOfRandomBytes();
        const auto frame_type_code = getRandomByte();

        REQUIRE(validateEncodeDecodeLoop(parser,
                                         frame_type_code,
                                         payload,
                                         extraneous));

        validateEncodeDecodeLoopWithStreamEmitter(parser,
                                                  frame_type_code,
                                                  payload,
                                                  extraneous);
    }

    std::cout << "\r" << NumberOfIterations << " ITERATIONS DONE" << std::endl;
}


TEST_CASE("ParserMaxPacketLength")
{
    transport::Parser<1024> parser;
    transport::CRCComputer crc;

    REQUIRE(isParserOutputEmpty(parser.processNextByte(transport::FrameDelimiter)));

    // Fill with known data
    for (unsigned i = 0; i < 1024; i++)
    {
        const auto byte = std::uint8_t(i & 0x7FU);
        REQUIRE(isParserOutputEmpty(parser.processNextByte(byte)));
        crc.add(byte);
    }

    REQUIRE(isParserOutputEmpty(parser.processNextByte(123)));
    crc.add(123);

    REQUIRE(isParserOutputEmpty(parser.processNextByte(std::uint8_t(crc.get() >> 0))));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(std::uint8_t(crc.get() >> 8))));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(std::uint8_t(crc.get() >> 16))));
    REQUIRE(isParserOutputEmpty(parser.processNextByte(std::uint8_t(crc.get() >> 24))));

    auto out = parser.processNextByte(transport::FrameDelimiter);
    auto rf = out.getReceivedFrame();
    REQUIRE(rf != nullptr);
    REQUIRE(rf->type_code == 123);
    REQUIRE(rf->payload.size() == 1024);
    REQUIRE((reinterpret_cast<std::uintptr_t>(rf->payload.data()) % transport::ParserBufferAlignment) == 0);

    (void) rf->payload.alias<long double>();

    // Validate the data
    for (unsigned i = 0; i < 1024; i++)
    {
        const auto byte = std::uint8_t(i & 0x7FU);
        REQUIRE(rf->payload.at(i) == byte);
    }
}


TEST_CASE("ParserOverflow")
{
    transport::Parser<1024> parser;

    // Fill with known data
    for (unsigned i = 1; i < 1030; i++)
    {
        const auto byte = std::uint8_t(i & 0x7FU);
        REQUIRE(isParserOutputEmpty(parser.processNextByte(byte)));
    }

    // Ensure that the overflow is handled correctly
    auto out = parser.processNextByte(123);
    auto ed = out.getExtraneousData();
    REQUIRE(ed != nullptr);
    for (unsigned i = 1; i < 1030; i++)
    {
        const auto byte = std::uint8_t(i & 0x7FU);
        REQUIRE(byte == ed->at(i - 1));
    }

    // Fill with more data
    for (unsigned i = 1; i < 1028; i++)
    {
        const auto byte = std::uint8_t(i & 0x7FU);
        REQUIRE(isParserOutputEmpty(parser.processNextByte(byte)));
    }
}


TEST_CASE("CRC")
{
    transport::CRCComputer crc;

    REQUIRE(crc.get() == 0);
    REQUIRE(!crc.isResidueCorrect());

    crc.add('1');
    crc.add('2');
    crc.add('3');
    crc.add('4');
    crc.add('5');
    crc.add('6');
    crc.add('7');
    crc.add('8');
    crc.add('9');

    REQUIRE(crc.get() == 0xE3069283);
    REQUIRE(!crc.isResidueCorrect());

    crc.add(0x83);
    crc.add(0x92);
    crc.add(0x06);
    crc.add(0xE3);

    REQUIRE(crc.isResidueCorrect());
}


TEST_CASE("FixedCapacityString")
{
    util::FixedCapacityString<10> s;
    REQUIRE(s.empty());
    REQUIRE(std::string(s.c_str()) == "");  // NOLINT
    REQUIRE(s == "");  // NOLINT
    REQUIRE(s != " ");
    REQUIRE(s.capacity() == 10);
    REQUIRE(s.max_size() == 10);
    REQUIRE(s.length() == 0);
    REQUIRE(s.size() == 0);  // NOLINT

    s += "123";
    REQUIRE(!s.empty());
    REQUIRE(std::string(s.c_str()) == "123");
    REQUIRE("123" == s);
    REQUIRE(" " != s);

    s += util::FixedCapacityString<10>("456");
    REQUIRE(!s.empty());
    REQUIRE(std::string(s.c_str()) == "123456");
    REQUIRE(s == "123456");
    REQUIRE(s != "123");
    REQUIRE(s.capacity() == 10);
    REQUIRE(s.max_size() == 10);
    REQUIRE(s.length() == 6);
    REQUIRE(s.size() == 6);

    s += "7890a";
    REQUIRE(!s.empty());
    REQUIRE(std::string(s.c_str()) == "1234567890");
    REQUIRE(s == "1234567890");
    REQUIRE(s != "1234567890a");

    s.clear();
    REQUIRE(s.empty());
    REQUIRE(std::string(s.c_str()) == "");  // NOLINT
    REQUIRE(s == "");  // NOLINT

    s = util::FixedCapacityString<30>("qwertyuiopasdfghjklzxcvbnm");
    REQUIRE(std::string(s.c_str()) == "qwertyuiop");
    REQUIRE(s == "qwertyuiop");

    s = util::FixedCapacityString<30>("123");
    s += 'a';
    s += 'b';
    s += 'c';
    REQUIRE(std::string(s.c_str()) == "123abc");
    REQUIRE(s == "123abc");
    REQUIRE(s[0] == '1');
    REQUIRE(s[1] == '2');
    REQUIRE(s[2] == '3');
    REQUIRE(s[3] == 'a');
    REQUIRE(s[4] == 'b');
    REQUIRE(s[5] == 'c');
    REQUIRE(s.front() == '1');
    REQUIRE(s.back() == 'c');
    REQUIRE(*s.begin() == '1');
    REQUIRE(*(s.end() - 1) == 'c');
    REQUIRE(*s.end() == '\0');

    s = util::FixedCapacityString<30>("hElLo/*-12");
    REQUIRE(s.toLowerCase() == "hello/*-12");
    REQUIRE("HELLO/*-12" == s.toUpperCase());

    auto s2 = s + util::FixedCapacityString<10>(" World!");
    REQUIRE(s2.capacity() == 20);
    REQUIRE(s2.max_size() == 20);
    REQUIRE(s2.size() == 17);
    REQUIRE(s2.length() == 17);
    REQUIRE(s2 == "hElLo/*-12 World!");

    REQUIRE("[hElLo/*-12 World!]" == ("[" + s2 + "]"));
}


TEST_CASE("FixedCapacityVector")
{
    util::FixedCapacityVector<std::int32_t, 10> vec;

    REQUIRE(sizeof(vec) == 40 + sizeof(std::size_t));
    REQUIRE(vec.empty());
    REQUIRE(vec.capacity() == 10);
    REQUIRE(vec.max_size() == 10);
    REQUIRE(vec.size() == 0);
    REQUIRE(vec.begin() == vec.end());

    vec.push_back(1);
    REQUIRE(!vec.empty());
    REQUIRE(vec.capacity() == 10);
    REQUIRE(vec.max_size() == 10);
    REQUIRE(vec.size() == 1);
    REQUIRE(vec.begin() != vec.end());
    REQUIRE(1 == *vec.begin());
    REQUIRE(1 == *(vec.end() - 1));
    REQUIRE(1 == vec.front());
    REQUIRE(1 == vec.back());

    vec.push_back(2);
    REQUIRE(!vec.empty());
    REQUIRE(vec.capacity() == 10);
    REQUIRE(vec.max_size() == 10);
    REQUIRE(vec.size() == 2);
    REQUIRE(vec.begin() != vec.end());
    REQUIRE(1 == *vec.begin());
    REQUIRE(2 == *(vec.end() - 1));
    REQUIRE(1 == vec.front());
    REQUIRE(2 == vec.back());

    vec.push_back(3);
    vec.push_back(4);
    vec.push_back(5);
    vec.push_back(6);
    vec.push_back(7);
    vec.push_back(8);
    vec.push_back(9);
    vec.push_back(10);
    REQUIRE(!vec.empty());
    REQUIRE(vec.size() == 10);
    REQUIRE(1 == *vec.begin());
    REQUIRE(10 == *(vec.end() - 1));
    REQUIRE(1 == vec.front());
    REQUIRE(10 == vec.back());

    const std::int8_t arr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    const util::FixedCapacityVector<std::int8_t, 80> vec2(std::begin(arr), std::end(arr));
    REQUIRE(sizeof(vec2) == 80 + sizeof(std::size_t));
    REQUIRE(!vec2.empty());
    REQUIRE(vec2.capacity() == 80);
    REQUIRE(vec2.max_size() == 80);
    REQUIRE(vec2.size() == 10);
    REQUIRE(vec2.begin() != vec2.end());
    REQUIRE(1 == *vec2.begin());
    REQUIRE(10 == *(vec2.end() - 1));
    REQUIRE(1 == vec2.front());
    REQUIRE(10 == vec2.back());

    REQUIRE(vec == vec2);
    REQUIRE(vec2 == vec);
    REQUIRE((vec != vec2) == false);
    REQUIRE((vec2 != vec) == false);

    vec[3] = -3;

    REQUIRE(vec != vec2);
    REQUIRE(vec2 != vec);
    REQUIRE((vec == vec2) == false);
    REQUIRE((vec2 == vec) == false);

    const util::FixedCapacityVector<std::int32_t, 10> vec_copy = vec;
    REQUIRE(!vec_copy.empty());
    REQUIRE(vec_copy.capacity() == 10);
    REQUIRE(vec_copy.max_size() == 10);
    REQUIRE(vec_copy.size() == 10);
    REQUIRE(vec_copy.begin() != vec_copy.end());

    REQUIRE(vec_copy != vec2);
    REQUIRE(vec2 != vec_copy);
    REQUIRE(vec == vec_copy);
    REQUIRE(vec_copy == vec);
    REQUIRE((vec_copy == vec2) == false);
    REQUIRE((vec2 == vec_copy) == false);
    REQUIRE((vec != vec_copy) == false);
    REQUIRE((vec_copy != vec) == false);

    vec.clear();
    REQUIRE(vec.empty());
    REQUIRE(vec.capacity() == 10);
    REQUIRE(vec.max_size() == 10);
    REQUIRE(vec.size() == 0);
    REQUIRE(vec.begin() == vec.end());

    REQUIRE(vec != vec2);
    REQUIRE(vec2 != vec);
    REQUIRE(vec != vec_copy);
    REQUIRE(vec_copy != vec);
    REQUIRE((vec == vec2) == false);
    REQUIRE((vec2 == vec) == false);
    REQUIRE((vec == vec_copy) == false);
    REQUIRE((vec_copy == vec) == false);

    {
        util::FixedCapacityVector<std::int32_t, 6> vec3(5, 123);
        REQUIRE(!vec3.empty());
        REQUIRE(vec3.capacity() == 6);
        REQUIRE(vec3.max_size() == 6);
        REQUIRE(vec3.size() == 5);
        REQUIRE(vec3[0] == 123);
        REQUIRE(vec3[1] == 123);
        REQUIRE(vec3[2] == 123);
        REQUIRE(vec3[3] == 123);
        REQUIRE(vec3[4] == 123);
    }

    {
        util::FixedCapacityVector<std::int8_t, 7> vec4({1, 2, 3, 4});
        REQUIRE(!vec4.empty());
        REQUIRE(vec4.capacity() == 7);
        REQUIRE(vec4.max_size() == 7);
        REQUIRE(vec4.size() == 4);
        REQUIRE(vec4[0] == 1);
        REQUIRE(vec4[1] == 2);
        REQUIRE(vec4[2] == 3);
        REQUIRE(vec4[3] == 4);
    }
}


TEST_CASE("StreamEncoder")
{
    util::FixedCapacityVector<std::uint8_t, 100> vec;
    presentation::StreamEncoder encoder(std::back_inserter(vec));

    REQUIRE(encoder.getOffset() == 0);
    REQUIRE(vec.size() == 0);

    encoder.addU8(123U);
    encoder.addI8(-123);
    REQUIRE(encoder.getOffset() == 2);
    REQUIRE(vec.size() == 2);
    REQUIRE(vec[0] == 123);
    REQUIRE(vec[1] == 133);     // as unsigned

    encoder.addI16(-30000);
    encoder.addU16(30000U);
    REQUIRE(encoder.getOffset() == 6);
    REQUIRE(vec.size() == 6);
    REQUIRE(vec[0] == 123);
    REQUIRE(vec[1] == 133);
    REQUIRE(vec[2] == 208);
    REQUIRE(vec[3] == 138);
    REQUIRE(vec[4] == 48);
    REQUIRE(vec[5] == 117);

    encoder.fillUpToOffset(9, 42);
    REQUIRE(encoder.getOffset() == 9);
    REQUIRE(vec.size() == 9);
    REQUIRE(vec[0] == 123);
    REQUIRE(vec[1] == 133);
    REQUIRE(vec[2] == 208);
    REQUIRE(vec[3] == 138);
    REQUIRE(vec[4] == 48);
    REQUIRE(vec[5] == 117);
    REQUIRE(vec[6] == 42);
    REQUIRE(vec[7] == 42);
    REQUIRE(vec[8] == 42);

    encoder.addBytes(makeArray(1, 2, 3, 4, 5, 6));
    REQUIRE(encoder.getOffset() == 15);
    REQUIRE(vec.size() == 15);
    REQUIRE(vec[6] == 42);
    REQUIRE(vec[7] == 42);
    REQUIRE(vec[8] == 42);
    REQUIRE(vec[9] == 1);
    REQUIRE(vec[10] == 2);
    REQUIRE(vec[11] == 3);
    REQUIRE(vec[12] == 4);
    REQUIRE(vec[13] == 5);
    REQUIRE(vec[14] == 6);

    encoder.addI32(-30000000);
    encoder.addU32(30000000U);      // 00000001 11001001 11000011 10000000
    REQUIRE(encoder.getOffset() == 23);
    REQUIRE(vec.size() == 23);
    REQUIRE(vec[15] == 128);
    REQUIRE(vec[16] == 60);
    REQUIRE(vec[17] == 54);
    REQUIRE(vec[18] == 254);
    REQUIRE(vec[19] == 0b10000000);
    REQUIRE(vec[20] == 0b11000011);
    REQUIRE(vec[21] == 0b11001001);
    REQUIRE(vec[22] == 0b00000001);

    encoder.addI64(-30000000010LL);
    encoder.addU64(30000000010ULL);      // 00000000 00000000 00000000 00000110 11111100 00100011 10101100 00001010
    REQUIRE(encoder.getOffset() == 39);
    REQUIRE(vec.size() == 39);
    REQUIRE(vec[23] == 246);
    REQUIRE(vec[24] == 83);
    REQUIRE(vec[25] == 220);
    REQUIRE(vec[26] == 3);
    REQUIRE(vec[27] == 249);
    REQUIRE(vec[28] == 255);
    REQUIRE(vec[29] == 255);
    REQUIRE(vec[30] == 255);
    // the unsigned starts here
    REQUIRE(vec[31] == 0b00001010);
    REQUIRE(vec[32] == 0b10101100);
    REQUIRE(vec[33] == 0b00100011);
    REQUIRE(vec[34] == 0b11111100);
    REQUIRE(vec[35] == 0b00000110);
    REQUIRE(vec[36] == 0b00000000);
    REQUIRE(vec[37] == 0b00000000);
    REQUIRE(vec[38] == 0b00000000);
}


TEST_CASE("StreamDecoder-slow")
{
    constexpr auto BufferSize = 400'000'000;    ///< This might be too much for some systems?

    auto vec = std::make_shared<util::FixedCapacityVector<std::uint8_t, BufferSize>>();
    presentation::StreamEncoder encoder(std::back_inserter(*vec));
    presentation::StreamDecoder decoder(vec->begin(),
                                        vec->begin() + vec->capacity());
    REQUIRE(decoder.getOffset() == 0);
    REQUIRE(decoder.getRemainingLength() == BufferSize);
    std::unordered_map<std::uint8_t, std::uint64_t> stats;

    std::cout << "Running randomized stream decoder test with " << BufferSize << " bytes of data..." << std::endl;

    while ((vec->size() + 65536) < vec->capacity())
    {
        const std::uint8_t tag = std::uint8_t(getRandomByte() % 13U);
        stats[tag]++;
        switch (tag)
        {
        case 0:
        {
            const auto value = getRandomNumber<1, false, false>();
            encoder.addU8(value);
            REQUIRE(decoder.fetchU8() == value);
            break;
        }
        case 1:
        {
            const auto value = getRandomNumber<2, false, false>();
            encoder.addU16(value);
            REQUIRE(decoder.fetchU16() == value);
            break;
        }
        case 2:
        {
            const auto value = getRandomNumber<4, false, false>();
            encoder.addU32(value);
            REQUIRE(decoder.fetchU32() == value);
            break;
        }
        case 3:
        {
            const auto value = getRandomNumber<8, false, false>();
            encoder.addU64(value);
            REQUIRE(decoder.fetchU64() == value);
            break;
        }
        case 4:
        {
            const auto value = getRandomNumber<1, true, false>();
            encoder.addI8(value);
            REQUIRE(decoder.fetchI8() == value);
            break;
        }
        case 5:
        {
            const auto value = getRandomNumber<2, true, false>();
            encoder.addI16(value);
            REQUIRE(decoder.fetchI16() == value);
            break;
        }
        case 6:
        {
            const auto value = getRandomNumber<4, true, false>();
            encoder.addI32(value);
            REQUIRE(decoder.fetchI32() == value);
            break;
        }
        case 7:
        {
            const auto value = getRandomNumber<8, true, false>();
            encoder.addI64(value);
            REQUIRE(decoder.fetchI64() == value);
            break;
        }
        case 8:
        {
            const auto value = getRandomNumber<4, true, true>();
            encoder.addF32(value);
            if (!std::isnan(value))
            {
                REQUIRE(decoder.fetchF32() == Approx(value));
            }
            else
            {
                REQUIRE(std::isnan(decoder.fetchF32()));
            }
            break;
        }
        case 9:
        {
            const auto value = getRandomNumber<8, true, true>();
            encoder.addF64(value);
            if (!std::isnan(value))
            {
                REQUIRE(decoder.fetchF64() == Approx(value));
            }
            else
            {
                REQUIRE(std::isnan(decoder.fetchF64()));
            }
            break;
        }
        case 10:
        {
            const auto depth = getRandomByte();
            const auto fill = getRandomByte();
            encoder.fillUpToOffset(encoder.getOffset() + depth, fill);
            util::FixedCapacityVector<std::uint8_t, 255> out;
            if (getRandomBit())
            {
                decoder.fetchBytes(std::back_inserter(out), depth);
            }
            else
            {
                out.resize(depth);
                decoder.fetchBytes(out.begin(), out.end());
            }

            REQUIRE(out.size() == depth);
            for (unsigned i = 0; i < depth; i++)
            {
                REQUIRE(out[i] == fill);
            }
            break;
        }
        case 11:
        {
            const auto offset = encoder.getOffset() + getRandomByte();
            encoder.fillUpToOffset(offset);
            decoder.skipUpToOffset(offset);
            REQUIRE(decoder.getOffset() == encoder.getOffset());
            break;
        }
        case 12:
        {
            util::FixedCapacityString<65535> str;
            const std::uint16_t str_length = getRandomNumber<2, false, false>();
            for (unsigned i = 0; i < str_length; i++)
            {
                const auto char_byte = std::max(1U, getRandomByte() & 0b0111'1111U);
                assert(char_byte > 0);
                assert(char_byte < 128);
                str.push_back(char(char_byte));
            }
            encoder.addBytes(str);
            if (str.size() < str.capacity())        // Decoder will ignore the null terminator if at full capacity
            {
                encoder.addI8(0);                   // ...which is by design.
            }
            util::FixedCapacityString<65535> out = "Some garbage";
            decoder.fetchASCIIString(out);
            REQUIRE(out == str);
            REQUIRE(decoder.getOffset() == encoder.getOffset());
            break;
        }
        default:
        {
            assert(false);
        }
        }
    }

    std::cout << "Randomized stream decoder test has finished running." << std::endl;
    std::cout << "decoder.getOffset()          = " << decoder.getOffset() << std::endl;
    std::cout << "decoder.getRemainingLength() = " << decoder.getRemainingLength() << std::endl;

    REQUIRE(decoder.getOffset() == encoder.getOffset());
    REQUIRE(encoder.getOffset() == vec->size());
    REQUIRE(decoder.getRemainingLength() == (vec->capacity() - vec->size()));

    std::cout << "Decoder test type tag usage:" << std::endl;
    std::uint64_t total = 0;
    for (auto p : stats)
    {
        total += p.second;
        std::cout << int(p.first) << ": " << p.second << std::endl;
    }
    std::cout << "Total: " << total << std::endl;
}


TEST_CASE("NodeInfoMessage")
{
    const std::array<std::uint8_t, 372> carefully_crafted_message
    {{
        0x00, 0x00,                                       // Message ID
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,               // Reserved in the header

        0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xFF,   // SW CRC
        0xEF, 0xBE, 0xAD, 0xDE,                           // SW VCS ID
        0xD2, 0x00, 0xDF, 0xBA,                           // SW build timestamp UTC
        0x01, 0x02,                                       // SW version
        0x03, 0x04,                                       // HW version
        0x07,                                             // Flags (CRC set, release build, dirty build)
        0x00,                                             // Mode
        0x00, 0x00,                                       // Reserved

        0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09,   // Unique ID
        0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,

        0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x21, 0x00, 0x00,   // Name
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x53, 0x70, 0x61, 0x63, 0x65, 0x21, 0x00, 0x00,   // Description
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x75, 0x70, 0x79, 0x61, 0x63, 0x68, 0x6b, 0x61,   // Build environment description
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x52, 0x55, 0x4e, 0x54, 0x49, 0x4d, 0x45, 0x21,   // Runtime environment description
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

        0x01, 0x02, 0x03, 0x04,
    }};

    // Check whether all items are inited correctly
    REQUIRE(carefully_crafted_message.back() == 0x04);

    standard::NodeInfoMessage msg;

    msg.software_version.image_crc = 0xFFDEBC9A78563412ULL;
    msg.software_version.vcs_commit_id = 0xDEADBEEFUL;
    msg.software_version.build_timestamp_utc = 0xBADF00D2UL;
    msg.software_version.major = 1;
    msg.software_version.minor = 2;
    msg.software_version.release_build = true;
    msg.software_version.dirty_build = true;

    msg.hardware_version.major = 3;
    msg.hardware_version.minor = 4;

    msg.mode = standard::NodeInfoMessage::Mode::Normal;
    msg.globally_unique_id = makeArray(0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09,
                                       0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01);

    msg.node_name = "Hello!";
    msg.node_description = "Space!";
    msg.build_environment_description = "upyachka";
    msg.runtime_environment_description = "RUNTIME!";

    msg.certificate_of_authenticity.push_back(1);
    msg.certificate_of_authenticity.push_back(2);
    msg.certificate_of_authenticity.push_back(3);
    msg.certificate_of_authenticity.push_back(4);

    const auto encoded = msg.encode();
    REQUIRE(encoded.size() == 8 + 360 + 4);

    std::cout << "Manually constructed:" << std::endl;
    printHexDump(carefully_crafted_message);
    std::cout << "Rendered:" << std::endl;
    printHexDump(encoded);
    REQUIRE(std::equal(carefully_crafted_message.begin(), carefully_crafted_message.end(), encoded.begin()));

    /*
     * Decoding test
     */
    const auto m2 = standard::NodeInfoMessage::tryDecode(carefully_crafted_message.begin(),
                                                         carefully_crafted_message.end());
    REQUIRE(m2);
    std::cout << "After reparsing:" << std::endl;
    printHexDump(m2->encode());
    REQUIRE(m2->encode() == carefully_crafted_message);

    {
        auto ccm = carefully_crafted_message;

        // Change mode
        ccm[21 + standard::MessageHeader::Size] = std::uint8_t(standard::NodeInfoMessage::Mode::Bootloader);
        REQUIRE(standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.end())->mode ==
                    standard::NodeInfoMessage::Mode::Bootloader);

        // Use invalid mode
        ccm[21 + standard::MessageHeader::Size] = 123;
        REQUIRE(!standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.end()));  // Not parsed
    }

    {
        auto ccm = carefully_crafted_message;
        ccm[0] = 123;       // Different message ID
        REQUIRE(!standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.end()));  // Not parsed
    }

    {
        auto ccm = carefully_crafted_message;
        REQUIRE(!standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.begin() + 360));  // Too short
        REQUIRE(!standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.begin() + 700));  // Too long
        REQUIRE(standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.end()));           // Just right
    }

    {
        auto ccm = carefully_crafted_message;
        // Default
        auto m = standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.end());
        REQUIRE(m);
        REQUIRE(m->software_version.image_crc.has_value());
        REQUIRE(m->software_version.release_build);
        REQUIRE(m->software_version.dirty_build);
        // Erase flags
        ccm[20 + standard::MessageHeader::Size] = 0;
        m = standard::NodeInfoMessage::tryDecode(ccm.begin(), ccm.end());
        REQUIRE(m);
        REQUIRE(!m->software_version.image_crc.has_value());
        REQUIRE(!m->software_version.release_build);
        REQUIRE(!m->software_version.dirty_build);
    }
}


TEST_CASE("RegisterDataEncoding")
{
    using standard::MessageID;

    {
        // Testing standard::RegisterDataResponseMessage here; the other tests test the other message
        using standard::RegisterDataResponseMessage;
        RegisterDataResponseMessage msg;
        REQUIRE(msg.name.empty());
        REQUIRE(msg.is<RegisterDataResponseMessage::Empty>());
        REQUIRE(!msg.is<RegisterDataResponseMessage::Unstructured>());
        REQUIRE(msg.as<RegisterDataResponseMessage::Empty>() != nullptr);
        REQUIRE(msg.as<RegisterDataResponseMessage::String>() == nullptr);

        REQUIRE(msg == RegisterDataResponseMessage());
        REQUIRE((msg != RegisterDataResponseMessage()) == false);

        {
            const auto encoded = msg.encode();
            REQUIRE(encoded.size() == 10);
            REQUIRE(encoded == makeArray(std::uint8_t(MessageID::RegisterDataResponse), 0,  // msg ID
                                         0, 0, 0, 0, 0, 0,  // reserved in header
                                         0, 0));            // payload
        }
    }

    using RegisterData = standard::RegisterDataRequestMessage;

    RegisterData msg;
    REQUIRE(msg.name.empty());
    REQUIRE(msg.is<RegisterData::Empty>());
    REQUIRE(!msg.is<RegisterData::Unstructured>());
    REQUIRE(msg.as<RegisterData::Empty>() != nullptr);
    REQUIRE(msg.as<RegisterData::String>() == nullptr);

    REQUIRE(msg == RegisterData());
    REQUIRE((msg != RegisterData()) == false);

    {
        const auto encoded = msg.encode();
        REQUIRE(encoded.size() == 10);
        REQUIRE(encoded == makeArray(std::uint8_t(MessageID::RegisterDataRequest), 0,  // msg ID
                                     0, 0, 0, 0, 0, 0,  // reserved in header
                                     0, 0));            // payload
    }

    msg.name = "1234567";

    REQUIRE(msg != RegisterData());
    REQUIRE((msg == RegisterData()) == false);

    {
        const auto encoded = msg.encode();
        REQUIRE(encoded.size() == 17);
        REQUIRE(encoded == makeArray(std::uint8_t(MessageID::RegisterDataRequest), 0,   // msg ID
                                     0, 0, 0, 0, 0, 0,                                  // reserved in header
                                     0,                                                 // type ID
                                     7,                                                 // name length
                                     49, 50, 51, 52, 53, 54, 55));                      // name
    }

    while (msg.name.length() != msg.name.max_size())
    {
        msg.name.push_back('Z');
    }
    REQUIRE(msg.name.length() == 93);

    {
        const auto encoded = msg.encode();
        REQUIRE(encoded.size() == 10 + 93);
        const auto reference = makeArray(
            std::uint8_t(MessageID::RegisterDataRequest), 0,                                 // msg ID
            0, 0, 0, 0, 0, 0,                                                                // reserved in header
            0,                                                                               // type ID
            93,                                                                              // name length
            49, 50, 51, 52, 53, 54, 55, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,  // name
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90
        );
        std::cout << "ENCODED (Z):" << std::endl;
        printHexDump(encoded);
        std::cout << "REFERENCE (Z):" << std::endl;
        printHexDump(reference);
        REQUIRE(encoded.size() == reference.size());
        REQUIRE(encoded == reference);
    }

    msg.name.clear();
    msg.value.emplace<RegisterData::String>("1234567");
    REQUIRE(!msg.is<RegisterData::Empty>());
    REQUIRE(msg.is<RegisterData::String>());
    REQUIRE(msg.as<RegisterData::Empty>() == nullptr);
    REQUIRE(msg.as<RegisterData::String>() != nullptr);

    {
        const auto encoded = msg.encode();
        std::cout << "ENCODED (value '1234567'):" << std::endl;
        printHexDump(encoded);
        REQUIRE(encoded.size() == 17);
        REQUIRE(encoded == makeArray(std::uint8_t(MessageID::RegisterDataRequest), 0,  // msg ID
                                     0, 0, 0, 0, 0, 0,                          // reserved in header
                                     1,                                         // type ID
                                     0,                                         // name length
                                     49, 50, 51, 52, 53, 54, 55));              // value
    }

    while (msg.name.length() != msg.name.max_size())
    {
        msg.name.push_back('Z');
    }

    msg.value.emplace<RegisterData::U64>();
    while (msg.as<RegisterData::U64>()->size() != msg.as<RegisterData::U64>()->max_size())
    {
        msg.as<RegisterData::U64>()->push_back(0xDEAD'BEEF'BADC'0FFEULL);
    }

    REQUIRE(msg.name.length() == 93);
    REQUIRE(msg.as<RegisterData::U64>()->size() == 32);
    REQUIRE(msg.as<RegisterData::U64>()->size() == RegisterData::U64::Capacity);

    {
        const auto encoded = msg.encode();
        REQUIRE(encoded.size() == 10 + 93 + 256);
        const auto reference = makeArray(
            std::uint8_t(MessageID::RegisterDataRequest), 0,                                 // msg ID
            0, 0, 0, 0, 0, 0,                                                                // reserved in header
            8,                                                                               // type ID
            93,                                                                              // name length
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,  // name
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,  // value
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE,
            0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE, 0xFE, 0x0F, 0xDC, 0xBA, 0xEF, 0xBE, 0xAD, 0xDE
        );
        std::cout << "ENCODED (dead beef, bad covfefe):" << std::endl;
        printHexDump(encoded);
        std::cout << "REFERENCE (dead beef, bad covfefe):" << std::endl;
        printHexDump(reference);
        REQUIRE(encoded.size() == reference.size());
        REQUIRE(encoded == reference);
    }

    msg.name = "0";
    msg.value.emplace<RegisterData::Boolean>({false, true, false, true});

    {
        const auto encoded = msg.encode();
        REQUIRE(encoded.size() == 10 + 1 + 4);
        const auto reference = makeArray(
            std::uint8_t(MessageID::RegisterDataRequest), 0,                                 // msg ID
            0, 0, 0, 0, 0, 0,                                                                // reserved in header
            3,                                                                               // type ID
            1,                                                                               // name length
            48,                                                                              // name
            0, 1, 0, 1                                                                       // value
        );
        std::cout << "ENCODED (bool):" << std::endl;
        printHexDump(encoded);
        std::cout << "REFERENCE (bool):" << std::endl;
        printHexDump(reference);
        REQUIRE(encoded.size() == reference.size());
        REQUIRE(encoded == reference);
    }

    std::uint8_t demo_buffer[] = {1, 2, 3, 4, 5};

    msg.name = "1";
    msg.value.emplace<RegisterData::Unstructured>(5, &demo_buffer[0]);

    {
        const auto encoded = msg.encode();
        REQUIRE(encoded.size() == 10 + 1 + 5);
        const auto reference = makeArray(
            std::uint8_t(MessageID::RegisterDataRequest), 0,                                 // msg ID
            0, 0, 0, 0, 0, 0,                                                                // reserved in header
            2,                                                                               // type ID
            1,                                                                               // name length
            49,                                                                              // name
            1, 2, 3, 4, 5                                                                    // value
        );
        std::cout << "ENCODED (unstructured):" << std::endl;
        printHexDump(encoded);
        std::cout << "REFERENCE (unstructured):" << std::endl;
        printHexDump(reference);
        REQUIRE(encoded.size() == reference.size());
        REQUIRE(encoded == reference);
    }
}


TEST_CASE("RegisterDataDecoding")
{
    using standard::MessageID;
    using standard::RegisterDataRequestMessage;

    constexpr std::uint8_t M = std::uint8_t(MessageID::RegisterDataRequest);

    const auto go = [](auto... values)
    {
        const auto data = makeArray(values...);
        return RegisterDataRequestMessage::tryDecode(data.begin(), data.end());
    };

    REQUIRE_FALSE(go());
    REQUIRE_FALSE(go(0));
    REQUIRE_FALSE(go(M, 0));
    REQUIRE_FALSE(go(0, 0, 0));
    REQUIRE_FALSE(go(M, 0, 0, 0));
    REQUIRE_FALSE(go(0, 0, 0, 0, 0));
    REQUIRE_FALSE(go(M, 0, 0, 0, 0, 0));
    REQUIRE_FALSE(go(0, 0, 0, 0, 0, 0, 0));
    REQUIRE_FALSE(go(M, 0, 0, 0, 0, 0, 0, 0));
    REQUIRE_FALSE(go(0, 0, 0, 0, 0, 0, 0, 0, 0));
    REQUIRE_FALSE(go(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0, 0, 0));

    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0, 0, 0));
    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0, 0, 0)->name.empty());
    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0, 0, 0)->is<RegisterDataRequestMessage::Empty>());
    // Payload ignored for empty register values:
    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3)->is<RegisterDataRequestMessage::Empty>());

    REQUIRE_FALSE(go(M, 0, 0, 0, 0, 0, 0, 0,
                     99, 0));                  // Bad type ID

    REQUIRE_FALSE(go(M, 0, 0, 0, 0, 0, 0, 0,
                     0, 99));                  // Bad name length

    REQUIRE_FALSE(go(M, 0, 0, 0, 0, 0, 0, 0,
                     0, 1));                   // Bad name length

    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0,
                     0, 1, 49)->name == "1");

    REQUIRE_FALSE(go(M, 0, 0, 0, 0, 0, 0, 0,
                     0, 2, 49));               // Bad name length

    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0,
                     1, 1, 49, 48)->name == "1");
    REQUIRE      (go(M, 0, 0, 0, 0, 0, 0, 0,
                     1, 1, 49, 48)->as<RegisterDataRequestMessage::String>()->operator==("0"));
}


template <std::size_t Capacity>
static inline void fillRandomString(util::FixedCapacityString<Capacity>& out_string)
{
    out_string.clear();
    std::size_t size = (std::size_t(getRandomByte()) * std::size_t(getRandomByte())) % Capacity;
    while (size --> 0)
    {
        out_string.push_back(char((getRandomByte() % 94) + 33));
    }
}


template <typename T, std::size_t Capacity>
static void fillRandomVector(util::FixedCapacityVector<T, Capacity>& out_vector)
{
    out_vector.clear();
    std::size_t size = (std::size_t(getRandomByte()) * std::size_t(getRandomByte())) % Capacity;
    out_vector.resize(size);
    for (auto& x : out_vector)
    {
        x = getRandomNumber<sizeof(T), std::is_signed_v<T>, std::is_floating_point_v<T>>();
    }
}


template <std::uint8_t CandidateVariantTypeIndex = 0>
static void fillRandomRegisterValue(standard::RegisterData::Value& value, const std::uint8_t variant_type_index)
{
    using standard::RegisterData;
    if constexpr (CandidateVariantTypeIndex < std::variant_size_v<standard::RegisterData::Value>)
    {
        if (CandidateVariantTypeIndex == variant_type_index)
        {
            using Type = std::variant_alternative_t<CandidateVariantTypeIndex, standard::RegisterData::Value>;
            auto& ref = value.template emplace<CandidateVariantTypeIndex>();
            if constexpr (std::is_same_v<Type, std::monostate>)
            {
                ;   // Nothing to do
            }
            else if constexpr (std::is_same_v<Type, RegisterData::String>)
            {
                fillRandomString(ref);
            }
            else
            {
                fillRandomVector(ref);
            }
        }
        else
        {
            fillRandomRegisterValue<CandidateVariantTypeIndex + 1>(value, variant_type_index);
        }
    }
    else
    {
        assert(false);
    }
}


template <typename T>
static T makeRandomRegisterData()
{
    T msg;
    fillRandomString(msg.name);
    fillRandomRegisterValue(msg.value,
                            std::uint8_t(getRandomByte() % std::variant_size_v<typename T::Value>));
    return msg;
}


struct ValuePrinter
{
    template <typename T, std::size_t Capacity>
    void operator()(const util::FixedCapacityVector<T, Capacity>& vector) const
    {
        std::cout << "Vector of "
                  << (std::is_same_v<T, bool> ? "bool" :
                        (std::is_integral_v<T> ? (std::is_signed_v<T> ? "signed" : "unsigned") :
                            (std::is_floating_point_v<T> ? "real" : "unknown")))
                  << " " << (sizeof(T) * 8) << "-bit "
                  << "[<=" << Capacity << "]:";
        for (auto& x : vector)
        {
            std::cout << " " << std::conditional_t<(sizeof(T) < 2), long, T>(x);
        }
        std::cout << std::endl;
    }

    void operator()(const standard::RegisterData::Unstructured& data) const
    {
        std::cout << "Unstructured:\n";
        printHexDump(data);
    }

    template <std::size_t Capacity>
    void operator()(const util::FixedCapacityString<Capacity>& string) const
    {
        std::cout << "String: " << string.c_str() << std::endl;
    }

    void operator()(std::monostate) const
    {
        std::cout << "Empty" << std::endl;
    }
};


static void printRegisterData(const standard::RegisterData& rd)
{
    std::cout << "Register name:  " << rd.name.c_str() << std::endl;
    std::cout << "Register value: ";
    std::visit(ValuePrinter(), rd.value);
}


TEST_CASE("RegisterDataEncodingDecodingLoop-slow")
{
    using RegisterData = standard::RegisterDataRequestMessage;

    std::cout << "Below are several randomly generated register data structs printed for debugging needs:\n"
              << "---------\n";
    for (std::size_t i = 0; i < 10; i++)
    {
        std::cout << i << ":\n";
        printRegisterData(makeRandomRegisterData<RegisterData>());
    }
    std::cout << "---------\nEnd of randomly generated registers" << std::endl;

    constexpr long long NumberOfIterations = 3'000'000;
    std::size_t real_comparison_failures = 0;

    for (long long ago = 0; ago < NumberOfIterations; ago++)
    {
        if (ago % 100'000 == 0)
        {
            std::cout << "\r" << ago << "/" << NumberOfIterations << "  \r" << std::flush;
        }

        const RegisterData synthesized = makeRandomRegisterData<RegisterData>();

        // Testing the alternative encoding API
        standard::DynamicMessageBuffer<RegisterData::MaxEncodedSize> encoded;
        encoded.resize(encoded.capacity());
        encoded.resize(synthesized.encode(encoded.begin()));

        const auto maybe_decoded = RegisterData::tryDecode(encoded.begin(), encoded.end());
        REQUIRE(maybe_decoded);
        const RegisterData decoded = *maybe_decoded;

        REQUIRE(decoded.name == synthesized.name);
        REQUIRE(decoded.value.index() == synthesized.value.index());    // Paranoia

        const auto decoded_then_encoded = decoded.encode();

        if (decoded_then_encoded != encoded)
        {
            std::cout << "decoded_then_encoded != encoded" << std::endl;
            std::cout << "Where decoded_then_encoded:\n";
            printHexDump(decoded_then_encoded);
            std::cout << "Where encoded:\n";
            printHexDump(encoded);
            FAIL();
        }

        if (decoded != synthesized)
        {
            // Floating point vectors may contain NaN, which are expected to compare non-equal.
            // Therefore we ignore the non-equality for floating-point types.
            real_comparison_failures++;
            REQUIRE((decoded.is<RegisterData::F64>() || decoded.is<RegisterData::F32>()));
        }
    }

    std::cout << "\r" << NumberOfIterations << " ITERATIONS DONE; real non-equal comparisons: "
              << real_comparison_failures
              << " (" << 100.0 * double(real_comparison_failures) / double(NumberOfIterations) << "%)"
              << std::endl;
}


TEST_CASE("RegisterName")
{
    using N = standard::RegisterName;
    N n;

    const auto encode = [](N& value)
    {
        standard::DynamicMessageBuffer<N::MaxEncodedSize> buf;
        presentation::StreamEncoder encoder(std::back_inserter(buf));
        value.encode(encoder);
        REQUIRE(buf.size() == encoder.getOffset());
        REQUIRE(buf.size() >= N::MinEncodedSize);
        REQUIRE(buf.size() <= N::MaxEncodedSize);
        return buf;
    };

    const auto decode = [](auto... bytes) -> std::optional<N>
    {
        const auto data = makeArray(bytes...);
        N value;
        presentation::StreamDecoder decoder(data.begin(), data.end());
        if (N::tryDecode(decoder, value))
        {
            return value;
        }
        return {};
    };

    REQUIRE(encode(n) == makeArray(0));
    n += "123";
    REQUIRE(encode(n) == makeArray(3, 49, 50, 51));
    while (n.size() < n.max_size())
    {
        n.push_back('Z');
    }
    REQUIRE(encode(n) == makeArray(93, 49, 50, 51,
                                   90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                                   90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                                   90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                                   90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                                   90, 90, 90, 90, 90, 90, 90, 90, 90, 90));

    REQUIRE(!decode());
    REQUIRE(!decode(1));
    REQUIRE(!decode(94));
    REQUIRE(decode(0));
    REQUIRE(decode(0)->empty());

    REQUIRE(decode(1, 49));
    REQUIRE("1" == *decode(1, 49));

    {
        auto res = decode(93, 49, 50, 51,
                          90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                          90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                          90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                          90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90,
                          90, 90, 90, 90, 90, 90, 90, 90, 90, 90);
        REQUIRE(res);
        REQUIRE(n == *res);
    }
}
