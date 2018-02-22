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

#include <popcop.hpp>
#include <cstdlib>  // NOLINT

#define CATCH_CONFIG_MAIN
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
inline bool doesEmitterOutputMatch(transport::Emitter encoder,
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


TEST_CASE("EmitterSimple")
{
    using transport::FrameDelimiter;
    using transport::EscapeCharacter;

    REQUIRE(doesEmitterOutputMatch(transport::Emitter(123, "", 0),
                                   makeArray(FrameDelimiter, 123, 0x67, 0xAC, 0x6C, 0xBA, FrameDelimiter)));

    REQUIRE(doesEmitterOutputMatch(transport::Emitter(90, makeArray(42, 12, 34, 56, 78)),
                                   makeArray(FrameDelimiter, 42, 12, 34, 56, 78, 90, 0xCE, 0x4E, 0x88, 0xBC,
                                             FrameDelimiter)));

    REQUIRE(doesEmitterOutputMatch(transport::Emitter(EscapeCharacter, makeArray(FrameDelimiter)),
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
    transport::Emitter encoder(frame_type_code, frame_payload.data(), frame_payload.size());
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


inline std::uint8_t getRandomByte()
{
    return std::uint8_t(std::rand());  // NOLINT
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


TEST_CASE("EmitterParserLoop")
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

        REQUIRE(validateEncodeDecodeLoop(parser,
                                         getRandomByte(),
                                         payload,
                                         extraneous));
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


TEST_CASE("ScalarCodec")
{
    std::array<std::uint8_t, 64> underlying_buffer{};
    presentation::ScalarEncoder codec(underlying_buffer);

    REQUIRE(codec.getUnderlyingBufferSize() == 64);
    REQUIRE(codec.getSubCodec(0).getUnderlyingBufferSize() == 64);
    REQUIRE(codec.getSubCodec(1).getUnderlyingBufferSize() == 63);
    REQUIRE(codec.getSubCodec(64).getUnderlyingBufferSize() == 0);

    // Zero check
    REQUIRE(codec.get<std::uint8_t>(0) == 0);
    REQUIRE(codec.get<std::int16_t>(0) == 0);
    REQUIRE(codec.get<std::uint32_t>(0) == 0);
    REQUIRE(codec.get<std::int64_t>(0) == 0);
    REQUIRE(std::abs(codec.get<float>(0)) < 1e-6F);
    REQUIRE(std::abs(codec.get<double>(0)) < 1e-9);
    REQUIRE(codec.getASCIIString<10>(0).empty());

    // Simple patterns
    codec.set<std::uint8_t>(0, 48);
    codec.set(1, std::int16_t(49 + (50 << 8)));
    codec.set(3, std::int32_t(51 + (52 << 8) + (53 << 16) + (54 << 24)));

    std::cout << "Buffer state:\n";
    printHexDump(underlying_buffer);

    {
        std::array<std::uint8_t, 7> buf{};
        codec.get(0, std::begin(buf), std::end(buf));
        REQUIRE(std::equal(buf.begin(), buf.end(), makeArray(48, 49, 50, 51, 52, 53, 54).begin()));
    }

    REQUIRE(codec.getASCIIString<10>(0) == "0123456");

    codec.setASCIIString<5>(7, "789");
    REQUIRE(codec.getASCIIString<10>(0) == "0123456789");

    {
        const auto a = makeArray(58, 59, 60);
        codec.set(10, a.begin(), a.end());
    }

    std::cout << "Buffer state:\n";
    printHexDump(underlying_buffer);
    REQUIRE(codec.getASCIIString<60>(0) == "0123456789:;<");

    // struct.unpack('<q', b'12345678')
    REQUIRE(codec.get<std::int64_t>(1) == 4050765991979987505LL);
    REQUIRE(codec.getSubCodec(1).get<std::int64_t>(0) == 4050765991979987505LL);

    // Emitter check
    {
        transport::Parser<> p;
        transport::Emitter e = codec.makeEmitter(presentation::StandardFrameTypeCode, 14);
        while (true)
        {
            const auto out = p.processNextByte(e.getNextByte());
            REQUIRE(out.getExtraneousData() == nullptr);
            if (auto f = out.getReceivedFrame())
            {
                REQUIRE(e.isFinished());
                REQUIRE(f->type_code == presentation::StandardFrameTypeCode);
                REQUIRE(f->payload.size() == 14);
                REQUIRE(std::equal(f->payload.begin(), f->payload.end(), underlying_buffer.begin()));
                break;
            }
        }
    }
}


TEST_CASE("ScalarCodecBoundaries")
{
    std::array<std::uint8_t, 4> underlying_buffer{};
    presentation::ScalarEncoder codec(underlying_buffer);

    REQUIRE(codec.getUnderlyingBufferSize() == 4);
    REQUIRE(codec.getSubCodec(0).getUnderlyingBufferSize() == 4);
    REQUIRE(codec.getSubCodec(1).getUnderlyingBufferSize() == 3);
    REQUIRE(codec.getSubCodec(2).getUnderlyingBufferSize() == 2);
    REQUIRE(codec.getSubCodec(3).getUnderlyingBufferSize() == 1);
    REQUIRE(codec.getSubCodec(4).getUnderlyingBufferSize() == 0);

    REQUIRE(codec.get<std::uint8_t>(0) == 0);
    REQUIRE(codec.get<std::uint8_t>(1) == 0);
    REQUIRE(codec.get<std::uint8_t>(2) == 0);
    REQUIRE(codec.get<std::uint8_t>(3) == 0);

    codec.set<std::uint8_t>(0, 0x34);
    codec.set<std::uint8_t>(1, 0x12);
    codec.set<std::uint8_t>(2, 0x56);
    codec.set<std::uint8_t>(3, 0x78);

    REQUIRE(codec.get<std::uint16_t>(0) == 0x1234U);
    REQUIRE(codec.get<std::uint16_t>(1) == 0x5612U);
    REQUIRE(codec.get<std::uint16_t>(2) == 0x7856U);

    REQUIRE(codec.get<std::uint32_t>(0) == 0x78561234UL);

    REQUIRE(codec.get<std::uint8_t>(codec.getUnderlyingBufferSize() - 1)  == 0x78U);
    REQUIRE(codec.get<std::uint16_t>(codec.getUnderlyingBufferSize() - 2) == 0x7856U);
    REQUIRE(codec.get<std::uint32_t>(codec.getUnderlyingBufferSize() - 4) == 0x78561234UL);
}


TEST_CASE("NodeInfoMessageCodec")
{
    std::array<std::uint8_t, 372> carefully_crafted_message
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

    // Check the message codec
    presentation::ScalarEncoder codec(carefully_crafted_message);
    standard::MessageHeaderView<presentation::ScalarEncoder> header(codec);

    REQUIRE(codec.getUnderlyingBufferPointer() == carefully_crafted_message.data());
    REQUIRE(codec.getUnderlyingBufferSize() == carefully_crafted_message.size());

    REQUIRE(!header.isMessagePayloadEmpty());
    REQUIRE(header.doesMessageIDMatch<standard::NodeInfoView>());
    REQUIRE(header.getMessageView<standard::NodeInfoView>());

    // Using explicit type intentionally
    const std::optional<standard::NodeInfoView<presentation::ScalarEncoder>> optional =
        header.getMessageView<standard::NodeInfoView>();

    REQUIRE(optional->getSoftwareImageCRCIfAvailable() == 0xFFDEBC9A78563412ULL);
    REQUIRE(optional->getSoftwareVCSCommitID() == 0xDEADBEEFULL);

    REQUIRE(optional->getSoftwareVersion().major == 1);
    REQUIRE(optional->getSoftwareVersion().minor == 2);

    REQUIRE(optional->getHardwareVersion().major == 3);
    REQUIRE(optional->getHardwareVersion().minor == 4);

    REQUIRE(optional->getMode() == standard::NodeInfoView<presentation::ScalarEncoder>::Mode::Normal);

    standard::NodeInfoView<presentation::ScalarEncoder> m = *optional;

    m.clearSoftwareImageCRC();
    REQUIRE(!m.getSoftwareImageCRCIfAvailable());

    REQUIRE(m.getSoftwareReleaseBuildFlag());
    REQUIRE(m.getSoftwareDirtyBuildFlag());
    m.setSoftwareReleaseBuildFlag(false);
    REQUIRE(!m.getSoftwareReleaseBuildFlag());
    REQUIRE(m.getSoftwareDirtyBuildFlag());
    m.setSoftwareDirtyBuildFlag(false);
    REQUIRE(!m.getSoftwareReleaseBuildFlag());
    REQUIRE(!m.getSoftwareDirtyBuildFlag());
    m.setSoftwareReleaseBuildFlag(true);
    REQUIRE(m.getSoftwareReleaseBuildFlag());
    REQUIRE(!m.getSoftwareDirtyBuildFlag());

    REQUIRE(m.getSoftwareBuildTimestampUTC() == 0xBADF00D2);
    m.setSoftwareBuildTimestampUTC(123);
    REQUIRE(m.getSoftwareBuildTimestampUTC() == 123);

    {
        const auto arr = m.getGloballyUniqueID();
        const auto ref = makeArray(16, 15, 14, 13, 12, 11, 10, 9,
                                    8,  7,  6,  5,  4,  3,  2, 1);
        REQUIRE(std::equal(arr.begin(), arr.end(), ref.begin()));
    }

    REQUIRE(m.getNodeName() == "Hello!");
    REQUIRE(m.getNodeDescription() == "Space!");
    REQUIRE(m.getBuildEnvironmentDescription() == "upyachka");
    REQUIRE(m.getRuntimeEnvironmentDescription() == "RUNTIME!");

    {
        std::array<std::uint8_t, 255> arr{};
        const auto ref = makeArray(1, 2, 3, 4);
        REQUIRE(m.getCertificateOfAuthenticity(arr.begin()) == 4);
        REQUIRE(std::equal(arr.begin(), arr.begin() + 4, ref.begin()));
    }

    REQUIRE(standard::NodeInfoView<presentation::ScalarEncoder>::getSerializedSize(0) == 360);
    REQUIRE(standard::NodeInfoView<presentation::ScalarEncoder>::getSerializedSize(255) == 615);

    /*
     * Trying to re-create the above message
     */
    standard::MessageConstructionHelper<standard::NodeInfoView,
                                        standard::NodeInfoView<>::getSerializedSize(255)> new_message;

    // Software CRC was cleared
    //new_message.setSoftwareImageCRC(0xFFDEBC9A78563412ULL);
    new_message.setSoftwareVCSCommitID(0xDEADBEEFULL);
    new_message.setSoftwareVersion({1, 2});
    new_message.setHardwareVersion({3, 4});
    new_message.setMode(standard::NodeInfoView<>::Mode::Normal);
    new_message.setSoftwareReleaseBuildFlag(true);
    new_message.setSoftwareBuildTimestampUTC(123);
    new_message.setGloballyUniqueID(makeArray(16, 15, 14, 13, 12, 11, 10, 9,
                                               8,  7,  6,  5,  4,  3,  2, 1));
    new_message.setNodeName("Hello!");
    new_message.setNodeDescription("Space!");
    new_message.setBuildEnvironmentDescription("upyachka");
    new_message.setRuntimeEnvironmentDescription("RUNTIME!");
    new_message.setCertificateOfAuthenticity(makeArray(1, 2, 3, 4));

    std::cout << "Manually constructed:" << std::endl;
    printHexDump(carefully_crafted_message);
    std::cout << "Rendered:" << std::endl;
    printHexDump(new_message.getSerializedMessageWithHeader());

    REQUIRE(std::equal(carefully_crafted_message.begin(), carefully_crafted_message.end(),
                       new_message.getSerializedMessageWithHeader().begin()));
}
