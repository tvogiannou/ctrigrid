
#include <gtest/gtest.h>

#include <ctrigrid/BitStream.h>


TEST(BitStreamUnitTests, BitStreamWrite)
{
    using namespace ctrigrid;

    BitStreamBuffer streamBuf;

    // test width smaller than a byte
    {
        constexpr uint8_t width = 5u;
        constexpr size_t totalSize = 12u;

        BitStreamWriter writer(streamBuf, width);
        writer.Begin(totalSize);
        EXPECT_EQ(writer.GetCurrentWritePos(), 0u);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x00);
        EXPECT_EQ(streamBuf.GetSizeInBytes(), totalSize);

        writer.WriteNext(7);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0x07);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x00);

        writer.WriteNext(15);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0xe7);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0x01);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x00);

        writer.WriteNext(16);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0xe7);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0x41);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x00);

        writer.WriteNext(31);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0xe7);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0xc1);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x0f);

        writer.Finalize();
        EXPECT_EQ(writer.GetCurrentWritePos(), 4u * width);
        EXPECT_EQ(streamBuf.GetSizeInBytes(), 3u + sizeof(uint64_t));
    }

    // test width larger than byte
    {
        streamBuf.Clear();

        constexpr uint8_t width = 10u;
        constexpr size_t totalSize = 14u;

        BitStreamWriter writer(streamBuf, width);
        writer.Begin(totalSize);
        EXPECT_EQ(writer.GetCurrentWritePos(), 0u);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(3), 0x00);
        EXPECT_EQ(streamBuf.GetSizeInBytes(), totalSize);

        writer.WriteNext(16);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0x10);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x00);
        EXPECT_EQ(streamBuf.GetByteValue(3), 0x00);

        writer.WriteNext(255);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0x10);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0xfc);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0x03);
        EXPECT_EQ(streamBuf.GetByteValue(3), 0x00);

        writer.WriteNext(1023);
        EXPECT_EQ(streamBuf.GetByteValue(0), 0x10);
        EXPECT_EQ(streamBuf.GetByteValue(1), 0xfc);
        EXPECT_EQ(streamBuf.GetByteValue(2), 0xf3);
        EXPECT_EQ(streamBuf.GetByteValue(3), 0x3f);

        writer.Finalize();
        EXPECT_EQ(writer.GetCurrentWritePos(), 3u * width);
        EXPECT_EQ(streamBuf.GetSizeInBytes(), 4u + sizeof(uint64_t));
    }
}

TEST(BitStreamUnitTests, BitStreamRead)
{
    using namespace ctrigrid;

    BitStreamBuffer streamBuf;

    // test width smaller than a byte
    {
        constexpr uint8_t width = 5u;
        
        // buffers has the output for writing 7, 15, 16, 31 
        streamBuf.Alloc(10u);
        streamBuf.Set(0u);
        streamBuf.SetByteValue(0, 0xe7);
        streamBuf.SetByteValue(1, 0xc1);
        streamBuf.SetByteValue(2, 0x0f);

        BitStreamReader reader(streamBuf, width);
        reader.Begin(0u, 4u * width);
        EXPECT_FALSE(reader.Finished());

        uint64_t val = reader.Next();
        EXPECT_EQ(val, 7);
        EXPECT_FALSE(reader.Finished());

        val = reader.Next();
        EXPECT_EQ(val, 15);
        EXPECT_FALSE(reader.Finished());

        val = reader.Next();
        EXPECT_EQ(val, 16);
        EXPECT_FALSE(reader.Finished());

        val = reader.Next();
        EXPECT_EQ(val, 31);
        EXPECT_TRUE(reader.Finished());
    }

    // test width larger than a byte
    {
        streamBuf.Clear();

        constexpr uint8_t width = 10u;

        // buffers has the output for writing 16, 255, 1023
        streamBuf.Alloc(12u);
        streamBuf.Set(0u);
        streamBuf.SetByteValue(0, 0x10);
        streamBuf.SetByteValue(1, 0xfc);
        streamBuf.SetByteValue(2, 0xf3);
        streamBuf.SetByteValue(3, 0x3f);

        BitStreamReader reader(streamBuf, width);
        reader.Begin(0u, 3u * width);
        EXPECT_FALSE(reader.Finished());

        uint64_t val = reader.Next();
        EXPECT_EQ(val, 16);
        EXPECT_FALSE(reader.Finished());

        val = reader.Next();
        EXPECT_EQ(val, 255);
        EXPECT_FALSE(reader.Finished());

        val = reader.Next();
        EXPECT_EQ(val, 1023);
        EXPECT_TRUE(reader.Finished());
    }
}

TEST(BitStreamUnitTests, BitStreamWriteRead)
{
    using namespace ctrigrid;

    // write all values that can be represented by width bits
    constexpr uint8_t width = 19u;
    constexpr uint64_t maxValue = ~(~0ull << width);
    constexpr size_t totalSize = (maxValue * width) / 8u + sizeof(uint64_t);

    BitStreamBuffer streamBuf;
    BitStreamWriter writer(streamBuf, width);
    writer.Begin(totalSize);
    for (uint64_t i = 0u; i < maxValue; ++i)
        writer.WriteNext(i);
    writer.Finalize();
            
    BitStreamReader reader(streamBuf, width);
    reader.Begin(0u, maxValue * width);
    for (uint64_t i = 0u; i < maxValue; ++i)
    {
        EXPECT_FALSE(reader.Finished());
        uint64_t val = reader.Next();
        EXPECT_EQ(val, i);
    }
    EXPECT_TRUE(reader.Finished());
}

#ifdef NDEBUG
TEST(BitStreamUnitTests, DISABLED_TestBitStreamInvalid)
#else
TEST(BitStreamUnitTests, TestBitStreamInvalid)
#endif
{
    using namespace ctrigrid;

    BitStreamBuffer streamBuf;

    // invalid constructor input
    {
        EXPECT_DEATH({ BitStreamWriter writer(streamBuf, 0u); }, "");
        EXPECT_DEATH({ BitStreamWriter writer(streamBuf, sizeof(BitStreamWriter::WriteType) * 8u); }, "");

        EXPECT_DEATH({ BitStreamReader reader(streamBuf, 0u); }, "");
        EXPECT_DEATH({ BitStreamReader reader(streamBuf, sizeof(BitStreamReader::ReadType) * 8u); }, "");
    }

    const uint8_t width = 4u;

    // test invalid cases for writer
    {
        BitStreamWriter writer(streamBuf, width);

        // cannot write without allocating
        EXPECT_DEATH({ writer.WriteNext(7); }, "");

        writer.Begin(1u + sizeof(uint64_t));

        // cannot write value that requires more than width bits
        EXPECT_DEATH({ writer.WriteNext(16); }, "");

        writer.WriteNext(2);
        writer.WriteNext(7);

        // cannot write more values than allocated
        EXPECT_DEATH({ writer.WriteNext(3); }, "");
    }

    // test invalid cases for reader
    {
        BitStreamReader reader(streamBuf, width);

        // cannot read without beginning
        EXPECT_DEATH({ BitStreamReader::ReadType v = reader.Next(); CTRIGRID_UNUSED(v); }, "");

        reader.Begin(0u, 9u);
        BitStreamReader::ReadType v = reader.Next();
        v = reader.Next();

        // cannot read after end bit position
        EXPECT_DEATH({ BitStreamReader::ReadType vv = reader.Next(); CTRIGRID_UNUSED(vv); }, "");

        reader.Begin(0u, 16u);  // set end bit pos to go beyond buffer
        v = reader.Next();
        v = reader.Next();

        // cannot read outside buffer
        EXPECT_DEATH({ BitStreamReader::ReadType vv = reader.Next(); CTRIGRID_UNUSED(vv); }, "");
        CTRIGRID_UNUSED(v);
    }
}
