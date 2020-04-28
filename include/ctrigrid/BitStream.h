#pragma once

#include <ctrigrid/Compiler.h>

#include <vector>


namespace ctrigrid
{

// utility struct to wrap a "raw" byte buffer
struct BitStreamBuffer
{
    using BufferIndex = uint64_t;
    using BufferData = std::vector<uint8_t>;
    using BufferIndexArray = std::vector<uint64_t>;

    // API that needs to be supported by the bit stream
    // for the moment it simply forwards to a std vector
    void Clear() { m_data.clear(); }
    void Alloc(size_t Nbytes) { m_data.resize(Nbytes, 0u); }
    void Resize(size_t Nbytes) { m_data.resize(Nbytes, 0u); }
    void Set(uint8_t byteValue) { m_data.assign(m_data.size(), byteValue); }

    size_t GetSizeInBytes() const { return m_data.size(); }
    size_t GetSizeInBits() const { return m_data.size() * 8u; }
    uint8_t GetByteValue(size_t bytePos) const { return m_data.at(bytePos); }
    uint8_t* GetData() { return m_data.data(); }
    const uint8_t* GetData() const { return m_data.data(); }

    // internal
    // packs so that no more memory than required is used
    void Pack() { m_data.shrink_to_fit(); }
    void SetByteValue(size_t bytePos, uint8_t val) { m_data.at(bytePos) = val; }


    BufferData m_data;
};

// reads a BitStreamBuffer at specified bit width values
class BitStreamReader
{
public:

    using ReadType = uint64_t;  // read 64 bits
    static const ReadType InvalidIndex = ~0ull;

    BitStreamReader(const BitStreamBuffer& streamBuffer, uint8_t width);

    const BitStreamBuffer& m_streamBuffer;
    const uint8_t m_width;

    void Begin(BitStreamBuffer::BufferIndex startBitPos, BitStreamBuffer::BufferIndex endBitPos);
    ReadType Next();
    bool Finished() const { return m_startBitPos >= m_endBitPos; }

private:

    BitStreamBuffer::BufferIndex m_startBitPos;
    BitStreamBuffer::BufferIndex m_endBitPos;
};

// writes a BitStreamBuffer at specified bit width values
class BitStreamWriter
{
public:
    using WriteType = uint64_t;  // read 64 bits

    const uint8_t m_width;

    BitStreamWriter(BitStreamBuffer& streamBuffer, uint8_t width);

    void Begin(size_t totalSizeBytes);
    void WriteNext(WriteType value);
    void Finalize();
    BitStreamBuffer::BufferIndex GetCurrentWritePos() const { return m_writeBitPos; }

private:

    BitStreamBuffer& m_streamBuffer;
    BitStreamBuffer::BufferIndex m_writeBitPos = 0u;
};


}