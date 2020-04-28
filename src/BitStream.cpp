
#include <ctrigrid/BitStream.h>


namespace ctrigrid
{

BitStreamReader::BitStreamReader(const BitStreamBuffer& streamBuffer, uint8_t width) :
    m_streamBuffer(streamBuffer),
    m_width(width),
    m_startBitPos(0u),
    m_endBitPos(0u)
{
    CTRIGRID_ASSERT(m_width > 0u && width <= sizeof(ReadType) * 8u - 7u);
}

void 
BitStreamReader::Begin(BitStreamBuffer::BufferIndex startBitPos, BitStreamBuffer::BufferIndex endBitPos)
{
    CTRIGRID_ASSERT (endBitPos > startBitPos);

    m_startBitPos = startBitPos;
    m_endBitPos = endBitPos;
}

BitStreamReader::ReadType 
BitStreamReader::Next()
{
    // ref: https://fgiesen.wordpress.com/2018/02/19/reading-bits-in-far-too-many-ways-part-1/

    CTRIGRID_ASSERT(m_startBitPos < m_endBitPos);

    const uint8_t* buf = m_streamBuffer.GetData();

    // read 64bits from the byte containing the starting bit
    // TODO: gracefully handle the buffer is big enough to read 64 bits
    CTRIGRID_ASSERT((m_startBitPos >> 3u) + sizeof(uint64_t) < m_streamBuffer.GetSizeInBytes()); // 1 byte extra
    uint64_t bits = *((uint64_t*)(&buf[m_startBitPos >> 3u]));  // shift by 3 -> divide by 8

    // shift out the bytes that we are already ahead in the starting position
    bits >>= m_startBitPos & 7; // & 0x0111 -> modulus 8

    // mask out 64-width bits
    // TODO: use table instead of variable length shift
    const BitStreamReader::ReadType r = bits & ((1ull << m_width) - 1);

    m_startBitPos += m_width;

    return r;
}


BitStreamWriter::BitStreamWriter(BitStreamBuffer& streamBuffer, uint8_t width) :
    m_width(width),
    m_streamBuffer(streamBuffer)
{
    CTRIGRID_ASSERT(m_width > 0u && width <= sizeof(WriteType) * 8u - 7u);
}

void 
BitStreamWriter::Begin(size_t totalSizeBytes)
{
    m_streamBuffer.Clear();
    m_streamBuffer.Alloc(totalSizeBytes);
    m_streamBuffer.Set(0u);
    m_writeBitPos = 0u;
}

void 
BitStreamWriter::WriteNext(WriteType value)
{
    CTRIGRID_ASSERT(m_width > 0u);
    CTRIGRID_ASSERT((value >> m_width) == 0ull);

    uint64_t writeByte = m_writeBitPos >> 3u;
    CTRIGRID_ASSERT(writeByte + sizeof(uint64_t) < m_streamBuffer.GetSizeInBytes());
        
    uint64_t bits = (uint64_t)value;

    // shift bits so that they do not overwrite existing written bits
    const int shift = m_writeBitPos & 7;
    bits <<= shift; // & 0x0111 -> modulus 8

    const uint8_t* buf = m_streamBuffer.GetData();

    // set previously written bits if needed
    if (shift > 0)
    {
        const int mask = ~(~0 << shift);
        bits |= mask & buf[writeByte];
    }

    *((uint64_t*)(&buf[writeByte])) = bits;

    m_writeBitPos += m_width;
}

void 
BitStreamWriter::Finalize()
{
    // append enough zeros that it can be read safely by the reader
    uint64_t writeByte = m_writeBitPos >> 3u;

    m_streamBuffer.Resize((size_t)(writeByte + sizeof(uint64_t) + 1u));   // one byte extra
}

}