#include "SPIEx.h"

#if defined(PLATFORM_ESP32)
#include <soc/spi_struct.h>
#endif

void ICACHE_RAM_ATTR SPIExClass::_transfer(uint8_t cs_mask, uint8_t *data, uint32_t size, bool reading)
{
#if defined(PLATFORM_ESP32)
    spi_dev_t *spi = *(reinterpret_cast<spi_dev_t**>(bus()));
    // wait for SPI to become non-busy
    while(spi->cmd.usr) {}

    // Set the CS pins which we want controlled by the SPI module for this operation
    spiDisableSSPins(bus(), ~cs_mask);
    spiEnableSSPins(bus(), cs_mask);

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    spi->ms_dlen.ms_data_bitlen = (size*8)-1;
#else
    spi->mosi_dlen.usr_mosi_dbitlen = ((size * 8) - 1);
    spi->miso_dlen.usr_miso_dbitlen = ((size * 8) - 1);
#endif

    // write the data to the SPI FIFO
    const uint32_t words = (size + 3) / 4;
    auto * const wordsBuf = reinterpret_cast<uint32_t *>(data);
    for(int i=0; i<words; i++)
    {
        spi->data_buf[i] = wordsBuf[i]; //copy buffer to spi fifo
    }

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
    spi->cmd.update = 1;
    while (spi->cmd.update) {}
#endif
    // start the SPI module
    spi->cmd.usr = 1;

    if (reading)
    {
        // wait for SPI write to complete
        while(spi->cmd.usr) {}

        for(int i=0; i<words; i++)
        {
            wordsBuf[i] = spi->data_buf[i]; //copy spi fifo to buffer
        }
    }
#elif defined(PLATFORM_ESP8266)
    // we support only one hardware controlled CS pin, so theres nothing to do to configure it at this point

    // wait for SPI to become non-busy
    while(SPI1CMD & SPIBUSY) {}

    // Set in/out Bits to transfer
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    const auto bits = (size * 8) - 1;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));

    // write the data to the SPI FIFO
    volatile uint32_t * const fifoPtr = &SPI1W0;
    const uint8_t outSize = ((size + 3) / 4);
    uint32_t * const dataPtr = (uint32_t*) data;
    for(int i=0; i<outSize; i++)
    {
        fifoPtr[i] = dataPtr[i];
    }

    // start the SPI module
    SPI1CMD |= SPIBUSY;

    if (reading)
    {
        // wait for SPI write to complete
        while(SPI1CMD & SPIBUSY) {}

        // read data from the FIFO back to the application buffer
        for(int i=0; i<outSize; i++)
        {
            dataPtr[i] = fifoPtr[i];
        }
    }
#elif defined(PLATFORM_RP2350)
    const uint64_t pins = ((cs_mask & 1) ? _csMask[0] : 0) | ((cs_mask & 2) ? _csMask[1] : 0);
    gpio_clr_mask64(pins);

    if (reading)
        spi_write_read_blocking(_spi, data, data, size);
    else
        spi_write_blocking(_spi, data, size);

    gpio_set_mask64(pins);
#endif
}

#if defined(PLATFORM_RP2350)
void SPIExClass::begin(int sck, int miso, int mosi, int nss, int nss2, uint32_t frequency)
{
    // GPIO 0-7 and 16-23 route to spi0, 8-15 and 24-31 to spi1
    _spi = ((sck / 8) % 2) ? spi1 : spi0;
    spi_init(_spi, frequency);
    spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(sck, GPIO_FUNC_SPI);
    gpio_set_function(mosi, GPIO_FUNC_SPI);
    gpio_set_function(miso, GPIO_FUNC_SPI);
    gpio_pull_up(miso);

    const int cs[2] = {nss, nss2};
    for (int i = 0; i < 2; i++)
    {
        _csMask[i] = 0;
        if (cs[i] == UNDEF_PIN)
            continue;
        gpio_init(cs[i]);
        gpio_set_dir(cs[i], GPIO_OUT);
        gpio_put(cs[i], true);
        _csMask[i] = 1ULL << cs[i];
    }
}

void SPIExClass::end()
{
    if (_spi)
        spi_deinit(_spi);
}

void SPIExClass::transferBytes(const uint8_t *out, uint8_t *in, uint32_t size)
{
    if (in)
        spi_write_read_blocking(_spi, out, in, size);
    else
        spi_write_blocking(_spi, out, size);
}
#endif

#if defined(PLATFORM_ESP32_S3) || defined(PLATFORM_ESP32_C3)
SPIExClass SPIEx(FSPI);
#elif defined(PLATFORM_ESP32)
SPIExClass SPIEx(VSPI);
#else
SPIExClass SPIEx;
#endif
